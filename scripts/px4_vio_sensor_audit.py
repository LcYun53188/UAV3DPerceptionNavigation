"""Observe physical simulated stereo/IMU and actual cuVSLAM outputs, never inject pose."""
from collections import Counter, deque
import math
import time
import numpy as np
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image,CameraInfo,Imu
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleStatus,VehicleLandDetected
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from rclpy.qos import qos_profile_sensor_data, qos_profile_default, QoSProfile, ReliabilityPolicy
from tf2_ros import StaticTransformBroadcaster
from px4_comm_bridge.vio_input import convert_vio
from vio_sensor_quality import sample_window, stereo_pairs, static_imu


def stamp(message):
    s = message.header.stamp
    return s.sec+s.nanosec/1e9


class SensorAudit:
    def __init__(self,node,profile,frames,*,allow_pose_fusion=False):
        self.allow_pose_fusion=allow_pose_fusion
        self.node,self.profile = node,profile
        self.last,self.receive,self.counts,self.topics = {},{},Counter(),{}
        self.records = deque(maxlen=5000)
        self.states = set()
        self.samples = {name:deque(maxlen=40000) for name in ('left','right','imu','odom','tracking')}
        self.imu_values = deque(maxlen=1500)
        self.tracking_states = deque(maxlen=150)
        self.tf = StaticTransformBroadcaster(node)
        transforms = []
        for name,frame in frames.items():
            t = TransformStamped()
            t.header.frame_id,t.child_frame_id = 'base_link',name
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z = frame['position']
            if name.endswith('optical'):
                t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w = -.5,.5,-.5,.5
            else: t.transform.rotation.w = 1.
            transforms.append(t)
        self.tf.sendTransform(transforms)
        entries = [('clock','/clock',Clock),('left','/vio/left/image',Image),('right','/vio/right/image',Image),
            ('left_info','/vio/left/camera_info',CameraInfo),('right_info','/vio/right/camera_info',CameraInfo),
            ('imu','/vio/imu',Imu),('odom','/visual_slam/tracking/odometry',Odometry),
            ('pose_cov','/visual_slam/tracking/vo_pose_covariance',PoseWithCovarianceStamped),
            ('tracking','/visual_slam/status',VisualSlamStatus),
            ('vehicle','/px4_7/fmu/out/vehicle_status_v1',VehicleStatus),
            ('land','/px4_7/fmu/out/vehicle_land_detected',VehicleLandDetected)]
        for key,topic,kind in entries:
            self.topics[key] = topic
            def receiver(name):
                def callback(message):
                    now = time.monotonic()
                    self.last[name],self.receive[name] = message,now
                    self.counts[name] += 1
                    if name in self.samples: self.samples[name].append(stamp(message))
                    if name == 'tracking': self.tracking_states.append((stamp(message),int(message.vo_state)))
                    if name == 'imu':
                        a,w = message.linear_acceleration,message.angular_velocity
                        self.imu_values.append([a.x,a.y,a.z,w.x,w.y,w.z])
                    if name == 'vehicle': self.states.add(int(message.arming_state))
                    if name == 'odom':
                        p = message.pose.pose.position
                        self.records.append(dict(mono=now,stamp=stamp(message),position=[p.x,p.y,p.z],
                            pose_variance=[float(message.pose.covariance[i*7]) for i in range(6)],
                            velocity_variance=[float(message.twist.covariance[i*7]) for i in range(6)]))
                return callback
            # This observer also records EKF telemetry. Five IMU slots only
            # cover 20 ms; retain source stamps across ordinary callback bursts.
            # Original gap/freshness acceptance remains unchanged.
            qos=(QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT) if key=='imu' else
                 qos_profile_default if key in ('left','right') else qos_profile_sensor_data)
            node.create_subscription(kind,topic,receiver(key),qos)

    def result(self):
        now = time.monotonic()
        ros = self.last['clock'].clock.sec+self.last['clock'].clock.nanosec/1e9 if 'clock' in self.last else 0.
        checks = {}
        for name in self.topics:
            checks['present:'+name] = self.counts[name] >= (20 if name in ('left','right','imu','odom','tracking') else 1)
            checks['writer:'+name] = len(self.node.get_publishers_info_by_topic(self.topics[name])) == 1
            max_age = 1.5 if name == 'land' else .75 if name == 'vehicle' else .2
            checks['fresh:'+name] = now-self.receive.get(name,0.) <= max_age
        images,calibration = {},{}
        for side in ('left','right'):
            if side not in self.last: continue
            m = self.last[side]
            images[side] = dict(width=m.width,height=m.height,encoding=m.encoding,step=m.step,
                frame=m.header.frame_id,stamp=stamp(m),byte_std=float(np.std(np.frombuffer(m.data,dtype=np.uint8))))
            checks['image:'+side] = (m.width==self.profile['width'] and m.height==self.profile['height']
                and m.encoding=='rgb8' and len(m.data)==m.step*m.height and images[side]['byte_std'] > 10.)
            checks['image_age:'+side] = -.05 <= ros-stamp(m) <= .2
            info = self.last.get(side+'_info')
            if info is not None:
                calibration[side] = dict(frame=info.header.frame_id,stamp=stamp(info),
                    width=info.width,height=info.height,distortion_model=info.distortion_model,
                    k=list(info.k),d=list(info.d),r=list(info.r),p=list(info.p))
                fx = m.width/(2*math.tan(self.profile['horizontal_fov_rad']/2))
                checks['calibration:'+side] = (info.header.frame_id==m.header.frame_id
                    and info.width==m.width and info.height==m.height and abs(info.k[0]-fx)<.01
                    and abs(info.k[4]-fx)<.01 and info.distortion_model=='plumb_bob'
                    and abs(info.k[2]-m.width/2)<.01 and abs(info.k[5]-m.height/2)<.01
                    and np.allclose(np.asarray(info.r).reshape(3,3),np.eye(3),atol=1e-8)
                    and all(abs(v)<1e-9 for v in info.d))
        quality = {name:sample_window(self.samples[name],ros,rate) for name,rate in (
            ('left',self.profile['image_rate_hz']),('right',self.profile['image_rate_hz']),
            ('imu',self.profile['imu_rate_hz']),('odom',self.profile['image_rate_hz']))}
        quality['stereo'] = stereo_pairs(self.samples['left'],self.samples['right'],ros)
        quality['static_imu'] = static_imu(list(self.imu_values))
        for name,value in quality.items(): checks['quality:'+name] = value['passed']
        checks['imu_frame'] = 'imu' in self.last and self.last['imu'].header.frame_id=='vio_imu'
        steady_states = [state for t,state in self.tracking_states if ros-5 <= t <= ros]
        checks['tracking_continuous'] = bool(steady_states) and all(s==1 for s in steady_states)
        tracking = self.last.get('tracking')
        checks['tracking_ok'] = tracking is not None and tracking.vo_state==1
        odom = self.last.get('odom')
        recent = [r for r in self.records if now-r['mono'] <= 5.]
        checks['tracking_window'] = len(recent)>=50 and recent[-1]['mono']-recent[0]['mono']>=4.
        drift = None
        if recent:
            drift = max(math.dist(r['position'],recent[0]['position']) for r in recent)
            checks['static_drift'] = math.isfinite(drift) and drift <= .15
        adapter = dict(accepted=False,reason='NO_ODOMETRY')
        if odom is not None:
            checks['odometry_age'] = -.05 <= ros-stamp(odom) <= .2
            checks['frames'] = odom.header.frame_id=='odom' and odom.child_frame_id=='base_link'
            try:
                convert_vio(odom,ros)
                adapter = dict(accepted=True,reason='')
            except ValueError as exc: adapter['reason'] = str(exc)
        checks['disarmed'] = self.states=={1}
        checks['landed'] = 'land' in self.last and self.last['land'].landed
        controls = {name:len(self.node.get_publishers_info_by_topic('/px4_7/fmu/in/'+name))
                    for name in ('vehicle_command','trajectory_setpoint','offboard_control_mode','vehicle_visual_odometry')}
        if self.allow_pose_fusion:
            checks['only_pose_fmu_input'] = controls['vehicle_visual_odometry']==1 and not any(
                controls[n] for n in ('vehicle_command','trajectory_setpoint','offboard_control_mode'))
        else:checks['no_fmu_inputs'] = not any(controls.values())
        return dict(passed=bool(checks) and all(checks.values()),checks=checks,
            scope=('disarmed same-aircraft sensors with explicitly enabled pose EV audit' if self.allow_pose_fusion else
                'disarmed physical simulated stereo/IMU -> actual cuVSLAM; no EV emission or flight acceptance'),
            counts=dict(self.counts),images=images,calibration=calibration,quality=quality,
            static_drift_m=drift,raw_odometry_numeric_conversion=adapter,
            px4_source_contract_verified=False,
            observer_imu_qos=dict(depth=100,reliability='BEST_EFFORT'),
            arming_states=sorted(self.states),control_publishers=controls,
            last_odometry=recent[-1] if recent else None,
            sdk_pose_covariance=list(self.last['pose_cov'].pose.covariance) if 'pose_cov' in self.last else None)
