"""Synthetic EV input -> actual PX4 EKF fusion audit, disarmed only.

No camera/VIO algorithm claim; fixed independent pose, never PX4 estimate feedback.
"""
from collections import Counter
import time
import math
import uuid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import (VehicleOdometry, EstimatorStatusFlags, EstimatorSelectorStatus,
                         EstimatorAidSource1d, EstimatorAidSource2d, EstimatorAidSource3d, VehicleLocalPosition, VehicleStatus, VehicleLandDetected)
from uav_nav_interfaces.msg import VioStatus
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from px4_comm_bridge.vio_input import convert_vio
from uav_mission.vio_gate import VioGate
from px4_comm_bridge.pose_fusion import PoseAlignment,convert_aligned_pose,FRAME


class VisionFusionAudit:
    aids = ('ev_pos','ev_hgt','ev_vel','ev_yaw')
    pose_only = False
    profile = 'full_odometry'

    def __init__(self, node, fusion_profile='full_odometry'):
        self.node = node
        self.calibration = 'a'*64  # Explicit test identity, NOT a device calibration.
        self.session = 'synthetic-ev:'+str(uuid.uuid4())
        self.profile = fusion_profile
        self.pose_only = fusion_profile == 'aligned_pose_v1'
        self.gate = VioGate(self.calibration,fusion_profile=fusion_profile)
        self.aids = self.gate.aids
        self.alignment = PoseAlignment([0.,0.,0.],0.,'b'*64) if self.pose_only else None
        self.alignment_samples = []
        self.unknown_velocity_count = 0
        self.observed_inputs = []
        self.local_before_stop = {}
        self.counts, self.fused_counts, self.reasons = Counter(), Counter(), Counter()
        self.last = {}
        self.ready_count = 0
        self.next_sample = 0.
        self.inject = True
        self.stop_snapshot = None
        self.drained_snapshot = None
        self.stopped_at = None
        self.ready_before_stop = False
        self.first_stop_rejection = None
        self.input_count = 0
        self.aiding_before_stop = {}
        self.topics = {}
        self.pub = node.create_publisher(VehicleOdometry, '/px4_7/fmu/in/vehicle_visual_odometry', 10)
        self.status_pub = node.create_publisher(VioStatus, '/uav/vio/status', 10)
        entries = [('source','/uav/vio/status',VioStatus),
            ('flags','/px4_7/fmu/out/estimator_status_flags',EstimatorStatusFlags),
            ('selector','/px4_7/fmu/out/estimator_selector_status',EstimatorSelectorStatus),
            ('ev_pos','/px4_7/fmu/out/estimator_aid_src_ev_pos',EstimatorAidSource2d),
            ('ev_hgt','/px4_7/fmu/out/estimator_aid_src_ev_hgt',EstimatorAidSource1d),
            ('ev_vel','/px4_7/fmu/out/estimator_aid_src_ev_vel',EstimatorAidSource3d),
            ('ev_yaw','/px4_7/fmu/out/estimator_aid_src_ev_yaw',EstimatorAidSource1d)]
        if self.pose_only:
            entries += [('local','/px4_7/fmu/out/vehicle_local_position',VehicleLocalPosition),
                ('vehicle','/px4_7/fmu/out/vehicle_status',VehicleStatus),
                ('land','/px4_7/fmu/out/vehicle_land_detected',VehicleLandDetected)]
        if self.pose_only:
            node.create_subscription(VehicleOdometry,'/px4_7/fmu/in/vehicle_visual_odometry',
                lambda m:self.observed_inputs.append(m),10)
        for name,topic,kind in entries:
            version = getattr(kind,'MESSAGE_VERSION',0)
            topic += f'_v{version}' if version else ''
            self.topics[name] = topic
            def receiver(key):
                def receive(message):
                    self.gate.receive(key,message,time.monotonic())
                    self.last[key] = message
                    self.counts[key] += 1
                    if getattr(message,'fused',False): self.fused_counts[key] += 1
                return receive
            node.create_subscription(kind,topic,receiver(name),qos_profile_sensor_data)

    def writers(self):
        return {name:len(self.node.get_publishers_info_by_topic(topic)) for name,topic in self.topics.items()}

    def tick(self, clock):
        now = time.monotonic()
        if clock is None: return
        ros = clock.sec+clock.nanosec/1e9
        if ros <= 0: return
        if self.inject and now >= self.next_sample:
            message = Odometry()
            message.header.stamp = clock
            message.header.frame_id,message.child_frame_id = 'odom','base_link'
            message.pose.pose.orientation.w = 1.
            for index in range(6):
                message.pose.covariance[7*index] = .01
                message.twist.covariance[7*index] = .01
            converted = None
            if self.pose_only:
                source = PoseWithCovarianceStamped()
                source.header = message.header
                source.pose = message.pose
                source.pose.pose.position.x,source.pose.pose.position.y,source.pose.pose.position.z = 2.,-1.,.3
                source.pose.pose.orientation.w,source.pose.pose.orientation.z = math.cos(.3),math.sin(.3)
                identity = dict(source_session=self.session,calibration_id=self.calibration)
                if self.alignment.binding is None:
                    self.alignment_samples.append(source)
                    if (ros-self.alignment_samples[0].header.stamp.sec-self.alignment_samples[0].header.stamp.nanosec/1e9 >= 2.
                            and 'vehicle' in self.last and 'land' in self.last):
                        self.alignment.bind(self.alignment_samples,ros,**identity,
                            disarmed=self.last['vehicle'].arming_state==1,landed=self.last['land'].landed)
                else:
                    aligned = self.alignment.apply(source,ros,**identity)
                    converted = convert_aligned_pose(aligned,ros)
                    self.unknown_velocity_count += int(converted.velocity_frame==VehicleOdometry.VELOCITY_FRAME_UNKNOWN
                        and all(math.isnan(v) for n in ('velocity','angular_velocity','velocity_variance') for v in getattr(converted,n)))
            else:
                converted = convert_vio(message,ros)
            if converted is not None:
                self.pub.publish(converted)
                session = self.session+':'+self.alignment.alignment_id if self.pose_only else self.session
                status = VioStatus(localization_session=session,calibration_id=self.calibration,valid=True)
                status.header = message.header
                if self.pose_only: status.header.frame_id = FRAME
                status.sample_stamp = clock
                self.status_pub.publish(status)
                self.input_count += 1
            self.next_sample = now+1/30
        if not self.inject and self.drained_snapshot is None and now-self.stopped_at >= 1.:
            self.drained_snapshot = {name:int(self.last[name].time_last_fuse)
                                     for name in self.aids if name in self.last}
        ready = self.gate.ready(now,ros,self.writers())
        self.reasons[self.gate.reason] += 1
        if not self.inject and not ready and self.first_stop_rejection is None:
            self.first_stop_rejection = dict(reason=self.gate.reason,after_stop_s=now-self.stopped_at)
        if ready and self.inject:
            self.ready_count += 1
            self.ready_before_stop = True

    def stop_input(self):
        self.ready_before_stop = self.gate.reason == 'READY'
        flags = self.last.get('flags')
        if flags is not None:
            self.aiding_before_stop = {name:bool(getattr(flags,name)) for name in
                ('cs_gnss_pos','cs_gnss_vel','cs_gnss_yaw','cs_gps_hgt','cs_mag_hdg','cs_mag_3d',
                 'cs_mag','cs_opt_flow','cs_baro_hgt','cs_ev_pos','cs_ev_hgt','cs_ev_vel','cs_ev_yaw')}
        if self.pose_only and 'local' in self.last:
            m=self.last['local']
            self.local_before_stop={n:getattr(m,n) for n in ('timestamp','timestamp_sample','x','y','z','vx','vy','vz',
                'xy_valid','z_valid','v_xy_valid','v_z_valid','heading_good_for_control','heading','heading_var',
                'evh','evv','dead_reckoning','xy_reset_counter','z_reset_counter','vxy_reset_counter','vz_reset_counter','heading_reset_counter')}
        if self.pose_only and flags is not None:
            self.aiding_before_stop.update({n:bool(getattr(flags,n)) for n in ('cs_rng_hgt','cs_aux_gpos')})
        self.stopped_at = time.monotonic()
        self.stop_snapshot = {name:int(self.last[name].time_last_fuse)
                              for name in self.aids if name in self.last}
        self.inject = False

    def result(self):
        controls = {topic:len(self.node.get_publishers_info_by_topic('/px4_7/fmu/in/'+topic))
                    for topic in ('vehicle_command','trajectory_setpoint','offboard_control_mode')}
        last_fuse = {name:int(self.last[name].time_last_fuse)
                     for name in self.aids if name in self.last}
        stopped = set(self.drained_snapshot or {}) == set(self.aids) and all(
            last_fuse.get(name)==stamp for name,stamp in self.drained_snapshot.items())
        independent = bool(self.aiding_before_stop) and not any(self.aiding_before_stop[n] for n in
            ('cs_gnss_pos','cs_gnss_vel','cs_gnss_yaw','cs_gps_hgt','cs_mag_hdg','cs_mag_3d','cs_mag','cs_opt_flow'))
        def finite_json(value):
            if isinstance(value,dict): return {k:finite_json(v) for k,v in value.items()}
            if isinstance(value,(list,tuple)): return [finite_json(v) for v in value]
            if isinstance(value,float) and not math.isfinite(value): return None
            return value
        terminal_rejected = self.gate.reason == 'VIO_TELEMETRY_STALE:source'
        if self.pose_only:
            terminal_rejected = self.gate.reason in ('VIO_TELEMETRY_STALE:source','VIO_EKF_LOCAL_RESET')
            terminal_rejected &= bool(self.first_stop_rejection) and self.first_stop_rejection['after_stop_s'] <= .5
            terminal_rejected &= bool(self.first_stop_rejection) and self.first_stop_rejection['reason'] in (
                'VIO_SOURCE_INVALID','VIO_TELEMETRY_STALE:source')
        passed = (independent and self.ready_before_stop and self.ready_count >= 5 and all(self.fused_counts[n] >= 5 for n in self.aids)
                  and terminal_rejected and stopped and not any(controls.values()))
        if self.pose_only:
            passed &= self.unknown_velocity_count == self.input_count > 0
            passed &= len(self.observed_inputs) >= .95*self.input_count
            passed &= all(m.velocity_frame==VehicleOdometry.VELOCITY_FRAME_UNKNOWN
                and all(math.isnan(v) for n in ('velocity','angular_velocity','velocity_variance') for v in getattr(m,n))
                for m in self.observed_inputs)
            passed &= len(self.node.get_publishers_info_by_topic('/px4_7/fmu/in/vehicle_visual_odometry'))==1
            passed &= not any(self.aiding_before_stop.get(n,True) for n in ('cs_rng_hgt','cs_aux_gpos'))
            passed &= not self.aiding_before_stop.get('cs_ev_vel',True) and self.fused_counts['ev_vel']==0
        return dict(passed=bool(passed),fusion_profile=self.profile,
            alignment=self.alignment.binding if self.pose_only else None,
            unknown_velocity_samples=self.unknown_velocity_count if self.pose_only else 0,
            first_stop_rejection=self.first_stop_rejection if self.pose_only else None,
            local_before_stop=self.local_before_stop if self.pose_only else None,
            observed_input_count=len(self.observed_inputs) if self.pose_only else None,
            last_observed_input=finite_json(message_to_ordereddict(self.observed_inputs[-1])) if self.pose_only and self.observed_inputs else None,scope='synthetic external vision -> actual PX4 EKF, disarmed; NOT camera/VIO acceptance',
            other_aiding_before_stop=self.aiding_before_stop,nonfinite_telemetry_encoding="null",
            input_count=self.input_count,received=dict(self.counts),fused_samples=dict(self.fused_counts),
            ready_count=self.ready_count,reasons=dict(self.reasons),final_gate_reason=self.gate.reason,
            ready_before_stop=self.ready_before_stop,last_fuse_at_stop=self.stop_snapshot,
            last_fuse_after_drain=self.drained_snapshot,last_fuse_at_end=last_fuse,fusion_stopped=stopped,writers=self.writers(),control_publishers=controls,
            last_messages={name:finite_json(message_to_ordereddict(m)) for name,m in self.last.items()})
