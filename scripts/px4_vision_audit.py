"""Synthetic EV input -> actual PX4 EKF fusion audit, disarmed only.

No camera/VIO algorithm claim; fixed independent pose, never PX4 estimate feedback.
"""
from collections import Counter
import time
import math
import uuid
from nav_msgs.msg import Odometry
from px4_msgs.msg import (VehicleOdometry, EstimatorStatusFlags, EstimatorSelectorStatus,
                         EstimatorAidSource1d, EstimatorAidSource2d, EstimatorAidSource3d)
from uav_nav_interfaces.msg import VioStatus
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.convert import message_to_ordereddict
from px4_comm_bridge.vio_input import convert_vio
from uav_mission.vio_gate import VioGate


class VisionFusionAudit:
    def __init__(self, node):
        self.node = node
        self.calibration = 'a'*64  # Explicit test identity, NOT a device calibration.
        self.session = 'synthetic-ev:'+str(uuid.uuid4())
        self.gate = VioGate(self.calibration)
        self.counts, self.fused_counts, self.reasons = Counter(), Counter(), Counter()
        self.last = {}
        self.ready_count = 0
        self.next_sample = 0.
        self.inject = True
        self.stop_snapshot = None
        self.drained_snapshot = None
        self.stopped_at = None
        self.ready_before_stop = False
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
            self.pub.publish(convert_vio(message,ros))
            status = VioStatus(localization_session=self.session,calibration_id=self.calibration,valid=True)
            status.header = message.header
            status.sample_stamp = clock
            self.status_pub.publish(status)
            self.input_count += 1
            self.next_sample = now+1/30
        if not self.inject and self.drained_snapshot is None and now-self.stopped_at >= 1.:
            self.drained_snapshot = {name:int(self.last[name].time_last_fuse)
                                     for name in ('ev_pos','ev_hgt','ev_vel','ev_yaw') if name in self.last}
        ready = self.gate.ready(now,ros,self.writers())
        self.reasons[self.gate.reason] += 1
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
        self.stopped_at = time.monotonic()
        self.stop_snapshot = {name:int(self.last[name].time_last_fuse)
                              for name in ('ev_pos','ev_hgt','ev_vel','ev_yaw') if name in self.last}
        self.inject = False

    def result(self):
        controls = {topic:len(self.node.get_publishers_info_by_topic('/px4_7/fmu/in/'+topic))
                    for topic in ('vehicle_command','trajectory_setpoint','offboard_control_mode')}
        last_fuse = {name:int(self.last[name].time_last_fuse)
                     for name in ('ev_pos','ev_hgt','ev_vel','ev_yaw') if name in self.last}
        stopped = set(self.drained_snapshot or {}) == {'ev_pos','ev_hgt','ev_vel','ev_yaw'} and all(
            last_fuse.get(name)==stamp for name,stamp in self.drained_snapshot.items())
        independent = bool(self.aiding_before_stop) and not any(self.aiding_before_stop[n] for n in
            ('cs_gnss_pos','cs_gnss_vel','cs_gnss_yaw','cs_gps_hgt','cs_mag_hdg','cs_mag_3d','cs_mag','cs_opt_flow'))
        def finite_json(value):
            if isinstance(value,dict): return {k:finite_json(v) for k,v in value.items()}
            if isinstance(value,(list,tuple)): return [finite_json(v) for v in value]
            if isinstance(value,float) and not math.isfinite(value): return None
            return value
        passed = (independent and self.ready_before_stop and self.ready_count >= 5 and all(self.fused_counts[n] >= 5 for n in ('ev_pos','ev_hgt','ev_vel','ev_yaw'))
                  and self.gate.reason == 'VIO_TELEMETRY_STALE:source' and stopped and not any(controls.values()))
        return dict(passed=passed,scope='synthetic external vision -> actual PX4 EKF, disarmed; NOT camera/VIO acceptance',
            other_aiding_before_stop=self.aiding_before_stop,nonfinite_telemetry_encoding="null",
            input_count=self.input_count,received=dict(self.counts),fused_samples=dict(self.fused_counts),
            ready_count=self.ready_count,reasons=dict(self.reasons),final_gate_reason=self.gate.reason,
            ready_before_stop=self.ready_before_stop,last_fuse_at_stop=self.stop_snapshot,
            last_fuse_after_drain=self.drained_snapshot,last_fuse_at_end=last_fuse,fusion_stopped=stopped,writers=self.writers(),control_publishers=controls,
            last_messages={name:finite_json(message_to_ordereddict(m)) for name,m in self.last.items()})
