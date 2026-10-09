"""Owned, disarmed same-aircraft cuVSLAM -> PX4 EV audit. No synthetic inputs.

The supervisor verifies binaries, calibration, scene and exclusive instance before
constructing this object. Nothing here can arm or publish flight controls. Ground
state is continuously required, including after launch alignment.
"""
from collections import Counter,OrderedDict
import math
import time
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_msgs.msg import (VehicleOdometry,VehicleStatus,VehicleLandDetected,VehicleLocalPosition,
    EstimatorStatusFlags,EstimatorSelectorStatus,EstimatorAidSource1d,EstimatorAidSource2d,EstimatorAidSource3d)
from uav_nav_interfaces.msg import VioStatus
from rclpy.qos import qos_profile_sensor_data
from rclpy.clock import Clock,ClockType
from rosidl_runtime_py.convert import message_to_ordereddict
from px4_comm_bridge.pose_stream import AlignedPoseStream,stamp_ns
from px4_comm_bridge.pose_fusion import FRAME
from px4_comm_bridge.vio_input import stamp_s
from px4_comm_bridge.source_timing import SourceTiming
from uav_mission.vio_gate import VioGate


def json_message(message):
    def clean(v):
        if isinstance(v,dict):return {k:clean(x) for k,x in v.items()}
        if isinstance(v,(tuple,list)):return [clean(x) for x in v]
        if isinstance(v,float) and not math.isfinite(v):return None
        return v
    return clean(message_to_ordereddict(message))


class RealPoseFusionAudit:
    stream_class=AlignedPoseStream
    input_topics=('/uav/vio/pose','/uav/vio/pose_status')
    control_names=('vehicle_command','trajectory_setpoint','offboard_control_mode')

    def __init__(self,node,calibration,anchor,anchor_id,*,quality_policy="strict"):
        self.node=node
        self.stream=self.stream_class(calibration,anchor['position_enu'],anchor['yaw_enu'],anchor_id,quality_policy=quality_policy)
        self.gate=VioGate(calibration,fusion_profile='aligned_pose_v1')
        self.pending=OrderedDict();self.statuses=OrderedDict()
        self.gid_modes=set();self.graph_violations=[];self.timing=SourceTiming()
        self.last={};self.received={};self.topics={}
        self.history=[];self.outputs=[];self.echoes=[];self.reasons=Counter();self.fused=Counter()
        self.ready_count=0;self.ready_since=None;self.longest_ready_s=0.
        self.last_sample=None;self.stopped_at=None;self.first_rejection=None;self.before_stop=None;self.drained=None
        self.ev_topic='/px4_7/fmu/in/vehicle_visual_odometry'
        self.output=node.create_publisher(VehicleOdometry,self.ev_topic,10)
        self.status_pub=node.create_publisher(VioStatus,'/uav/vio/status',10)
        node.create_subscription(PoseWithCovarianceStamped,self.input_topics[0],self.on_pose,10)
        node.create_subscription(VioStatus,self.input_topics[1],self.on_status,10)
        node.create_subscription(VehicleOdometry,self.ev_topic,
            lambda m:self.echoes.append(dict(mono=time.monotonic(),message=m)),10)
        entries=[('source','/uav/vio/status',VioStatus),('vehicle','/px4_7/fmu/out/vehicle_status',VehicleStatus),
            ('land','/px4_7/fmu/out/vehicle_land_detected',VehicleLandDetected),
            ('local','/px4_7/fmu/out/vehicle_local_position',VehicleLocalPosition),
            ('flags','/px4_7/fmu/out/estimator_status_flags',EstimatorStatusFlags),
            ('selector','/px4_7/fmu/out/estimator_selector_status',EstimatorSelectorStatus),
            ('ev_pos','/px4_7/fmu/out/estimator_aid_src_ev_pos',EstimatorAidSource2d),
            ('ev_hgt','/px4_7/fmu/out/estimator_aid_src_ev_hgt',EstimatorAidSource1d),
            ('ev_vel','/px4_7/fmu/out/estimator_aid_src_ev_vel',EstimatorAidSource3d),
            ('ev_yaw','/px4_7/fmu/out/estimator_aid_src_ev_yaw',EstimatorAidSource1d)]
        for name,topic,kind in entries:
            version=getattr(kind,'MESSAGE_VERSION',0)
            if version:topic+=f'_v{version}'
            self.topics[name]=topic
            def receiver(key):
                def callback(m):
                    now=time.monotonic();self.last[key]=m;self.received[key]=now
                    self.gate.receive(key,m,now)
                    if getattr(m,'fused',False):self.fused[key]+=1
                    # Keep received messages; serialize after stopping processes. Full
                    # telemetry conversion in this callback competes with 250 Hz IMU.
                    self.history.append(dict(mono=now,name=key,message=m))
                return callback
            node.create_subscription(kind,topic,receiver(name),qos_profile_sensor_data)
        node.create_timer(.02,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def ros(self):return self.node.get_clock().now().nanoseconds/1e9

    def ground(self):
        now=time.monotonic();ros=self.ros()
        for key,age in (('vehicle',.75),('land',1.5)):
            m=self.last.get(key)
            if (m is None or now-self.received[key]>age or not -.05<=ros-m.timestamp/1e6<=age
                    or len(self.node.get_publishers_info_by_topic(self.topics[key]))!=1):return False
        return self.last['vehicle'].arming_state==1 and self.last['land'].landed

    def state_keywords(self):return dict(ground=self.ground())

    def controls_valid(self):
        return not any(self.node.get_publishers_info_by_topic('/px4_7/fmu/in/'+n) for n in self.control_names)

    def gid(self,topic,info=None):
        endpoints=self.node.get_publishers_info_by_topic(topic)
        if len(endpoints)!=1:raise ValueError('VIO_WRITER_COUNT')
        gid=bytes(endpoints[0].endpoint_gid).hex()
        if info is not None:
            actual_gid=info.get('publisher_gid') if isinstance(info,dict) else getattr(info,'publisher_gid',None)
            # Jazzy rclpy 7.1.11 omits the RMW publisher GID. In this owned
            # local audit bind the unique graph endpoint plus original source
            # UUID/reset/calibration. Report this limitation; it is not authentication.
            self.gid_modes.add('callback_and_graph' if actual_gid is not None else 'unique_graph_only')
            if actual_gid is not None and bytes(actual_gid).hex()!=gid:
                raise ValueError('VIO_PUBLISHER_CHANGED')
        return gid

    def on_pose(self,m,info):
        self.timing.record('pose_rx',self.ros(),stamp_s(m.header.stamp),info)
        if self.stream.fault:return
        try:
            gid=self.gid(self.input_topics[0],info);key=stamp_ns(m.header.stamp)
            if key in self.pending:raise ValueError('VIO_TIME_DISCONTINUITY')
            if len(self.pending)>=20:raise ValueError('VIO_PAIR_QUEUE_OVERFLOW')
            self.pending[key]=(m,time.monotonic(),gid)
            self.drain()
        except ValueError as exc:self.stream.reject(str(exc))

    def on_status(self,m,info):
        self.timing.record('status_rx',self.ros(),stamp_s(m.sample_stamp),info)
        if self.stream.fault:return
        try:
            gid=self.gid(self.input_topics[1],info)
            # A bad callback cannot be hidden by a later healthy heartbeat.
            if not m.valid or m.reason:raise ValueError('VIO_SOURCE_INVALID:'+m.reason)
            if self.stream.identity is not None and (m.localization_session,m.calibration_id,m.reset_counter)!=self.stream.identity[:3]:
                raise ValueError('VIO_SOURCE_IDENTITY_CHANGED')
            self.statuses[stamp_ns(m.sample_stamp)]=(m,time.monotonic(),gid)
            while len(self.statuses)>100:self.statuses.popitem(last=False)
            self.drain()
        except ValueError as exc:self.stream.reject(str(exc))

    def drain(self):
        for key in sorted(self.pending):
            pose,received,pose_gid=self.pending[key]
            if time.monotonic()-received>.2:
                del self.pending[key];self.stream.reject('VIO_PAIR_STALE')
            elif key in self.statuses:
                del self.pending[key]
                status,status_received,status_gid=self.statuses[key]
                if time.monotonic()-status_received>.2:
                    self.stream.reject('VIO_PAIR_STALE');continue
                previous=self.stream.alignment.last_stamp if self.stream.bound else None
                result=self.stream.accept(pose,status,self.ros(),time.monotonic(),pose_gid=pose_gid,
                    status_gid=status_gid,**self.state_keywords())
                if self.stream.fault:
                    self.timing.record('accept_fault',self.ros(),stamp_s(pose.header.stamp),reason=self.stream.fault,previous=previous)
                if result is not None:
                    self.timing.record('accepted',self.ros(),stamp_s(pose.header.stamp),previous=previous)
                    aligned,converted=result
                    # Graph ownership is checked immediately before the only FMU write.
                    if (len(self.node.get_publishers_info_by_topic(self.ev_topic))!=1
                            or len(self.node.get_publishers_info_by_topic('/uav/vio/status'))!=1
                            or not self.controls_valid()):
                        self.stream.reject('VIO_OUTPUT_WRITER_COUNT');return
                    self.output.publish(converted);self.last_sample=aligned.header.stamp
                    self.outputs.append(dict(mono=time.monotonic(),aligned=aligned,ev=converted))
            else:break
            if self.stream.fault:self.pending.clear();break

    def writers(self):
        return {key:len(self.node.get_publishers_info_by_topic(topic)) for key,topic in self.topics.items()}

    def tick(self):
        ros=self.ros();now=time.monotonic()
        if ros<=0:return
        # Process exact, ready source pairs before timing out the previous
        # accepted sample. Incoming data must pass all admission checks first.
        self.drain()
        old_fault=self.stream.fault
        self.stream.check(ros,now,**self.state_keywords())
        if self.stream.fault and not old_fault:
            self.timing.record('watchdog_fault',ros,self.stream.alignment.last_stamp if self.stream.bound else None,reason=self.stream.fault)
        graph={n:len(self.node.get_publishers_info_by_topic('/px4_7/fmu/in/'+n)) for n in self.control_names}
        if (not self.controls_valid() or len(self.node.get_publishers_info_by_topic(self.ev_topic))!=1
                or len(self.node.get_publishers_info_by_topic('/uav/vio/status'))!=1):
            self.graph_violations.append(dict(mono=now,controls=graph))
            self.stream.reject('VIO_OUTPUT_WRITER_COUNT')
        if self.stream.bound and not self.stream.fault:
            try:
                gids=tuple(self.gid(t) for t in self.input_topics)
                if gids!=self.stream.identity[3:]:raise ValueError('VIO_PUBLISHER_CHANGED')
                if len(self.node.get_publishers_info_by_topic(self.ev_topic))!=1:raise ValueError('VIO_OUTPUT_WRITER_COUNT')
            except ValueError as exc:self.stream.reject(str(exc))
        self.drain()
        session=''
        if self.stream.bound:session=self.stream.identity[0]+':'+self.stream.alignment.alignment_id
        status=VioStatus(localization_session=session,calibration_id=self.stream.calibration,
            reset_counter=self.stream.identity[2] if self.stream.identity else 0,
            valid=bool(self.last_sample is not None and not self.stream.reason and not self.stream.fault),
            reason=self.stream.fault or self.stream.reason)
        status.header.frame_id=FRAME;status.header.stamp=self.node.get_clock().now().to_msg()
        if self.last_sample is not None:status.sample_stamp=self.last_sample
        self.status_pub.publish(status)
        ready=self.gate.ready(now,ros,self.writers());self.reasons[self.gate.reason]+=1
        if ready:
            self.ready_count+=1
            if self.ready_since is None:self.ready_since=now
            self.longest_ready_s=max(self.longest_ready_s,now-self.ready_since)
        else:self.ready_since=None
        if self.stopped_at is not None:
            if not ready and self.first_rejection is None:
                self.first_rejection=dict(after_stop_s=now-self.stopped_at,reason=self.gate.reason)
            if self.drained is None and now-self.stopped_at>=1.:
                self.drained=self.last_fuse()

    def last_fuse(self):
        return {n:int(self.last[n].time_last_fuse) for n in self.gate.aids if n in self.last}

    def mark_stop(self):
        self.before_stop=dict(ready=self.gate.reason=='READY',source_fault=self.stream.fault,
            source_count=len(self.outputs),last_fuse=self.last_fuse(),local=json_message(self.last['local']) if 'local' in self.last else None,
            flags=json_message(self.last['flags']) if 'flags' in self.last else None)
        self.stopped_at=time.monotonic()

    def result(self):
        controls={n:len(self.node.get_publishers_info_by_topic('/px4_7/fmu/in/'+n)) for n in self.control_names}
        first=self.first_rejection or {}
        checks=dict(bound=self.stream.bound,ready_before_stop=bool(self.before_stop and self.before_stop['ready']),
            ready_window=self.longest_ready_s>=5.,emitted=len(self.outputs)>=100,
            actual_fusion=all(self.fused[n]>=50 for n in self.gate.aids),no_velocity_fusion=self.fused['ev_vel']==0,
            dds_echo=len(self.echoes)>=.95*len(self.outputs)>0,
            unknown_velocity=bool(self.echoes) and all(e['message'].velocity_frame==0 and all(math.isnan(v)
                for n in ('velocity','velocity_variance','angular_velocity') for v in getattr(e['message'],n)) for e in self.echoes),
            rejected_promptly=bool(first) and first['after_stop_s']<=.5 and first['reason'] in
                ('VIO_SOURCE_LOST','VIO_SOURCE_INVALID','VIO_TELEMETRY_STALE:source'),
            source_retired=bool(self.stream.fault),gate_retired=bool(self.gate.fault),
            no_output_after_drain=self.stopped_at is not None and not any(o['mono']>self.stopped_at+.5 for o in self.outputs),
            fusion_stopped=self.drained is not None and len(self.drained)==3 and self.drained==self.last_fuse(),
            no_flight_controls=not any(controls.values()) and not self.graph_violations,sole_ev_writer=len(self.node.get_publishers_info_by_topic(self.ev_topic))==1,
            disarmed_landed=self.ground())
        return dict(passed=all(checks.values()),checks=checks,scope='actual same-aircraft simulated stereo/IMU -> cuVSLAM -> PX4 EV, disarmed only',
            stop_request_mono=self.stopped_at,graph_violations=self.graph_violations,
            endpoint_identity_modes=sorted(self.gid_modes),
            alignment=self.stream.alignment.binding,before_stop=self.before_stop,first_stop_rejection=self.first_rejection,
            source_fault=self.stream.fault,source_reason=self.stream.reason,gate_reason=self.gate.reason,
            input_count=len(self.outputs),observed_count=len(self.echoes),fused_samples=dict(self.fused),
            longest_ready_s=self.longest_ready_s,gate_reasons=dict(self.reasons),last_fuse_after_drain=self.drained,
            last_fuse_at_end=self.last_fuse(),control_publishers=controls)

    def evidence(self):
        """Call after retiring owned processes, away from sensor callbacks."""
        return {
            'fusion-telemetry.json':[dict(e,message=json_message(e['message'])) for e in self.history],
            'fusion-dds-echoes.json':[dict(e,message=json_message(e['message'])) for e in self.echoes],
            'fusion-outputs.json':[dict(e,aligned=json_message(e['aligned']),ev=json_message(e['ev'])) for e in self.outputs]}
