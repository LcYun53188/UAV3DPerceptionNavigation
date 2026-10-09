"""Continuous EV writer for owned warehouse SITL; no command/setpoint publishers."""
import time
from px4_comm_bridge.pose_stream import FlightAlignedPoseStream
from px4_vio_real_fusion import RealPoseFusionAudit


class FlightPoseSession(RealPoseFusionAudit):
    stream_class=FlightAlignedPoseStream

    def __init__(self,*args,**kwargs):
        self.control_identity=None
        self.gateway_seen=False
        self.control_diagnostics=[]
        self.control_discovery_pending=False
        super().__init__(*args,**kwargs)

    def state_keywords(self):
        if self.control_discovery_pending:return dict(ground=False,armed=False)
        now=time.monotonic();ros=self.ros()
        for key,age in (('vehicle',.75),('land',1.5)):
            m=self.last.get(key)
            if (m is None or now-self.received[key]>age or not -.05<=ros-m.timestamp/1e6<=age
                    or len(self.node.get_publishers_info_by_topic(self.topics[key]))!=1):
                return dict(ground=False,armed=False)
        return dict(ground=self.last['vehicle'].arming_state==1 and self.last['land'].landed,
                    armed=self.last['vehicle'].arming_state==2)

    def controls_valid(self):
        endpoints=[self.node.get_publishers_info_by_topic('/px4_7/fmu/in/'+n) for n in self.control_names]
        actual=[[(e.node_name,e.node_namespace,bytes(e.endpoint_gid).hex()) for e in group] for group in endpoints]
        unknown=any(e.node_name=='_NODE_NAME_UNKNOWN_' or e.node_namespace=='_NODE_NAMESPACE_UNKNOWN_' for group in endpoints for e in group)
        if unknown:
            self.control_discovery_pending=True
            # Fast DDS can discover GIDs before participant node metadata. No
            # ground binding or EV emission is allowed during this startup gap.
            return not self.stream.bound and not self.gateway_seen and all(len(group)<=1 for group in endpoints)
        self.control_discovery_pending=False
        valid_names=all(len(group)<=1 and all(e.node_name=='px4_flight_gateway' and e.node_namespace=='/' for e in group) for group in endpoints)
        if not valid_names:
            self.control_diagnostics.append(dict(mono=time.monotonic(),reason='OWNER',actual=actual));return False
        if not all(endpoints):
            if self.gateway_seen:self.control_diagnostics.append(dict(mono=time.monotonic(),reason='LOST',actual=actual))
            return not self.gateway_seen
        identity=tuple(bytes(e[0].endpoint_gid).hex() for e in endpoints)
        if self.control_identity is None:self.control_identity=identity
        self.gateway_seen=True
        valid=identity==self.control_identity
        if not valid:self.control_diagnostics.append(dict(mono=time.monotonic(),reason='GID',expected=self.control_identity,actual=actual))
        return valid

    def flight_result(self):
        return dict(scope='continuous actual stereo/IMU EV for owned warehouse flight',
            source_fault=self.stream.fault,source_reason=self.stream.reason,alignment=self.stream.alignment.binding,
            quality_policy=self.stream.quality_policy,dropped_aligned_uncertainty=self.stream.dropped_uncertain,input_count=len(self.outputs),observed_count=len(self.echoes),fused_samples=dict(self.fused),
            graph_violations=self.graph_violations,control_diagnostics=self.control_diagnostics,endpoint_identity_modes=sorted(self.gid_modes),
            longest_ready_s=self.longest_ready_s,gate_reasons=dict(self.reasons),
            sole_gateway_observed=self.gateway_seen,last_sample=(None if self.last_sample is None else
                dict(sec=self.last_sample.sec,nanosec=self.last_sample.nanosec)))
