#!/usr/bin/env python3
"""Ground-only source selector attached to owned joint SITL; no flight publishers."""
import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from px4_msgs.msg import (VehicleStatus,VehicleLandDetected,VehicleLocalPosition,
    EstimatorStatusFlags,EstimatorSelectorStatus,EstimatorAidSource1d,EstimatorAidSource2d)
from uav_nav_interfaces.msg import VioStatus,LocalizationSelection
from uav_nav_interfaces.srv import SelectLocalization
from unique_identifier_msgs.msg import UUID
from uav_mission.localization_selector import LocalizationSelector
from uav_mission.sitl_fusion_parameters import SitlFusionParameters

ROOT=Path(__file__).resolve().parents[1]
ENTRIES=[('source','/uav/vio/status',VioStatus),
    ('flags','/px4_7/fmu/out/estimator_status_flags',EstimatorStatusFlags),
    ('selector','/px4_7/fmu/out/estimator_selector_status',EstimatorSelectorStatus),
    ('local','/px4_7/fmu/out/vehicle_local_position',VehicleLocalPosition),
    ('ev_pos','/px4_7/fmu/out/estimator_aid_src_ev_pos',EstimatorAidSource2d),
    ('ev_hgt','/px4_7/fmu/out/estimator_aid_src_ev_hgt',EstimatorAidSource1d),
    ('ev_yaw','/px4_7/fmu/out/estimator_aid_src_ev_yaw',EstimatorAidSource1d),
    ('optical_flow','/px4_7/fmu/out/estimator_aid_src_optical_flow',EstimatorAidSource2d),
    ('rng_hgt','/px4_7/fmu/out/estimator_aid_src_rng_hgt',EstimatorAidSource1d),
    ('vehicle','/px4_7/fmu/out/vehicle_status',VehicleStatus),
    ('land','/px4_7/fmu/out/vehicle_land_detected',VehicleLandDetected)]


class SelectionSession(Node):
    def __init__(self, evidence):
        super().__init__('ground_localization_selector')
        self.evidence=Path(evidence)
        self.manifest=json.loads((self.evidence/'manifest.json').read_text())
        self.instance=self.manifest['run_id']
        calibration=self.manifest['calibration_id']
        self.samples,self.received,self.topics={}, {}, {}
        self.selection_pub=self.create_publisher(LocalizationSelection,'/uav/localization/selection',10)
        backend=SitlFusionParameters(ROOT,self.evidence,self.ground_safe)
        self.selector=LocalizationSelector(calibration,backend)
        for name,topic,kind in ENTRIES:
            version=getattr(kind,'MESSAGE_VERSION',0)
            topic+=f'_v{version}' if version else ''
            self.topics[name]=topic
            def callback(name):
                def receive(message):
                    now=time.monotonic()
                    self.samples[name],self.received[name]=message,now
                    if name not in ('vehicle','land'):self.selector.receive(name,message,now)
                return receive
            self.create_subscription(kind,topic,callback(name),qos_profile_sensor_data)
        self.create_service(SelectLocalization,'/uav/localization/select',self.select)
        self.create_timer(.05,self.tick)
        self.history=[];self.last_record=0.

    def ground_safe(self):
        now=time.monotonic();ros=self.get_clock().now().nanoseconds/1e9
        if hasattr(self,'selector') and self.selector.fault:return False
        for name,age in [('vehicle',.75),('land',1.2),('local',.5)]:
            message=self.samples.get(name)
            if (message is None or now-self.received[name]>age or not -.05<=ros-message.timestamp/1e6<=age or
                    len(self.get_publishers_info_by_topic(self.topics[name]))!=1):return False
        s,p,land=(self.samples[name] for name in ('vehicle','local','land'))
        if not (s.system_id==8 and s.component_id==1 and s.arming_state==1 and not s.failsafe and
                land.landed and all(math.isfinite(getattr(p,n)) for n in ('vx','vy','vz')) and
                math.sqrt(p.vx*p.vx+p.vy*p.vy+p.vz*p.vz)<.1):return False
        return (all(not self.get_publishers_info_by_topic('/px4_7/fmu/in/'+name)
                    for name in ('vehicle_command','trajectory_setpoint','offboard_control_mode')) and
                not self.get_publishers_info_by_topic('/planning/source/map') and
                not self.get_publishers_info_by_topic('/uav/map/snapshot'))

    def tick(self):
        endpoints={name:self.get_publishers_info_by_topic(topic) for name,topic in self.topics.items()
                   if name not in ('vehicle','land')}
        identities=tuple(sorted((name,tuple(bytes(e.endpoint_gid) for e in entries)) for name,entries in endpoints.items()))
        now=time.monotonic();ros=self.get_clock().now().nanoseconds/1e9
        self.selector.tick(now,ros,{name:len(entries) for name,entries in endpoints.items()},self.ground_safe(),identities)
        s=self.selector
        message=LocalizationSelection(coordinator_instance=self.instance,request_id=UUID(uuid=list(s.request_id)),
            active_profile=s.active_profile,requested_profile=s.requested_profile,phase=s.phase,reason=s.reason,
            generation=s.generation,ready=s.ready,navigation_allowed=s.navigation_allowed,
            requires_map_reinitialization=s.generation>0,height_yaw_source='VIO',mock=self.manifest.get('synthetic_vio',False))
        message.header.stamp=self.get_clock().now().to_msg()
        self.selection_pub.publish(message)
        if now-self.last_record>=.1:
            self.history.append(dict(mono=now,ros=ros,phase=s.phase,reason=s.reason,active=s.active_profile,
                requested=s.requested_profile,generation=s.generation,ready=s.ready,ground_safe=self.ground_safe()))
            self.last_record=now

    def select(self, request, response):
        if request.coordinator_instance!=self.instance or any(request.mission_uuid.uuid):
            response.reason='STALE_IDENTITY_OR_NON_NIL_ROOT';return response
        self.tick()
        response.accepted,response.reason=self.selector.request(request.profile,request.request_id.uuid,
            time.monotonic(),self.ground_safe())
        response.phase=self.selector.phase
        return response

    def close(self):
        self.selector.backend.close()
        (self.evidence/'localization-selection.json').write_text(json.dumps(dict(
            active_profile=self.selector.active_profile,phase=self.selector.phase,reason=self.selector.reason,
            fault=self.selector.fault,generation=self.selector.generation,history=self.history,
            events=self.selector.events,scope='ground-only; VIO height/yaw retained; no flight acceptance'),indent=2)+'\n')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session',required=True)
    parser.add_argument('--enable-sitl-parameters',action='store_true')
    parser.add_argument('--duration',type=float,default=120.)
    args=parser.parse_args()
    if not args.enable_sitl_parameters or not 5<=args.duration<=3600:
        parser.error('Requires --enable-sitl-parameters and duration 5..3600 s')
    session=Path(args.session).resolve()
    if not session.is_relative_to(ROOT/'.cache/simulation'):parser.error('Requires an owned session directory')
    manifest=json.loads((session/'manifest.json').read_text())
    if manifest.get('domain')!=78:parser.error('Requires domain 78')
    os.environ.update(ROS_DOMAIN_ID='78',ROS_LOCALHOST_ONLY='1',GZ_PARTITION=manifest['partition'])
    stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    with (session/'localization-selection.lock').open('a') as handle:
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'],signal_handler_options=SignalHandlerOptions.NO)
        node=SelectionSession(session)
        until=time.monotonic()+args.duration
        try:
            while not stopping and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.05)
        finally:
            node.close();node.destroy_node();rclpy.try_shutdown()


if __name__=='__main__':main()
