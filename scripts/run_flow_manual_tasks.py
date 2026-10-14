#!/usr/bin/env python3
"""Owned no-EV/no-GNSS native Position test with a simulated RC receiver.

No OffboardControlMode or TrajectorySetpoint writers. Manual input stands in for
the independent physical receiver, not a Jetson flight controller.
"""
import json
import math
import os
from pathlib import Path
import subprocess
import threading
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import (VehicleStatus, VehicleLocalPosition, VehicleLandDetected, BatteryStatus,
    VehicleCommand, ManualControlSetpoint, EstimatorStatusFlags, EstimatorSelectorStatus,
    EstimatorAidSource1d, EstimatorAidSource2d, DistanceSensor, SensorOpticalFlow)
from uav_mission.flow_gate import FlowGate
from uav_mission.flight_profiles import load_region
from sim_validation import ROOT, write_json


def main():
    config=load_region(ROOT,os.environ)
    if os.environ.get('UAV_LOCALIZATION_PROFILE')!='FLOW_LOCAL':
        raise RuntimeError('Standalone owned flow profile required')
    out=Path(os.environ['UAV_FLIGHT_EVIDENCE'])
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=rclpy.create_node('flow_native_manual_audit')
    gate=FlowGate();samples={};received={};topics={}
    entries=[('vehicle',VehicleStatus,'out/vehicle_status'),('land',VehicleLandDetected,'out/vehicle_land_detected'),
        ('battery',BatteryStatus,'out/battery_status'),
        ('local',VehicleLocalPosition,'out/vehicle_local_position'),('flags',EstimatorStatusFlags,'out/estimator_status_flags'),
        ('selector',EstimatorSelectorStatus,'out/estimator_selector_status'),
        ('optical_flow',EstimatorAidSource2d,'out/estimator_aid_src_optical_flow'),
        ('rng_hgt',EstimatorAidSource1d,'out/estimator_aid_src_rng_hgt'),
        ('range_sensor',DistanceSensor,'in/distance_sensor'),('flow_sensor',SensorOpticalFlow,'in/sensor_optical_flow'),
        ('manual',ManualControlSetpoint,'out/manual_control_setpoint')]
    for name,kind,path in entries:
        version=getattr(kind,'MESSAGE_VERSION',0)
        topic='/px4_7/fmu/'+path+(f'_v{version}' if version else '')
        topics[name]=topic
        def receive(m,key=name):
            samples[key]=m;received[key]=time.monotonic();gate.receive(key,m,received[key])
        node.create_subscription(kind,topic,receive,qos_profile_sensor_data)
    commands=node.create_publisher(VehicleCommand,'/px4_7/fmu/in/vehicle_command',10)
    rc=node.create_publisher(ManualControlSetpoint,'/px4_7/fmu/in/manual_control_input',10)
    history=[];command_history=[];truth=[];phase='GROUND';phase_start=time.monotonic()
    truth_log=(out/'truth-reader.log').open('w')
    reader=subprocess.Popen(['gz','topic','-e','-t','/world/default/pose/info','--json-output'],
        stdout=subprocess.PIPE,stderr=truth_log,text=True)
    def read_truth():
        for line in reader.stdout:
            try:
                for pose in json.loads(line).get('pose',[]):
                    if pose.get('name')==os.environ['UAV_FLIGHT_MODEL']:
                        p=pose['position'];truth.append(dict(mono=time.monotonic(),phase=phase,
                            position=[p.get(n,0.) for n in ('x','y','z')]))
            except (ValueError,KeyError):pass
    thread=threading.Thread(target=read_truth,daemon=True);thread.start()
    def command(number,p1=0.,p2=0.):
        commands.publish(VehicleCommand(timestamp=node.get_clock().now().nanoseconds//1000,
            command=number,param1=float(p1),param2=float(p2),target_system=8,target_component=1,
            source_system=246,source_component=191,from_external=True))
        command_history.append(dict(mono=time.monotonic(),phase=phase,command=number,param1=p1,param2=p2))
    def change(value):
        nonlocal phase,phase_start
        phase,phase_start=value,time.monotonic()
        print('NATIVE_FLOW_PHASE '+phase,flush=True)
    def fresh(key,age):
        m=samples.get(key);ros=node.get_clock().now().nanoseconds/1e9
        return m is not None and time.monotonic()-received[key]<=age and -.05<=ros-m.timestamp/1e6<=age
    result=dict(passed=False,scope='native PX4 Position with simulated RC, no companion Offboard heartbeat; reference sensors only',mock=False)
    last_rc=last_record=0.;mode_sent=False;arm_sent=False;last_land=0.
    try:
        until=time.monotonic()+85.
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.005)
            now=time.monotonic();ros=node.get_clock().now().nanoseconds/1e9
            vehicle=samples.get('vehicle');land=samples.get('land');p=samples.get('local')
            # A physical receiver keeps supplying sticks independently of the
            # companion. This ROS writer is only the simulation replacement.
            if ros>0 and now-last_rc>=.02:
                throttle=.35 if phase=='CLIMB' else 0. if phase=='HOVER' else -1.
                rc.publish(ManualControlSetpoint(timestamp=int(ros*1e6),timestamp_sample=int(ros*1e6),
                    valid=True,data_source=ManualControlSetpoint.SOURCE_MAVLINK_0,
                    roll=0.,pitch=0.,yaw=0.,throttle=throttle))
                last_rc=now
            writers={n:len(node.get_publishers_info_by_topic(topics[n])) for n in gate.required}
            healthy=gate.ready(now,ros,writers,native_landing=phase=='LANDING' and
                vehicle is not None and vehicle.nav_state==18)
            absent={n:len(node.get_publishers_info_by_topic(n)) for n in
                ('/px4_7/fmu/in/offboard_control_mode','/px4_7/fmu/in/trajectory_setpoint',
                 '/px4_7/fmu/in/vehicle_visual_odometry','/uav/vio/status')}
            if any(absent.values()):raise RuntimeError('Unexpected companion writer')
            if (len(node.get_publishers_info_by_topic('/px4_7/fmu/in/vehicle_command'))!=1 or
                    len(node.get_publishers_info_by_topic('/px4_7/fmu/in/manual_control_input'))!=1):
                raise RuntimeError('Non-unique native control owner')
            if now-last_record>=.1:
                f=samples.get('flags')
                history.append(dict(mono=now,phase=phase,healthy=healthy,reason=gate.reason,
                    nav_state=getattr(vehicle,'nav_state',None),arming_state=getattr(vehicle,'arming_state',None),
                    absent=absent,flow=getattr(f,'cs_opt_flow',False),range=getattr(f,'cs_rng_hgt',False),
                    no_ev_gnss=f is not None and not any(getattr(f,n) for n in
                        ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw','cs_ev_vel','cs_gnss_pos','cs_gnss_vel'))))
                last_record=now
            if not (fresh('vehicle',.75) and fresh('land',1.2)):
                continue
            b=samples.get('battery')
            aircraft_healthy=(vehicle.system_id==8 and vehicle.component_id==1 and not vehicle.failsafe and
                not vehicle.failure_detector_status and fresh('battery',1.5) and b.connected and
                b.warning==0 and b.remaining>.2)
            if phase=='GROUND' and healthy and land.landed and vehicle.arming_state==1:
                if aircraft_healthy and not mode_sent:command(176,1,3);mode_sent=True;change('POSITION_REQUEST')
            elif phase=='POSITION_REQUEST' and healthy and aircraft_healthy and vehicle.nav_state==2:
                manual=samples.get('manual')
                if (vehicle.pre_flight_checks_pass and fresh('manual',.5) and manual.valid and
                        manual.throttle<-.9 and not arm_sent):
                    command(400,1);arm_sent=True;change('ARM_REQUEST')
            elif phase=='ARM_REQUEST' and vehicle.arming_state==2:
                if not healthy or not aircraft_healthy or not truth or now-truth[-1]['mono']>.5:
                    raise RuntimeError('Health/truth lost after arm')
                change('CLIMB')
            elif phase in ('CLIMB','HOVER'):
                from uav_mission.flight_geometry import in_region
                inside=in_region((p.y,p.x,-p.z),config['bounds_min'],config['bounds_max'],
                    config['body_radius_m']+config['tracking_margin_m']+config['braking_margin_m'])
                if not healthy or not aircraft_healthy or not inside or vehicle.nav_state!=2 or vehicle.arming_state!=2:
                    raise RuntimeError('Native Position health/control lost: '+gate.reason)
                if phase=='CLIMB' and samples['range_sensor'].current_distance>=1.2:
                    change('HOVER')
                elif phase=='CLIMB' and now-phase_start>20.:raise RuntimeError('Native takeoff timed out')
                elif phase=='HOVER' and now-phase_start>=10.:
                    command(21);last_land=now;change('LANDING')
            elif phase=='LANDING':
                if land.landed and vehicle.arming_state==1:change('COMPLETE');break
                if now-last_land>2. and vehicle.nav_state!=18:command(21);last_land=now
        hover=[t for t in truth if t['phase']=='HOVER']
        drift=max((math.dist(t['position'],hover[0]['position']) for t in hover),default=math.inf)
        air=[s for s in history if s['phase'] in ('CLIMB','HOVER')]
        independent=(len(air)>=50 and all(s['nav_state']==2 and s['flow'] and s['range'] and
            s['no_ev_gnss'] and not any(s['absent'].values()) for s in air))
        # Native Position continues the full hover without any mode/trajectory/
        # command stream from a companion; only simulated RC input remains.
        hover_commands=[c for c in command_history if c['phase']=='HOVER' and c['command']!=21]
        result.update(phase=phase,commands=command_history,hover_drift_m=drift if math.isfinite(drift) else None,
            hover_samples=len(hover),final_arming=getattr(vehicle,'arming_state',None),
            final_land=getattr(land,'landed',None),standalone_flow=dict(passed=independent,
                no_vio_subscriptions=True,airborne_samples=len(air),no_companion_hover_commands=not hover_commands))
        result['passed']=(phase=='COMPLETE' and independent and not hover_commands and
            len(hover)>=100 and drift<=.15 and vehicle.arming_state==1 and land.landed)
    except Exception as error:
        result['error']=str(error)
    finally:
        # No retry/auto-arm after a rejected diagnostic. An already armed owned
        # simulator is handed to native LAND; never force-disarm in flight.
        if samples.get('vehicle') is not None and samples['vehicle'].arming_state==2:
            command(21)
            until=time.monotonic()+30.
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.02)
                if samples['vehicle'].arming_state==1 and samples.get('land') and samples['land'].landed:break
        result['fallback_landed_disarmed']=(samples.get('vehicle') is not None and
            samples['vehicle'].arming_state==1 and samples.get('land') is not None and samples['land'].landed)
        reader.terminate()
        try:reader.wait(timeout=3)
        except subprocess.TimeoutExpired:reader.kill();reader.wait()
        thread.join(timeout=1);truth_log.close()
        write_json(out/'standalone-flow-samples.json',history)
        write_json(out/'truth-trace.json',truth)
        write_json(out/'flight-observation.json',result)
        node.destroy_node();rclpy.try_shutdown()
        print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
