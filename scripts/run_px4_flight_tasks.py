#!/usr/bin/env python3
"""Exercise real ExecuteMission against owned x500 SITL, with independent truth."""
import json
import math
import os
from pathlib import Path
import threading
import subprocess
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from uav_nav_interfaces.action import ExecuteMission
from uav_nav_interfaces.srv import PauseMission,ResumeMission
from unique_identifier_msgs.msg import UUID
import uuid
from uav_mission.px4_flight import FlightServer
from uav_mission.flight_geometry import distance
from sim_validation import write_json,ROOT,file_hash


def main():
    out=Path(os.environ['UAV_FLIGHT_EVIDENCE'])
    scenario=os.environ.get('UAV_FLIGHT_SCENARIO','full')
    implementation_hashes={str(path.relative_to(ROOT)):file_hash(path) for path in
      [ROOT/'src/uav_mission/uav_mission/px4_flight.py',ROOT/'src/uav_mission/uav_mission/flight_geometry.py',
       ROOT/'src/px4_comm_bridge/px4_comm_bridge/converters.py',ROOT/'scripts/run_px4_flight_tasks.py',ROOT/'scripts/run_px4_sitl_smoke.py']}
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=FlightServer()
    truth=[]
    frames=set()
    truth_process=subprocess.Popen(['gz','topic','-e','-t','/world/default/pose/info','--json-output'],
                                   stdout=subprocess.PIPE,stderr=(out/'truth-reader.log').open('w'),text=True)
    def read_truth():
        for line in truth_process.stdout:
            try:
                msg=json.loads(line)
                for pose in msg.get('pose',[]):
                    frames.add(pose.get('name',''))
                    if pose.get('name')=='x500_7':
                        p=pose['position']
                        truth.append(dict(mono=time.monotonic(),position=[p.get('x',0),p.get('y',0),p.get('z',0)],phase=node.phase))
            except (ValueError,KeyError):
                pass
    truth_thread=threading.Thread(target=read_truth,daemon=True);truth_thread.start()
    executor=SingleThreadedExecutor();executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    client=ActionClient(node,ExecuteMission,'/uav/px4/execute_mission')
    result=dict(passed=False,scenario=scenario,profile='known_region_control',scope='real x500 takeoff/navigation/hover/return/native landing',mock=False)
    try:
        until=time.monotonic()+20
        while time.monotonic()<until:
            with node.lock:
                ready=node.healthy(ground=True) and node.samples.get('vehicle_status').pre_flight_checks_pass
            if ready and truth and time.monotonic()-truth[-1]['mono']<.5:break
            time.sleep(.05)
        if not ready or not truth:
            result['truth_frames']=sorted(frames)
            result['preflight_healthy']=ready
            result['preflight_fields']={n:{f:getattr(m,f) for f in fields} for n,fields in [('vehicle_status',['arming_state','nav_state','system_id','pre_flight_checks_pass','failsafe']),('battery_status',['connected','remaining','warning']),('vehicle_local_position',['xy_valid','z_valid','v_xy_valid','v_z_valid','eph','epv'])] if (m:=node.samples.get(n)) is not None}
            raise RuntimeError('Preflight/truth unavailable: '+str(list(node.samples)))
        home=node.position();home_truth=truth[-1]['position']
        alignment=node.alignment
        a=(home[0]+3,home[1]+2,home[2]+2)
        b=(home[0]-2,home[1]+2,home[2]+2)
        recipe=Path(os.environ.get('UAV_FLIGHT_MISSION_FILE') or ROOT/'simulation/missions/W0_flight_sequence.json')
        steps=json.loads(recipe.read_text())['steps']
        for step in steps:
            if step['type']=='NAVIGATE' and 'offset_enu' in step:
                offset=step.pop('offset_enu')
                step['target_map']=alignment.to_map(tuple(home[i]+offset[i] for i in range(3)))
        result['steps']=steps
        result['recipe_sha256']=file_hash(recipe)
        request=ExecuteMission.Goal(mission_type='FLIGHT_SEQUENCE',backend='PX4_KNOWN_REGION',timeout_s=210.,
                                   parameters_json=json.dumps(dict(authorization=node.nonce,steps=steps)))
        def wait(f,timeout):
            until=time.monotonic()+timeout
            while not f.done() and time.monotonic()<until:time.sleep(.02)
            if not f.done():raise TimeoutError('Action deadline exceeded')
            return f.result()
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Action discovery failed')
        handle=wait(client.send_goal_async(request),5)
        if not handle.accepted:raise RuntimeError('Mission rejected')
        terminal_future=handle.get_result_async()
        injected=False
        landing_cancel_rejected=False
        until=time.monotonic()+220
        while not terminal_future.done() and time.monotonic()<until:
            phase=node.phase
            if not injected and phase=='NAVIGATE' and time.monotonic()-node.phase_started>min(5.,node.segment.duration/2):
                if scenario=='pause-resume':
                    before_child=list(node.child.uuid)
                    pause=node.create_client(PauseMission,'/uav/px4/pause')
                    resume=node.create_client(ResumeMission,'/uav/px4/resume')
                    def request(kind):
                        return kind.Request(mission_uuid=handle.goal_id,coordinator_instance=node.instance,
                                            request_id=UUID(uuid=list(uuid.uuid4().bytes)))
                    decision=wait(pause.call_async(request(PauseMission)),5)
                    if not decision.accepted:raise RuntimeError('Pause rejected')
                    deadline=time.monotonic()+10
                    while node.phase!='PAUSED' and time.monotonic()<deadline:time.sleep(.02)
                    if node.phase!='PAUSED':raise RuntimeError('Pause did not stop')
                    pause_position=truth[-1]['position'];pause_start=time.monotonic()
                    time.sleep(3)
                    pause_drift=max(distance(t['position'],pause_position) for t in truth if t['mono']>=pause_start)
                    decision=wait(resume.call_async(request(ResumeMission)),5)
                    if not decision.accepted or before_child==list(node.child.uuid):raise RuntimeError('Resume identity failed')
                    result['pause_resume']=dict(passed=True,pause_drift_m=pause_drift,new_child=True)
                elif scenario=='cancel':
                    decision=wait(handle.cancel_goal_async(),5)
                    if not decision.goals_canceling:raise RuntimeError('Cancel rejected')
                elif scenario=='clock-fault':
                    for value,delay in (('true',1.1),('false',0)):
                        subprocess.run(['gz','service','-s','/world/default/control','--reqtype','gz.msgs.WorldControl',
                                        '--reptype','gz.msgs.Boolean','--timeout','2000','--req','pause: '+value],
                                       check=True,stdout=subprocess.DEVNULL,timeout=5)
                        time.sleep(delay)
                injected=True
            if phase in ('LAND_REQUEST','LANDING') and not landing_cancel_rejected:
                response=wait(handle.cancel_goal_async(),5)
                landing_cancel_rejected=not bool(response.goals_canceling)
                if not landing_cancel_rejected:raise RuntimeError('Native landing cancellation unexpectedly accepted')
            time.sleep(.02)
        terminal=wait(terminal_future,1)
        result['outputs_at_action_result']=node.output_count
        result['landing_cancel_rejected']=landing_cancel_rejected
        if scenario=='cancel':
            hold_start=time.monotonic();hold_position=truth[-1]['position']
            until=time.monotonic()+45
            while time.monotonic()<until:
                if node.samples['vehicle_status'].arming_state==1 and node.samples['vehicle_land_detected'].landed:break
                time.sleep(.05)
            hold_points=[t for t in truth if hold_start<=t['mono'] and t['phase']=='COMPLETE']
            result['cancel_hold']=dict(duration_s=(node.final_hold_until-hold_start),samples=len(hold_points),
                                      max_drift_m=max((distance(t['position'],hold_position) for t in hold_points),default=999.),
                                      final_landed_disarmed=node.samples['vehicle_status'].arming_state==1 and node.samples['vehicle_land_detected'].landed)

        result['action_status']=terminal.status
        result['result']=dict(code=terminal.result.result_code,reason=terminal.result.reason,
                              cleanup_confirmed=terminal.result.cleanup_confirmed,mock=terminal.result.mock)
        result['mission_uuid']=str(uuid.UUID(bytes=bytes(handle.goal_id.uuid)))
        result['home_enu']=home;result['home_truth']=home_truth
        result['phases']=[e['phase'] for e in node.events]
        result['max_truth_displacement_m']=max(distance(t['position'],home_truth) for t in truth)
        hovers=[]
        for e,next_e in zip(node.events,node.events[1:]):
            if e['phase']=='HOVER':
                points=[t for t in truth if e['mono']<=t['mono']<=next_e['mono']]
                if points:
                    start=points[0]['position']
                    hovers.append(dict(duration_s=next_e['mono']-e['mono'],samples=len(points),
                                       max_drift_m=max(distance(t['position'],start) for t in points)))
        result['hovers']=hovers
        waypoint_truth=[]
        for event,next_event in zip(node.events,node.events[1:]):
            if event['phase'] in ('TAKEOFF','NAVIGATE','RETURN') and event.get('target_enu') and next_event['phase'] not in ('PAUSING','CANCEL_BRAKE','FAULT'):
                points=[t for t in truth if event['mono']<=t['mono']<=next_event['mono']]
                if points:
                    expected=[home_truth[i]+event['target_enu'][i]-home[i] for i in range(3)]
                    waypoint_truth.append(dict(phase=event['phase'],expected_world=expected,
                                               actual_world=points[-1]['position'],error_m=distance(expected,points[-1]['position'])))
        result['waypoint_truth']=waypoint_truth
        result['source_sha256']=implementation_hashes
        result['final_truth']=truth[-1]['position']
        result['final_land']=node.samples['vehicle_land_detected'].landed
        result['final_arming']=node.samples['vehicle_status'].arming_state
        result['command_acks']=[dict(command=node.samples['vehicle_command_ack'].command,
                                     result=node.samples['vehicle_command_ack'].result)] if 'vehicle_command_ack' in node.samples else []
        expected_hover=[step['duration_s'] for step in steps if step['type']=='HOVER']
        expected_motion=sum(step['type'] in ('TAKEOFF','NAVIGATE','RETURN') for step in steps)
        result['passed']=(terminal.status==4 and terminal.result.cleanup_confirmed and not terminal.result.mock
                          and result['final_land'] and result['final_arming']==1 and landing_cancel_rejected
                          and len(hovers)==len(expected_hover) and len(waypoint_truth)>=expected_motion
                          and all(h['duration_s']>=duration and h['max_drift_m']<=.15
                                  for h,duration in zip(hovers,expected_hover))
                          and all(w['error_m']<=.3 for w in waypoint_truth))
        if scenario=='pause-resume':result['passed']=result['passed'] and result.get('pause_resume',{}).get('passed',False)
        if scenario=='cancel':result['passed']=(terminal.status==5 and terminal.result.cleanup_confirmed and result['cancel_hold']['final_landed_disarmed'] and result['cancel_hold']['max_drift_m']<=.15)
        result['outputs_at_end']=node.output_count
        if truth:result['final_truth']=truth[-1]['position']
        if 'vehicle_status' in node.samples:result['final_arming']=node.samples['vehicle_status'].arming_state
        if 'vehicle_land_detected' in node.samples:result['final_land']=node.samples['vehicle_land_detected'].landed
        if scenario=='clock-fault':result['passed']=result['passed'] and result.get('outputs_at_action_result')==node.output_count
        if scenario=='clock-fault':result['clock_fault_latched']=node.clock_fault_latched
        if scenario=='clock-fault':result['passed']=(terminal.status==6 and terminal.result.reason in ('CLOCK_FAULT','STALE_OR_INVALID_AIRCRAFT_STATE') and node.clock_fault_latched)
    except Exception as exc:
        result['error']=str(exc)
    finally:
        if (not result['passed'] or scenario=='clock-fault') and node.samples.get('vehicle_status') and node.samples['vehicle_status'].arming_state==2:
            until=time.monotonic()+40
            while time.monotonic()<until:
                if node.samples['vehicle_status'].arming_state==1 and node.samples.get('vehicle_land_detected') and node.samples['vehicle_land_detected'].landed:break
                time.sleep(.05)
            result['fallback_landed_disarmed']=(node.samples['vehicle_status'].arming_state==1 and node.samples.get('vehicle_land_detected') and node.samples['vehicle_land_detected'].landed)
        result['outputs_at_end']=node.output_count
        if truth:result['final_truth']=truth[-1]['position']
        if 'vehicle_status' in node.samples:result['final_arming']=node.samples['vehicle_status'].arming_state
        if 'vehicle_land_detected' in node.samples:result['final_land']=node.samples['vehicle_land_detected'].landed
        if scenario=='clock-fault':result['passed']=result['passed'] and result.get('outputs_at_action_result')==node.output_count
        if scenario=='clock-fault':result['clock_fault_latched']=node.clock_fault_latched
        if scenario=='clock-fault':result['passed']=result['passed'] and result.get('fallback_landed_disarmed',False)
        write_json(out/'flight-observation.json',result)
        write_json(out/'flight-events.json',node.events)
        write_json(out/'flight-diagnostics.json',node.diagnostics)
        write_json(out/'flight-status-history.json',node.status_history)
        write_json(out/'flight-commands.json',node.commands)
        write_json(out/'flight-trace.json',node.trace)
        write_json(out/'flight-truth.json',truth)
        truth_process.terminate()
        truth_process.wait(timeout=5);truth_thread.join(timeout=2)
        client.destroy();executor.shutdown();node.destroy_node();rclpy.shutdown();thread.join(timeout=3)
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
