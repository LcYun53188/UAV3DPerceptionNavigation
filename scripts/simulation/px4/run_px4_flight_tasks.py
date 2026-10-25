#!/usr/bin/env python3
"""Exercise real ExecuteMission against owned x500 SITL, with independent truth."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[2])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import json
import math
import os
from pathlib import Path
import threading
import subprocess
import signal
import re
from copy import deepcopy
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from tf2_msgs.msg import TFMessage
from uav_nav_interfaces.action import ExecuteMission
from uav_nav_interfaces.srv import PauseMission,ResumeMission,SetMissionIntent
from unique_identifier_msgs.msg import UUID
import uuid
from uav_mission.px4_flight import FlightServer
from uav_mission.flight_geometry import distance
from sim_validation import write_json,ROOT,file_hash


def observation_passed(task_passed, cleanup_verified, profile):
    # A safely rejected depth-observation diagnostic can pass its negative
    # test. Static avoidance requires the actual navigation task to complete.
    return bool(task_passed or (cleanup_verified and profile!='experimental_sdk16_static_avoidance'))


def completed_waypoint_truth(events, truth, home, home_truth):
    """Audit only motion followed by an accepted next recipe step, never a stop."""
    reached=[]
    for event,next_event in zip(events,events[1:]):
        if (event['phase'] in ('TAKEOFF','NAVIGATE','RETURN') and event.get('target_enu')
                and next_event['phase'] in ('NAVIGATE','HOVER','RETURN','LAND_REQUEST','AWAIT_STEP')):
            points=[t for t in truth if event['mono']<=t['mono']<=next_event['mono']]
            if points:
                expected=[home_truth[i]+event['target_enu'][i]-home[i] for i in range(3)]
                reached.append(dict(phase=event['phase'],expected_world=expected,
                                    actual_world=points[-1]['position'],error_m=distance(expected,points[-1]['position'])))
    return reached


def main():
    out=Path(os.environ['UAV_FLIGHT_EVIDENCE'])
    from vio_observer_gc import ObservationGcGuard
    from vio_gc_pause_audit import GcPauseAudit
    flight_gc=ObservationGcGuard(os.environ.get('UAV_EXPERIMENTAL_VIO_CHAIN')=='1')
    gc_pauses=GcPauseAudit()
    scenario=os.environ.get('UAV_FLIGHT_SCENARIO','full')
    use_bt=os.environ.get('UAV_FLIGHT_BT')=='1'
    bt=None
    implementation_hashes={str(path.relative_to(ROOT)):file_hash(path) for path in
      [ROOT/'src/uav_nav_interfaces/msg/VioStatus.msg',ROOT/'src/uav_mission/uav_mission/vio_gate.py',
       ROOT/'src/px4_comm_bridge/px4_comm_bridge/vio_input.py',ROOT/'src/px4_comm_bridge/px4_comm_bridge/vio_input_node.py',
       ROOT/'src/uav_nav_interfaces/msg/LocalizedOdometry.msg',ROOT/'src/uav_nav_interfaces/msg/TaskStatus.msg',
       ROOT/'src/uav_nav_interfaces/srv/ResumeOriginalMission.srv',ROOT/'src/uav_nav_interfaces/srv/RequestReturn.srv',
       ROOT/'src/uav_mission/uav_mission/mission_checkpoint.py',ROOT/'src/uav_mission/uav_mission/home_reference.py',ROOT/'src/uav_mission/uav_mission/pilot_intervention.py',ROOT/'src/uav_mission/uav_mission/px4_flight.py',ROOT/'src/uav_mission/uav_mission/flight_geometry.py',ROOT/'src/uav_mission/uav_mission/flight_profiles.py',
       ROOT/'src/uav_mission/uav_mission/observation_survey.py',ROOT/'src/uav_mission/uav_mission/manual_input_evidence.py',ROOT/'src/uav_mission/uav_mission/ego_execution.py',ROOT/'src/uav_nav_sim/uav_nav_sim/core.py',ROOT/'src/px4_comm_bridge/px4_comm_bridge/converters.py',ROOT/'scripts/simulation/px4/run_px4_flight_tasks.py',ROOT/'scripts/simulation/px4/no_gnss_flight_cases.py',ROOT/'scripts/simulation/px4/run_px4_sitl_smoke.py']}
    if use_bt:
        for name in ('src/uav_bt/src/mission_runner.cpp','src/uav_bt/trees/px4_flight.xml',
                     'src/uav_nav_interfaces/srv/AdvanceFlightStep.srv',
                     'scripts/simulation/px4/bt_flight_client.py','.deps/mission-install/uav_bt/lib/uav_bt/mission_runner'):
            implementation_hashes[name]=file_hash(ROOT/name)
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node=FlightServer()
    prewarmed=None
    if use_bt and os.environ.get('UAV_FLIGHT_REGION_PROFILE') in ('vio_avoidance','obstacle_course'):
        from bt_flight_client import prewarm_runner
        prewarmed=prewarm_runner(node,out,ROOT)
    map_diagnostics=[]
    from std_msgs.msg import String
    def map_diagnostic_receive(message):
        map_diagnostics.append(dict(mono=time.monotonic(), ros=node.get_clock().now().nanoseconds/1e9, data=json.loads(message.data)))
    node.create_subscription(String, '/uav/map/diagnostics', map_diagnostic_receive, 10)
    localized_samples=[]
    from uav_nav_interfaces.msg import LocalizedOdometry
    def localized_receive(message):
        localized_samples.append(dict(session=message.localization_session,
            counters=list(map(int,message.reset_counters)),frame=message.header.frame_id,
            stamp_matches=message.header==message.odometry.header,
            stamp_ns=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec,
            received_ros=node.get_clock().now().nanoseconds/1e9,
            child=message.odometry.child_frame_id))
    node.create_subscription(LocalizedOdometry,'/uav/px4/localized_odometry',localized_receive,10)
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
                    if pose.get('name')==os.environ.get('UAV_FLIGHT_MODEL','x500_7'):
                        p=pose['position']
                        stamp=msg.get('header',{}).get('stamp',{})
                        source_sim_s=float(stamp.get('sec',0))+float(stamp.get('nsec',0))/1e9
                        truth.append(dict(mono=time.monotonic(),source_sim_s=source_sim_s,
                            position=[p.get('x',0),p.get('y',0),p.get('z',0)],phase=node.phase))
            except (ValueError,KeyError):
                pass
    truth_thread=threading.Thread(target=read_truth,daemon=True);truth_thread.start()
    from no_gnss_flight_cases import NoGnssCase
    flight_case=NoGnssCase(scenario,node,out,truth)
    executor=SingleThreadedExecutor();executor.add_node(node)
    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass  # SIGINT may shut down the context before owned cleanup.
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    client=ActionClient(node,ExecuteMission,'/uav/px4/execute_mission')
    result=dict(passed=False,scenario=scenario,vio_required=os.environ.get('UAV_REQUIRE_VIO')=='1',profile=node.config['profile'],scope=('real same-aircraft takeoff/depth observation/EGO navigation/hover/return/native landing' if os.environ.get('UAV_EXPECT_OBSERVATION')=='1' else 'real x500 takeoff/navigation/hover/return/native landing'),mock=False)
    flow_samples=[]
    if node.localization_profile=='FLOW_LOCAL':
        result['scope']='Native PX4 no-GNSS/no-EV rendered MTF reference local takeoff/hover/native landing'
        def observe_flow():
            gate=node.vio_gate
            flags=gate.samples.get('flags')
            flow_samples.append(dict(mono=time.monotonic(),phase=node.phase,reason=gate.reason,
                visual_writers=len(node.get_publishers_info_by_topic('/px4_7/fmu/in/vehicle_visual_odometry')),
                vio_writers=len(node.get_publishers_info_by_topic('/uav/vio/status')),
                flags={n:getattr(flags,n) for n in ('cs_opt_flow','cs_rng_hgt','cs_ev_pos','cs_ev_vel',
                    'cs_ev_hgt','cs_ev_yaw','cs_gnss_pos','cs_gnss_vel')} if flags is not None else {}))
        node.create_timer(.1,observe_flow)
    try:
        until=time.monotonic()+(340 if os.environ.get('UAV_EXPERIMENTAL_VIO_CHAIN')=='1' else 40 if os.environ.get('UAV_FLIGHT_REGION_PROFILE') in ('warehouse','depth_reference') else 20)
        while time.monotonic()<until:
            with node.lock:
                if node.preflight_fault_latched:
                    raise RuntimeError('VIO ground prestream failed: '+str(node.diagnostics[-1:]))
                if node.localization_profile=='FLOW_LOCAL' and node.healthy(ground=True):
                    node.prepare_vio_ground_mode()
                ready=node.healthy(ground=True) and node.samples.get('vehicle_status').pre_flight_checks_pass
            if ready and os.environ.get('UAV_NAVIGATION_BACKEND')=='EGO':
                try:
                    with node.lock:node.planning_ready()
                except ValueError:ready=False
            admission=os.environ.get('UAV_FLIGHT_ADMISSION')
            if admission:
                path=Path(admission)
                receipt=json.loads(path.read_text()) if path.is_file() else {}
                ready=ready and receipt.get('sensor_checks_passed') is True and receipt.get('calibration_id')==os.environ.get('UAV_VIO_CALIBRATION_ID')
            if admission and receipt.get('sensor_checks_passed') is True:
                with node.lock:node.prepare_vio_ground_mode()
            if ready and truth and time.monotonic()-truth[-1]['mono']<.5:break
            time.sleep(.05)
        if not ready or not truth:
            result['truth_frames']=sorted(frames)
            result['preflight_healthy']=ready
            result['failsafe_flags']=str(node.samples.get('failsafe_flags'))
            result['vio_gate_reason']=node.vio_gate.reason if node.vio_gate else None
            if node.vio_gate:
                result['vio_missing']=[n for n in node.vio_gate.required if n not in node.vio_gate.samples]
                result['vio_received_ages']={n:time.monotonic()-t for n,t in node.vio_gate.received.items()}
            result['preflight_fields']={n:{f:getattr(m,f) for f in fields} for n,fields in [('vehicle_status',['arming_state','nav_state','system_id','pre_flight_checks_pass','failsafe']),('battery_status',['connected','remaining','warning']),('vehicle_local_position',['xy_valid','z_valid','v_xy_valid','v_z_valid','eph','epv'])] if (m:=node.samples.get(n)) is not None}
            raise RuntimeError('Preflight/truth unavailable: '+str(list(node.samples)))
        home=node.position();home_truth=truth[-1]['position']
        alignment=node.alignment
        a=(home[0]+3,home[1]+2,home[2]+2)
        b=(home[0]-2,home[1]+2,home[2]+2)
        recipe=Path(os.environ.get('UAV_FLIGHT_MISSION_FILE') or ROOT/'simulation/missions/W0_flight_sequence.json')
        steps=json.loads(recipe.read_text())['steps']
        if scenario.startswith('local-land-'):
            steps=[dict(type='TAKEOFF',height_m=2.),dict(type='HOVER',duration_s=20.),dict(type='LAND')]
        for step in steps:
            if step['type']=='NAVIGATE' and 'offset_enu' in step:
                offset=step.pop('offset_enu')
                step['target_map']=alignment.to_map(tuple(home[i]+offset[i] for i in range(3)))
        result['steps']=steps
        navigation='DIRECT' if scenario.startswith('local-land-') else os.environ.get('UAV_NAVIGATION_BACKEND','DIRECT')
        result['navigation_backend']=navigation
        if scenario.startswith('local-land-'):result['scope']='No-GNSS known-region takeoff, actual QGC pilot mode, native slow local LAND; no map-return acceptance'
        result['recipe_sha256']=file_hash(recipe)
        root_deadline=min(node.config.get('mission_timeout_max_s',240.),max(node.config.get('mission_timeout_max_s',210.) if 'mission_timeout_max_s' in node.config else 210.,max((step.get('timeout_s',0.) for step in steps if step['type']=='OBSERVE'),default=0.)+45.))
        request=ExecuteMission.Goal(mission_type='FLIGHT_SEQUENCE',backend='PX4_KNOWN_REGION',timeout_s=root_deadline,
                                   parameters_json=json.dumps(dict(authorization=node.nonce,steps=steps,navigation_backend=navigation)))
        def wait(f,timeout):
            until=time.monotonic()+timeout
            while not f.done() and time.monotonic()<until:time.sleep(.02)
            if not f.done():raise TimeoutError('Action deadline exceeded')
            return f.result()
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Action discovery failed')
        if use_bt:
            from bt_flight_client import BtFlightClient
            bt=BtFlightClient(node,request,out,ROOT,prewarmed=prewarmed)
            handle=bt.wait_accepted()
        else:
            handle=wait(client.send_goal_async(request),5)
        if not handle.accepted:raise RuntimeError('Mission rejected')
        terminal_future=handle.get_result_async()
        operator=node.create_client(SetMissionIntent,'/uav/bt/set_intent') if use_bt else None
        def intent(command):
            if not operator.wait_for_service(timeout_sec=3):raise RuntimeError('BT intent service unavailable')
            response=wait(operator.call_async(SetMissionIntent.Request(mission_uuid=handle.goal_id,
                coordinator_instance=node.instance,request_id=UUID(uuid=list(uuid.uuid4().bytes)),command=command)),5)
            if not response.accepted:raise RuntimeError(command+' rejected: '+response.reason)
        def await_phase(allowed,timeout=12):
            deadline=time.monotonic()+timeout
            while node.phase not in allowed and time.monotonic()<deadline and not terminal_future.done():time.sleep(.02)
            if node.phase not in allowed:raise RuntimeError('Expected phase '+str(allowed)+', observed '+node.phase)
        injected=False
        landing_cancel_rejected=False
        until=time.monotonic()+root_deadline+10.
        while not terminal_future.done() and time.monotonic()<until:
            phase=node.phase
            inject_ready = (phase=='HOVER' and time.monotonic()-node.phase_started>1.
                if scenario in ('pilot-intervention','finish-mission','local-land-manual','local-land-position') else
                phase=='HOVER' and time.monotonic()-node.phase_started>5. if scenario=='return-hover-resume' else
                phase=='NAVIGATE' and time.monotonic()-node.phase_started>min(5.,(node.segment.duration if node.segment is not None else getattr(node.planned,'duration',10.))/2))
            if not injected and inject_ready:
                if scenario.startswith('local-land-'):
                    flight_case.local_land()
                elif scenario=='pilot-intervention':
                    flight_case.intervene(intent)
                elif scenario=='finish-mission':
                    root=bytes(handle.goal_id.uuid);original_home=node.home;original_deadline=node.deadline
                    flight_case.missing_home_refusal()
                    intent('END_MISSION')
                    await_phase(('RETURN_BRAKE','HOLD_HANDOFF','PLAN_REQUEST','RETURN'))
                    flight_case.evidence.update(passed=True,root_home_deadline_preserved=
                        bytes(node.active_goal.goal_id.uuid)==root and node.home==original_home and node.deadline==original_deadline)
                elif scenario in ('return-resume-mission','return-hover-resume'):
                    original_root=bytes(handle.goal_id.uuid);original_home=node.home;original_deadline=node.deadline
                    original_index=node.step_index;original_child=bytes(node.child.uuid)
                    intent('RETURN')
                    await_phase(('RETURN',))
                    return_start_position=truth[-1]['position']
                    if scenario=='return-hover-resume':
                        movement_until=time.monotonic()+6.
                        while (distance(truth[-1]['position'],return_start_position)<.5 and
                                node.phase=='RETURN' and time.monotonic()<movement_until):time.sleep(.02)
                        if distance(truth[-1]['position'],return_start_position)<.5:
                            raise RuntimeError('Hover rejoin scenario did not move at least 0.5 m during return')
                    else:time.sleep(.3)
                    return_displacement=distance(truth[-1]['position'],return_start_position)
                    intent('PAUSE')
                    await_phase(('PAUSED',))
                    pause_start=time.monotonic();pause_position=truth[-1]['position']
                    time.sleep(2.)
                    pause_points=[t for t in truth if t['mono']>=pause_start]
                    pause_drift=max((distance(t['position'],pause_position) for t in pause_points),default=999.)
                    checkpoint=node.return_checkpoint
                    if checkpoint is None:raise RuntimeError('Temporary return lost its checkpoint')
                    generation=node.generation
                    intent('RESUME_MISSION')
                    await_phase(('NAVIGATE','PLAN_REQUEST','HOVER'))
                    same_identity=(bytes(node.active_goal.goal_id.uuid)==original_root and
                        node.home==original_home and node.deadline==original_deadline)
                    result['return_resume_mission']=dict(original_step_index=original_index,
                        restored_step_index=node.step_index,plan_revision=node.plan_revision,
                        pause_samples=len(pause_points),pause_drift_m=pause_drift,
                        return_displacement_m=return_displacement,
                        hover_remaining_s=checkpoint.hover_remaining,hover_anchor=checkpoint.hover_target,
                        root_home_deadline_preserved=same_identity,
                        new_child=any(node.child.uuid) and bytes(node.child.uuid)!=original_child,
                        new_generation=node.generation>generation,
                        passed=same_identity and node.step_index==original_index and node.plan_revision==2 and
                            not node.return_override and any(node.child.uuid) and bytes(node.child.uuid)!=original_child and
                            node.generation>generation and len(pause_points)>=5 and pause_drift<=.15)
                elif scenario=='pause-resume':
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
                elif scenario == 'odometry-stale':
                    # Drop only the gateway reader; PX4 and the independent observer keep running.
                    node.destroy_subscription(node.input_subscriptions.pop('vehicle_odometry'))
                    result['localization_injection'] = dict(kind='gateway_odometry_subscription_removed',
                        mono=time.monotonic())
                elif scenario == 'odometry-reset':
                    # Receiver contract injection, not an actual EKF reset or FMU command.
                    with node.lock:
                        message = deepcopy(node.samples['vehicle_odometry'])
                        previous = message.reset_counter
                        message.timestamp += 1
                        message.reset_counter = (previous+1) % 256
                        result['localization_injection'] = dict(kind='gateway_reset_sample',
                            mono=time.monotonic(), previous=previous, current=message.reset_counter)
                        node.callback('vehicle_odometry')(message)
                elif scenario in ('runner-exit','runner-stall'):
                    bt.process.send_signal(signal.SIGKILL if scenario=='runner-exit' else signal.SIGSTOP)
                    result['runner_fault_injected_mono']=time.monotonic()
                injected=True
            if phase in ('LAND_REQUEST','LANDING') and not landing_cancel_rejected:
                response=wait(handle.cancel_goal_async(),5)
                landing_cancel_rejected=not bool(response.goals_canceling)
                if not landing_cancel_rejected:raise RuntimeError('Native landing cancellation unexpectedly accepted')
            time.sleep(.02)
        terminal=wait(terminal_future,1)
        result['outputs_at_action_result']=node.output_count
        result['landing_cancel_rejected']=landing_cancel_rejected
        if scenario in ('cancel','runner-exit','runner-stall'):
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
                    hovers.append(dict(step_index=e.get('step_index'),duration_s=next_e['mono']-e['mono'],samples=len(points),
                                       max_drift_m=max(distance(t['position'],start) for t in points)))
        result['hovers']=hovers
        waypoint_truth=completed_waypoint_truth(node.events,truth,home,home_truth)
        result['waypoint_truth']=waypoint_truth
        result['source_sha256']=implementation_hashes
        result['final_truth']=truth[-1]['position']
        result['final_land']=node.samples['vehicle_land_detected'].landed
        result['final_arming']=node.samples['vehicle_status'].arming_state
        result['return_home_references']=[row['return_home'] for row in node.diagnostics if 'return_home' in row]
        if getattr(getattr(node,'vio_gate',None),'primary_mtf',False):
            result['fusion_profile']='aligned_pose_mtf_primary_sim_v1'
            result['auxiliary_fusion_events']=list(node.vio_gate.auxiliary_history)
        result['command_acks']=[dict(command=node.samples['vehicle_command_ack'].command,
                                     result=node.samples['vehicle_command_ack'].result)] if 'vehicle_command_ack' in node.samples else []
        expected_hover=[step['duration_s'] for step in steps if step['type']=='HOVER']
        hover_audit=hovers
        if scenario in ('return-hover-resume','pilot-intervention'):
            hover_audit=[dict(duration_s=sum(h['duration_s'] for h in hovers if h['step_index']==i),
                max_drift_m=max((h['max_drift_m'] for h in hovers if h['step_index']==i),default=999.))
                for i,step in enumerate(steps) if step['type']=='HOVER']
        expected_motion=sum(step['type'] in ('TAKEOFF','NAVIGATE','RETURN') for step in steps)
        result['passed']=(terminal.status==4 and terminal.result.cleanup_confirmed and not terminal.result.mock
                          and result['final_land'] and result['final_arming']==1 and landing_cancel_rejected
                          and len(hover_audit)==len(expected_hover) and len(waypoint_truth)>=expected_motion
                          and all(h['duration_s']>=duration and h['max_drift_m']<=.15
                                  for h,duration in zip(hover_audit,expected_hover))
                          and all(w['error_m']<=.3 for w in waypoint_truth))
        if scenario=='pause-resume':result['passed']=result['passed'] and result.get('pause_resume',{}).get('passed',False)
        if scenario=='pilot-intervention':result['passed'] &= flight_case.evidence.get('passed',False)
        if scenario=='finish-mission':
            result['passed']=(terminal.status==4 and terminal.result.reason=='MISSION_ENDED_LANDED_AND_DISARMED'
                and terminal.result.cleanup_confirmed and not terminal.result.mock and result['final_land']
                and result['final_arming']==1 and landing_cancel_rejected
                and flight_case.evidence.get('root_home_deadline_preserved',False)
                and flight_case.evidence.get('missing_home_refused_without_control_mutation',False)
                and any(w['phase']=='RETURN' for w in waypoint_truth) and all(w['error_m']<=.3 for w in waypoint_truth))
        if scenario in ('return-resume-mission','return-hover-resume'):
            result['passed'] &= (result.get('return_resume_mission',{}).get('passed',False) and
                result['phases'].count('TAKEOFF')==1 and terminal.result.reason=='LANDED_AND_DISARMED')
        if scenario=='return-hover-resume':
            restored_at=next((e['mono'] for e in node.events if e.get('reason')=='ORIGINAL_MISSION_RESTORED'),None)
            index=result.get('return_resume_mission',{}).get('original_step_index',-1)
            join_events=[e for e in node.events if restored_at is not None and e['mono']>restored_at and
                e.get('step_index')==index and e['phase'] in ('NAVIGATE','HOVER')]
            result['hover_rejoin_verified']=(len(join_events)==2 and
                [e['phase'] for e in join_events]==['NAVIGATE','HOVER'] and
                result.get('return_resume_mission',{}).get('return_displacement_m',0.)>=.5)
            result['passed'] &= result['hover_rejoin_verified']
        if scenario=='cancel':result['passed']=(terminal.status==5 and terminal.result.cleanup_confirmed and result['cancel_hold']['final_landed_disarmed'] and result['cancel_hold']['max_drift_m']<=.15)
        if scenario in ('runner-exit','runner-stall'):
            result['passed']=(terminal.status==6 and terminal.result.reason=='BT_PROGRESS_TIMEOUT'
                              and terminal.result.cleanup_confirmed and result['cancel_hold']['final_landed_disarmed']
                              and result['cancel_hold']['max_drift_m']<=.15)
        if scenario in ('odometry-stale', 'odometry-reset'):
            reason = 'LOCALIZATION_RESET' if scenario == 'odometry-reset' else 'STALE_OR_INVALID_AIRCRAFT_STATE'
            result['passed'] = (injected and terminal.status == 6 and
                terminal.result.result_code == 'ABORTED' and terminal.result.reason == reason and
                not terminal.result.cleanup_confirmed and not terminal.result.mock)
            result['localization_fault_latched'] = node.localization_fault_latched
            result['localization_fault_latency_s'] = next((e['mono']-result['localization_injection']['mono']
                for e in node.events if e['phase'] == 'FAULT'), None)
            result['passed'] &= (result['localization_fault_latency_s'] is not None and
                0 <= result['localization_fault_latency_s'] <= .5 and
                node.localization_fault_latched == (scenario == 'odometry-reset'))
        if os.environ.get('UAV_EXPECT_OBSERVATION')=='1':
            result['observations']=getattr(node,'observation_samples',[])
            if 'map' in node.planned.gate.inputs and 'alignment' in node.planned.gate.inputs:
                m=node.planned.gate.inputs['map'][0];a=node.planned.gate.inputs['alignment'][0]
                result['planning_context_final']=dict(map_id=m.map_id,map_epoch=m.epoch,map_version=m.version,
                    source_stamp_s=m.source_stamp.sec+m.source_stamp.nanosec/1e9,
                    alignment_id=a.alignment_id,generation=a.generation,localization_session=a.localization_session,
                    reset_counters=list(map(int,a.reset_counters)),observed_voxels=sum(m.observed))
            headings=[v['measured_heading_rad'] for v in result['observations'] if v.get('measured_heading_rad') is not None]
            result['observation_measured_yaw_sweep_rad']=sum(math.atan2(math.sin(b-a),math.cos(b-a)) for a,b in zip(headings,headings[1:]))
            result['observation_task_succeeded']=terminal.status==4
            result['observation_cleanup_verified']=(terminal.status==6 and terminal.result.reason=='OBSERVATION_INSUFFICIENT'
                and terminal.result.cleanup_confirmed and result['final_land'] and result['final_arming']==1
                and result['max_truth_displacement_m']>=1.5 and len(waypoint_truth)>=1
                and max((v['angle_rad'] for v in result['observations']),default=0)>=2*math.pi
                and result['observation_measured_yaw_sweep_rad']>=5.8)
            result['passed']=observation_passed(result['passed'],result['observation_cleanup_verified'],node.config['profile'])
        if bt:
            if scenario not in ('runner-exit','runner-stall'):
                bt.process.wait(timeout=5)
                expected_exit=1 if result.get('observation_cleanup_verified') else 130 if scenario=='cancel' else 1 if scenario in ('clock-fault','odometry-stale','odometry-reset') else 0
                result['passed']=result['passed'] and bt.process.returncode==expected_exit
            result['bt_exit_code']=bt.process.poll()
            log=(out/'bt-runner.log').read_text()
            result['bt_dispatch_count']=log.count('Dispatched root mission once')
            result['passed']=result['passed'] and result['bt_dispatch_count']==1
            result['bt_step_accepts'] = [int(i) for i in re.findall(r'BT_STEP_ACCEPTED index=(\d+)', log)]
            result['bt_step_completes'] = [int(i) for i in re.findall(r'BT_STEP_COMPLETE index=(\d+)', log)]
            if scenario in ('full', 'pause-resume') and not result.get('observation_cleanup_verified'):
                result['passed'] &= (result['bt_step_accepts'] == list(range(len(steps))) and
                                     result['bt_step_completes'] == list(range(len(steps))))
            if scenario in ('return-resume-mission','return-hover-resume'):
                index=result.get('return_resume_mission',{}).get('original_step_index',-1)
                expected_accepts=list(range(index+1))+list(range(index,len(steps)))
                result['passed'] &= (result['bt_step_accepts']==expected_accepts and
                    result['bt_step_completes']==list(range(len(steps))) and
                    log.count('BT_MISSION_RESTORED')==1)
            ticks=node.runner_progress_log
            result['bt_progress']=dict(samples=len(ticks),
                max_gap_s=max((b['mono']-a['mono'] for a,b in zip(ticks,ticks[1:])),default=None),
                handshake_s=ticks[0]['mono']-node.events[0]['mono'] if ticks else None,
                loss_to_brake_s=next((e['mono']-ticks[-1]['mono'] for e in node.events
                                      if e['phase']=='LEASE_BRAKE'),None) if ticks else None)
        result['outputs_at_end']=node.output_count
        if truth:result['final_truth']=truth[-1]['position']
        if 'vehicle_status' in node.samples:result['final_arming']=node.samples['vehicle_status'].arming_state
        if 'vehicle_land_detected' in node.samples:result['final_land']=node.samples['vehicle_land_detected'].landed
        if scenario=='clock-fault':result['passed']=result['passed'] and result.get('outputs_at_action_result')==node.output_count
        if scenario.startswith('local-land-'):
            result['passed']=(flight_case.evidence.get('passed',False) and terminal.status==6
                and terminal.result.reason=='PILOT_TAKEOVER' and result['final_land'] and result['final_arming']==1
                and bt.process.returncode==1)
        result['no_gnss_case']=flight_case.evidence
        if scenario=='clock-fault':result['clock_fault_latched']=node.clock_fault_latched
        if scenario=='clock-fault':result['passed']=(terminal.status==6 and terminal.result.reason in ('CLOCK_FAULT','STALE_OR_INVALID_AIRCRAFT_STATE') and node.clock_fault_latched)
    except Exception as exc:
        result['error']=str(exc)
        if scenario in ('pilot-intervention','local-land-manual','local-land-position'):
            with node.lock:
                if node.active_goal and not node.result:node.fault('ACCEPTANCE_CLIENT_ABORT')
            until=time.monotonic()+3
            while node.active_goal and time.monotonic()<until:time.sleep(.02)
            status=node.samples.get('vehicle_status')
            if node.operator_enabled and status and status.nav_state in (status.NAVIGATION_STATE_POSCTL,status.NAVIGATION_STATE_ACRO):
                with node.lock:accepted,reason=node.operator_command('LAND')
                result['acceptance_cleanup_land']=dict(accepted=accepted,reason=reason)
    finally:
        result['no_gnss_case']=flight_case.evidence
        if (not result['passed'] or scenario in ('clock-fault','odometry-stale','odometry-reset')) and node.samples.get('vehicle_status') and node.samples['vehicle_status'].arming_state==2:
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
        if scenario in ('odometry-stale','odometry-reset'):
            result['passed'] = (result['passed'] and result.get('fallback_landed_disarmed', False) and
                result.get('outputs_at_action_result') == node.output_count and result.get('bt_step_accepts', [0,1]) == [0,1])
        result['localized_odometry']=dict(samples=len(localized_samples),
            sessions=sorted({s['session'] for s in localized_samples}),
            valid=bool(localized_samples) and all(s['session']==node.instance and s['frame']=='odom' and
                s['child']=='base_link' and s['stamp_matches'] for s in localized_samples))
        result['passed'] = result['passed'] and result['localized_odometry']['valid']
        if node.localization_profile=='FLOW_LOCAL':
            air=[s for s in flow_samples if s['phase'] in ('TAKEOFF','HOVER')]
            standalone=(len(air)>=30 and all(s['visual_writers']==0 and s['vio_writers']==0 and
                s['flags'].get('cs_opt_flow') and s['flags'].get('cs_rng_hgt') and
                not any(s['flags'].get(n,True) for n in ('cs_ev_pos','cs_ev_vel','cs_ev_hgt','cs_ev_yaw',
                    'cs_gnss_pos','cs_gnss_vel')) for s in air))
            result['standalone_flow']=dict(passed=standalone,airborne_samples=len(air),
                no_vio_subscriptions=not any('source'==n or n.startswith('ev_') for n in node.vio_topics),
                max_hover_drift_m=max((h['max_drift_m'] for h in result.get('hovers',[])),default=None))
            result['passed'] &= standalone and result['standalone_flow']['no_vio_subscriptions']
            write_json(out/'standalone-flow-samples.json',flow_samples)
        result['observation_grid_exports']={}
        for key,(grid,metadata) in getattr(node,'observation_grid_snapshots',{}).items():
            import numpy as np
            filename='observation-grid-'+key+'.npz'
            np.savez_compressed(out/filename,origin=grid.origin,resolution=grid.resolution,
                distance=grid.distance,observed=grid.observed)
            metadata=dict(metadata,grid_file=filename,grid_sha256=file_hash(out/filename))
            write_json(out/('observation-grid-'+key+'.json'),metadata)
            result['observation_grid_exports'][key]=metadata
        write_json(out/'localized-odometry.json',localized_samples)
        write_json(out/'map-diagnostics.json',map_diagnostics)
        write_json(out/'flight-events.json',node.events)
        write_json(out/'flight-diagnostics.json',node.diagnostics)
        write_json(out/'flight-status-history.json',node.status_history)
        write_json(out/'flight-commands.json',node.commands)
        write_json(out/'flight-trace.json',node.trace)
        write_json(out/'flight-truth.json',truth)
        write_json(out/'bt-progress.json',node.runner_progress_log)
        try:
            from rosidl_runtime_py.convert import message_to_ordereddict
            write_json(out/'accepted-plans.json',[dict(authorization=authorization,
                bound=message_to_ordereddict(message)) for message,authorization in node.accepted_plans])
        except (TypeError,ValueError,OSError,ImportError) as error:
            result['evidence_export_error']=str(error)
            result['passed']=False
        write_json(out/'flight-observation.pending.json',result)
        os.replace(out/'flight-observation.pending.json',out/'flight-observation.json')
        if os.environ.get('UAV_FLIGHT_HOLD_RECEIPT')=='1':
            while not (out/'release-flight-gateway').exists():time.sleep(.02)
        if bt:bt.close()
        elif prewarmed:
            if prewarmed[0].poll() is None:prewarmed[0].terminate()
            prewarmed[0].wait(timeout=5);prewarmed[1].close()
        truth_process.terminate()
        truth_process.wait(timeout=5);truth_thread.join(timeout=2)
        executor.shutdown();thread.join(timeout=3);client.destroy();node.destroy_node();rclpy.try_shutdown()
        write_json(out/'flight-gc-pauses.json',gc_pauses.stop())
        flight_gc.restore()
        write_json(out/'flight-gc-policy.json',flight_gc.receipt())
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
