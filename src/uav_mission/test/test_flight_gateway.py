import json
import threading
from types import SimpleNamespace

import pytest
from rclpy.action import GoalResponse,CancelResponse
from uav_nav_interfaces.action import ExecuteMission
from uav_mission.px4_flight import FlightServer
from uav_mission.flight_geometry import Alignment,in_region


def fixture():
    return SimpleNamespace(nonce='owned-test',position=lambda:(0.,0.,0.),
                           alignment=Alignment(),region=in_region)


def request(steps=None,authorization='owned-test'):
    return ExecuteMission.Goal(backend='PX4_KNOWN_REGION',mission_type='FLIGHT_SEQUENCE',timeout_s=120.,
          parameters_json=json.dumps(dict(authorization=authorization,steps=steps or
                               [dict(type='TAKEOFF',height_m=2.),dict(type='LAND')])))


def test_recipe_explicit_authorization_and_complete_landing():
    f=fixture()
    assert len(FlightServer.parse(f,request()))==2
    with pytest.raises(ValueError):FlightServer.parse(f,request(authorization='old-token'))
    with pytest.raises(ValueError):FlightServer.parse(f,request([dict(type='TAKEOFF',height_m=2.)]))


@pytest.mark.parametrize('target', [(20.,0.,2.),(0.,0.,-.1),(0.,0.,6.),(float('nan'),0.,2.)])
def test_airborne_route_rejects_ground_or_envelope_violation(target):
    f=fixture()
    req=request([dict(type='TAKEOFF',height_m=2.),dict(type='NAVIGATE',target_map=f.alignment.to_map(target)
                   if all(x==x for x in target) else target),dict(type='LAND')])
    with pytest.raises(ValueError):FlightServer.parse(f,req)


@pytest.mark.parametrize('parameters', ['[]','{}','{"authorization":"owned-test","steps":[null,{}]}'])
def test_malformed_recipe_rejected(parameters):
    r=request();r.parameters_json=parameters
    with pytest.raises((ValueError,KeyError)):FlightServer.parse(fixture(),r)


def test_native_landing_cancel_guard():
    handle=object()
    f=SimpleNamespace(lock=threading.RLock(),active_goal=handle,land_committed=True,phase='LAND_REQUEST')
    assert FlightServer.cancel(f,handle)==CancelResponse.REJECT
    f.phase='LANDING'
    assert FlightServer.cancel(f,handle)==CancelResponse.REJECT


def test_unknown_state_and_clock_fault_never_admit_goal():
    f=SimpleNamespace(lock=threading.RLock(),clock_fault_latched=True,active_goal=None,reserved=False,owner='NONE')
    assert FlightServer.goal(f,request())==GoalResponse.REJECT
    f.clock_fault_latched=False;f.healthy=lambda ground:False
    assert FlightServer.goal(f,request())==GoalResponse.REJECT


def test_runner_lease_rejects_old_identity_and_replays():
    from uav_nav_interfaces.msg import MissionProgress
    from unique_identifier_msgs.msg import UUID
    handle=SimpleNamespace(goal_id=UUID(uuid=[1]*16))
    f=SimpleNamespace(lock=threading.RLock(),active_goal=handle,runner_required=True,
                      runner_lost=False,result=None,instance='current',runner_sequence=3,runner_last_tick=0.,
                      runner_progress_log=[])
    for identity,sequence,root in [('old',4,[1]*16),('current',3,[1]*16),('current',4,[2]*16)]:
        FlightServer.runner_progress(f,MissionProgress(coordinator_instance=identity,tick_sequence=sequence,
                                                      mission_uuid=UUID(uuid=root)))
        assert f.runner_last_tick==0.
    msg=MissionProgress(coordinator_instance='current',tick_sequence=4,mission_uuid=handle.goal_id)
    FlightServer.runner_progress(f,msg)
    assert f.runner_last_tick>0 and f.runner_sequence==4
    before=f.runner_last_tick
    f.runner_lost=True
    msg.tick_sequence=5
    FlightServer.runner_progress(f,msg)
    assert f.runner_last_tick==before


@pytest.mark.parametrize('armed,landing,expected', [(2,False,'LEASE_BRAKE'),(1,False,'BT_PROGRESS_TIMEOUT'),(2,True,None)])
def test_runner_stall_brakes_or_rejects_before_arming_but_preserves_native_land(armed,landing,expected):
    events=[]
    f=SimpleNamespace(runner_required=True,runner_lost=False,land_committed=landing,canceling=False,
                      runner_last_tick=1.,runner_sequence=1,samples={'vehicle_status':SimpleNamespace(arming_state=armed)},
                      config={'runner_progress_max_age_s':.5,'runner_handshake_timeout_s':5.},
                      fault=events.append,begin_stop=events.append)
    FlightServer.check_runner_progress(f,1.49)
    assert not events
    FlightServer.check_runner_progress(f,1.51)
    assert events==([expected] if expected else [])
    if expected:
        assert f.runner_lost
        FlightServer.check_runner_progress(f,2.)
        assert len(events)==1


def test_runner_requires_current_coordinator():
    f=fixture();f.instance='new'
    r=request();p=json.loads(r.parameters_json)
    p.update(runner_progress_required=True,coordinator_instance='old')
    r.parameters_json=json.dumps(p)
    with pytest.raises(ValueError):FlightServer.parse(f,r)
    p['coordinator_instance']='new';r.parameters_json=json.dumps(p)
    assert len(FlightServer.parse(f,r))==2


def test_runner_first_tick_handshake_is_bounded_on_ground():
    events=[]
    f=SimpleNamespace(runner_required=True,runner_lost=False,land_committed=False,canceling=False,
                      runner_last_tick=1.,runner_sequence=0,config={'runner_handshake_timeout_s':5.},
                      samples={'vehicle_status':SimpleNamespace(arming_state=1)},
                      fault=events.append)
    FlightServer.check_runner_progress(f,5.99)
    assert not events
    FlightServer.check_runner_progress(f,6.01)
    assert events==['BT_PROGRESS_TIMEOUT'] and f.runner_lost


def step_fixture():
    import time
    from unique_identifier_msgs.msg import UUID
    from uav_nav_interfaces.msg import ControlSession
    from uav_nav_interfaces.srv import AdvanceFlightStep
    f=SimpleNamespace(lock=threading.RLock(),active_goal=SimpleNamespace(goal_id=UUID(uuid=[1]*16)),
        instance='current',session=UUID(uuid=[2]*16),generation=3,owner='TASK',
        step_controlled=True,runner_required=True,runner_lost=False,canceling=False,result=None,
        land_committed=False,clock_fault_latched=False,healthy=lambda:True,
        runner_last_tick=time.monotonic(),config={'runner_progress_max_age_s':.5},
        step_index=-1,step_grant=None,step_requests={},phase='PRESTREAM',
        steps=[{'type':'TAKEOFF'},{'type':'LAND'}])
    req=AdvanceFlightStep.Request(mission_uuid=UUID(uuid=[1]*16),coordinator_instance=f.instance,
        control_session=ControlSession(session_id=UUID(uuid=[2]*16),generation=3,owner='TASK'),
        request_id=UUID(uuid=[4]*16),step_index=0,step_type='TAKEOFF')
    return f,req,AdvanceFlightStep.Response


def test_step_grant_is_exactly_once_and_request_id_conflicts_reject():
    f,req,response=step_fixture()
    first=FlightServer.advance_step(f,req,response())
    assert first.accepted and f.step_grant==0
    f.phase='ARM_REQUEST'
    repeat=FlightServer.advance_step(f,req,response())
    assert repeat.accepted and repeat.phase==first.phase
    req.step_type='LAND'
    assert FlightServer.advance_step(f,req,response()).reason=='REQUEST_ID_CONFLICT'
    req.request_id.uuid=[5]*16;req.step_type='TAKEOFF'
    assert not FlightServer.advance_step(f,req,response()).accepted


@pytest.mark.parametrize('fault', ['root','instance','generation','session','owner','skip','kind','stopping','lost','clock','health','late'])
def test_step_grants_reject_stale_identity_order_and_faults(fault):
    f,r,response=step_fixture()
    if fault=='root':r.mission_uuid.uuid=[8]*16
    elif fault=='instance':r.coordinator_instance='old'
    elif fault=='generation':r.control_session.generation=2
    elif fault=='session':r.control_session.session_id.uuid=[8]*16
    elif fault=='owner':r.control_session.owner='HOLD_CONTROLLER'
    elif fault=='skip':r.step_index=1;r.step_type='LAND'
    elif fault=='kind':r.step_type='LAND'
    elif fault=='stopping':f.canceling=True
    elif fault=='lost':f.runner_lost=True
    elif fault=='clock':f.clock_fault_latched=True
    elif fault=='health':f.healthy=lambda:False
    elif fault=='late':f.runner_last_tick-=1.
    assert not FlightServer.advance_step(f,r,response()).accepted
    assert f.step_grant is None


def test_step_completion_waits_with_reference_and_retires_child():
    from unique_identifier_msgs.msg import UUID
    events=[]
    f=SimpleNamespace(step_controlled=True,step_grant=None,step_index=0,segment=object(),
                      child=UUID(uuid=[1]*16),reference=(1.,2.,3.),change=events.append)
    FlightServer.next_step(f)
    assert events==['AWAIT_STEP'] and f.step_index==0 and f.segment is None
    assert not any(f.child.uuid) and f.reference==(1.,2.,3.)


def test_step_mode_requires_runner_lease():
    f=fixture();r=request();p=json.loads(r.parameters_json);p['step_controlled']=True
    r.parameters_json=json.dumps(p)
    with pytest.raises(ValueError,match='requires bound runner'):FlightServer.parse(f,r)


def test_initial_clock_discovery_is_not_a_stalled_observed_clock():
    f=SimpleNamespace(clock_last=None,clock_advance=0.,clock_observed=False,clock_fault_latched=False)
    for now in [0.,.6,1.2,5.]:
        FlightServer.update_clock_watchdog(f,now,0.)
        assert not f.clock_fault_latched and not f.clock_observed
    FlightServer.update_clock_watchdog(f,5.1,20.)
    assert f.clock_observed and not f.clock_fault_latched
    FlightServer.update_clock_watchdog(f,5.61,20.)
    assert f.clock_fault_latched


def test_observed_clock_reset_latches_and_fresh_data_cannot_clear_it():
    f=SimpleNamespace(clock_last=None,clock_advance=0.,clock_observed=False,clock_fault_latched=False)
    FlightServer.update_clock_watchdog(f,0.,20.)
    assert FlightServer.update_clock_watchdog(f,.01,19.)
    assert f.clock_fault_latched
    FlightServer.update_clock_watchdog(f,.02,21.)
    assert f.clock_fault_latched


def localization_fixture():
    from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry
    f = SimpleNamespace(lock=threading.RLock(), samples=dict(
        vehicle_local_position=VehicleLocalPosition(),
        vehicle_odometry=VehicleOdometry()), active_goal=object(), result=None,
        owner='TASK', localization_fault_latched=False, odom_valid=True,
        diagnostics=[], faults=[], generation=1, segment=object(), changes=[],
        received={}, status_history=[])
    f.fault=lambda reason:f.faults.append(reason)
    f.change=lambda phase,reason:f.changes.append((phase,reason))
    f.resets=lambda:FlightServer.resets(f)
    f.reset_baseline=f.resets()
    f.check_localization_reset=lambda:FlightServer.check_localization_reset(f)
    return f


@pytest.mark.parametrize('source,field', [
    ('vehicle_local_position', 'xy_reset_counter'),
    ('vehicle_local_position', 'z_reset_counter'),
    ('vehicle_local_position', 'vxy_reset_counter'),
    ('vehicle_local_position', 'vz_reset_counter'),
    ('vehicle_local_position', 'heading_reset_counter'),
    ('vehicle_odometry', 'reset_counter')])
def test_every_reset_retires_localization_before_publishing_tf(source,field):
    from copy import deepcopy
    f=localization_fixture()
    # A counter wrapping from 255 to 0 is also a reset event.
    setattr(f.samples[source],field,255)
    f.reset_baseline=f.resets()
    msg=deepcopy(f.samples[source]);msg.timestamp=10;setattr(msg,field,0)
    FlightServer.callback(f,source)(msg)
    assert f.faults==['LOCALIZATION_RESET'] and f.localization_fault_latched
    assert not f.odom_valid and len(f.diagnostics)==1
    f.check_localization_reset()
    assert len(f.faults)==1  # Recovery samples do not clear or repeat the fault.


def test_reset_of_final_hold_revokes_owner_without_rewriting_root_result():
    f=localization_fixture();f.active_goal=None
    f.result=('CANCELED','STOPPED_AND_HOLDING',True);f.owner='HOLD_CONTROLLER'
    f.samples['vehicle_odometry'].reset_counter=1
    f.check_localization_reset()
    assert f.owner=='NONE' and f.segment is None and f.generation==2
    assert f.result==('CANCELED','STOPPED_AND_HOLDING',True)
    assert f.changes==[('FAULT','FINAL_HOLD_LOCALIZATION_RESET')]


def test_unowned_startup_reset_can_establish_a_new_mission_baseline():
    f=localization_fixture();f.active_goal=None;f.owner='NONE'
    f.samples['vehicle_odometry'].reset_counter=1;f.check_localization_reset()
    assert not f.localization_fault_latched
    f.reset_baseline=f.resets();f.active_goal=object();f.owner='TASK'
    f.check_localization_reset()
    assert not f.faults


@pytest.mark.parametrize('bad', ['missing', 'receive_age', 'publish_age', 'sample_age', 'future_sample', 'zero_sample', 'invalid_pose', 'reset_latch'])
def test_fresh_local_position_does_not_mask_invalid_odometry(bad):
    from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry, VehicleStatus, BatteryStatus
    import time
    local=VehicleLocalPosition(heading_good_for_control=True,xy_valid=True,z_valid=True,
        v_xy_valid=True,v_z_valid=True,eph=.1,epv=.1)
    odom=VehicleOdometry(timestamp=10_000_000,timestamp_sample=10_000_000)
    f=SimpleNamespace(config=dict(status_max_age_s=.75,local_max_age_s=.1,land_max_age_s=1.2),
        localization_fault_latched=False,odom_valid=True,samples=dict(vehicle_local_position=local,
        vehicle_odometry=odom,vehicle_status=VehicleStatus(system_id=8,component_id=1),
        battery_status=BatteryStatus(connected=True,remaining=.9)),received={})
    f.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000))
    for name,msg in f.samples.items():
        msg.timestamp=10_000_000;f.received[name]=time.monotonic()
    f.fresh=lambda name,age:FlightServer.fresh(f,name,age)
    assert FlightServer.healthy(f)
    if bad=='missing':del f.samples['vehicle_odometry']
    elif bad=='receive_age':f.received['vehicle_odometry']-=.2
    elif bad=='publish_age':odom.timestamp-=200_000
    elif bad=='sample_age':odom.timestamp_sample-=200_000
    elif bad=='future_sample':odom.timestamp_sample+=100_000
    elif bad=='zero_sample':odom.timestamp_sample=0
    elif bad=='invalid_pose':f.odom_valid=False
    elif bad=='reset_latch':f.localization_fault_latched=True
    assert not FlightServer.healthy(f)


def test_invalid_odometry_cannot_retain_previous_tf_health():
    from px4_msgs.msg import VehicleOdometry
    f=localization_fixture()
    message=VehicleOdometry(timestamp=10)  # Unknown pose/velocity frame.
    FlightServer.callback(f,'vehicle_odometry')(message)
    assert not f.odom_valid and f.faults==['INVALID_ODOMETRY_FRAME']


def test_localization_fault_removes_reference_owner_and_aborts_root():
    f=localization_fixture()
    f.phase='NAVIGATE'
    f.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_000_000))
    f.received={name:0. for name in f.samples}
    FlightServer.fault(f,'LOCALIZATION_RESET')
    assert f.owner=='NONE' and f.segment is None and f.generation==2
    assert f.result==('ABORTED','LOCALIZATION_RESET',False)
    assert f.changes==[('FAULT','LOCALIZATION_RESET')]


def test_localized_odometry_carries_source_session_counters_and_sample_stamp():
    from nav_msgs.msg import Odometry
    f=localization_fixture();f.instance='unique-gateway-instance'
    f.samples['vehicle_local_position'].vxy_reset_counter=9
    f.samples['vehicle_odometry'].reset_counter=255
    o=Odometry();o.header.frame_id='odom';o.header.stamp.sec=12;o.child_frame_id='base_link'
    result=FlightServer.localized_odometry(f,o)
    assert result.localization_session==f.instance
    assert list(result.reset_counters)==[0,0,9,0,0,255]
    assert result.header==result.odometry.header==o.header


def test_vio_ground_prestream_never_arms_and_stops_on_lost_health():
    calls=[]
    f=SimpleNamespace(clock_fault_latched=False,healthy=lambda ground:True,fresh=lambda name,age:True,
        resets=lambda:(),reset_baseline=(),region=lambda p:True,position=lambda:(0.,0.,0.),
        preflight_until=120.,initial_mode=4,phase_started=100.,last_command=100.,
        get_publishers_info_by_topic=lambda topic:[object()],
        stream=lambda:calls.append('stream'),command=lambda *args:calls.append(args),
        fault=lambda reason:calls.append(reason),samples={
            'vehicle_status':SimpleNamespace(arming_state=1,nav_state=4),
            'vehicle_land_detected':SimpleNamespace(landed=True)})
    FlightServer.preflight_tick(f,102.)
    assert calls==['stream',(176,1,6)]
    f.healthy=lambda ground:False
    FlightServer.preflight_tick(f,102.02)
    assert f.preflight_fault_latched and calls[-1]=='VIO_GROUND_PRESTREAM_FAILED'
    assert len(calls)==3


def test_paced_land_receipt_keeps_original_ros_age_limit():
    from unittest.mock import patch
    f=SimpleNamespace(config={'land_receive_max_age_s':1.5},
        samples={'vehicle_land_detected':SimpleNamespace(timestamp=9_000_000)},
        received={'vehicle_land_detected':98.75},
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)))
    with patch('uav_mission.px4_flight.time.monotonic',return_value=100.):
        assert FlightServer.fresh(f,'vehicle_land_detected',1.2)
        f.samples['vehicle_land_detected'].timestamp=8_700_000
        assert not FlightServer.fresh(f,'vehicle_land_detected',1.2)
        f.samples['vehicle_land_detected'].timestamp=9_000_000;f.received['vehicle_land_detected']=98.4
        assert not FlightServer.fresh(f,'vehicle_land_detected',1.2)
        f.received['vehicle_land_detected']=98.75;f.config={}
        assert not FlightServer.fresh(f,'vehicle_land_detected',1.2)


def test_ego_recipe_is_explicit_and_requires_enabled_live_planning():
    f=fixture();r=request();p=json.loads(r.parameters_json);p['navigation_backend']='EGO';r.parameters_json=json.dumps(p)
    with pytest.raises(ValueError,match='not enabled'):FlightServer.parse(f,r)
    calls=[];f.planned=object();f.planning_ready=lambda:calls.append('checked')
    assert len(FlightServer.parse(f,r))==2 and calls==['checked']
    p['navigation_backend']='unknown';r.parameters_json=json.dumps(p)
    with pytest.raises(ValueError,match='Unsupported navigation'):FlightServer.parse(f,r)


def test_planning_writer_identity_cannot_be_replaced_with_single_new_writer():
    writers={'/planning/source/map':[1],'/planning/source/alignment':[2],'/planning/bound_trajectory':[3]}
    f=SimpleNamespace(get_publishers_info_by_topic=lambda topic:[SimpleNamespace(endpoint_gid=[value]*16) for value in writers[topic]],
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000)),planned=SimpleNamespace(ready=lambda ros,mono:None))
    FlightServer.planning_ready(f)
    writers['/planning/source/map']=[4]
    with pytest.raises(ValueError,match='WRITER_REPLACED'):FlightServer.planning_ready(f)
    writers['/planning/source/map']=[]
    with pytest.raises(ValueError,match='NON_UNIQUE'):FlightServer.planning_ready(f)


@pytest.mark.parametrize('kind',['NAVIGATE','RETURN'])
def test_planned_step_requests_curve_without_creating_direct_segment(kind):
    from unique_identifier_msgs.msg import UUID
    from builtin_interfaces.msg import Time
    calls=[]
    alignment=Alignment()
    f=SimpleNamespace(step_controlled=False,step_index=-1,steps=[dict(type=kind,target_map=alignment.to_map((3.,2.,2.)))],
        navigation_backend='EGO',planned=SimpleNamespace(retire=lambda:None,start=lambda *args:calls.append(args)),
        home=(0.,0.,0.),reference=(0.,0.,2.),region=in_region,alignment=alignment,
        instance='instance',active_goal=SimpleNamespace(goal_id=UUID(uuid=[1]*16)),session=UUID(uuid=[2]*16),
        generation=3,owner='TASK',planning_ready=lambda:None,
        planning_goal_pub=SimpleNamespace(get_subscription_count=lambda:1,publish=lambda m:calls.append(m)),
        get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10_000_000_000,to_msg=lambda:Time(sec=10))))
    f.planning_authorization=lambda:FlightServer.planning_authorization(f)
    f.change=lambda phase:setattr(f,'phase',phase)
    f.fault=lambda reason:pytest.fail(reason)
    FlightServer.next_step(f)
    assert f.phase=='PLAN_REQUEST' and f.segment is None and len(calls)==2
    assert calls[0][1]==f.planning_authorization() and any(calls[0][1][5])
    assert calls[1].header.frame_id=='map'
    assert f.reference==(0.,0.,2.)


def test_begin_stop_revokes_planner_before_child_identity_and_braking():
    from unique_identifier_msgs.msg import UUID
    calls=[]
    f=SimpleNamespace(planned=SimpleNamespace(retire=lambda:calls.append('retired')),segment=object(),child=UUID(uuid=[3]*16),
        position=lambda:(0.,0.,2.),samples={'vehicle_local_position':SimpleNamespace(vx=.1,vy=0.,vz=0.)},
        region=in_region,change=lambda phase:calls.append(phase))
    FlightServer.begin_stop(f,'CANCEL_BRAKE')
    assert calls==['retired','CANCEL_BRAKE'] and not any(f.child.uuid) and f.segment is None


def test_observation_requires_owned_depth_profile_and_ego():
    node=fixture()
    steps=[dict(type='TAKEOFF',height_m=2.),dict(type='OBSERVE',timeout_s=25.),dict(type='LAND')]
    with pytest.raises(ValueError,match='Observation requires'):
        FlightServer.parse(node,request(steps))
    node.config={'observation_enabled':True}
    node.planned=object();node.planning_ready=lambda:None
    req=request(steps);params=json.loads(req.parameters_json);params['navigation_backend']='EGO'
    req.parameters_json=json.dumps(params)
    assert FlightServer.parse(node,req)==steps
    params['steps'][1]['timeout_s']=True;req.parameters_json=json.dumps(params)
    with pytest.raises(ValueError,match='deadline'):FlightServer.parse(node,req)


def test_observation_timeout_retires_before_native_landing(monkeypatch):
    import math
    from uav_mission import ego_execution
    from unique_identifier_msgs.msg import UUID
    order=[]
    monkeypatch.setattr(ego_execution,'BrakingGrid',lambda *args:SimpleNamespace(collision=lambda *args:True,diagnostics=lambda *args:{}))
    node=SimpleNamespace(position=lambda:(0.,0.,2.),target=(0.,0.,2.),
        config={'tracking_margin_m':.3,'observation_yaw_rate_rps':.35,'braking_margin_m':1.2,'body_radius_m':.5},
        observe_started=0.,observe_until=25.,observe_yaw=0.,observation_samples=[],last_observation_sample=0.,
        planning_ready=lambda:None,alignment=Alignment((0.,0.,0.),0.),
        planned=SimpleNamespace(gate=SimpleNamespace(grid=None),retire=lambda:order.append('retire')),
        command=lambda value:order.append(value),change=lambda *args:order.append(args),child=UUID(),segment=object())
    FlightServer.observation_tick(node,25.,25.)
    assert order==['retire',21,('LAND_REQUEST','OBSERVATION_INSUFFICIENT')]
    assert node.landing_result==('ABORTED','OBSERVATION_INSUFFICIENT',True)
    assert node.land_committed and node.segment is None and not any(node.child.uuid)
    assert node.observation_samples[-1]['angle_rad']==2*math.pi
    assert not node.observation_samples[-1]['clear']


def test_observation_success_requires_complete_sweep_and_observed_volume(monkeypatch):
    from uav_mission import ego_execution
    results=[]
    monkeypatch.setattr(ego_execution,'BrakingGrid',lambda *args:SimpleNamespace(collision=lambda *args:False,diagnostics=lambda *args:{}))
    node=SimpleNamespace(position=lambda:(0.,0.,2.),target=(0.,0.,2.),
        config={'tracking_margin_m':.3,'observation_yaw_rate_rps':.35,'braking_margin_m':1.2,'body_radius_m':.5},
        observe_started=0.,observe_until=25.,observe_yaw=0.,observation_samples=[],last_observation_sample=0.,
        planning_ready=lambda:None,alignment=Alignment((0.,0.,0.),0.),
        planned=SimpleNamespace(gate=SimpleNamespace(grid=None)),stable=lambda *args:True,
        next_step=lambda:results.append('next'))
    FlightServer.observation_tick(node,10.,10.)
    assert not results
    FlightServer.observation_tick(node,20.,20.)
    assert results==['next']


def test_multiview_recipe_rejects_unsafe_survey_or_insufficient_root_deadline():
    from pathlib import Path
    node=fixture();node.config=json.loads(Path('simulation/safe_regions/depth_reference.json').read_text())
    node.region=lambda p:in_region(p,node.config['bounds_min'],node.config['bounds_max'],2.)
    node.planned=object();node.planning_ready=lambda:None
    steps=[dict(type='TAKEOFF',height_m=2.),dict(type='OBSERVE',timeout_s=160.),dict(type='LAND')]
    req=request(steps);req.timeout_s=210.
    params=json.loads(req.parameters_json);params['navigation_backend']='EGO';req.parameters_json=json.dumps(params)
    assert FlightServer.parse(node,req)==steps
    req.timeout_s=180.
    with pytest.raises(ValueError,match='landing deadline'):FlightServer.parse(node,req)
    req.timeout_s=210.;node.config['observation_offsets_enu'][0][0]=1.01
    with pytest.raises(ValueError,match='OUTSIDE_KNOWN_REGION'):FlightServer.parse(node,req)


def test_multiview_clear_map_does_not_authorize_navigation_before_return(monkeypatch):
    from uav_mission import ego_execution
    monkeypatch.setattr(ego_execution,'BrakingGrid',lambda *args:SimpleNamespace(collision=lambda *args:False,diagnostics=lambda *args:{}))
    advanced=[]
    survey=SimpleNamespace(sample=lambda *args:((.8,.8,1.),6.3,False),state='SCAN',index=1)
    node=SimpleNamespace(survey=survey,position=lambda:(.8,.8,1.),target=(0.,0.,2.),
        config={'tracking_margin_m':.3,'braking_margin_m':1.2,'body_radius_m':.5},
        observe_until=160.,observe_yaw=0.,observation_samples=[],last_observation_sample=0.,
        planning_ready=lambda:None,alignment=Alignment((0.,0.,0.),0.),
        planned=SimpleNamespace(gate=SimpleNamespace(grid=None,inputs={})),stable=lambda *args:True,
        samples={'vehicle_local_position':SimpleNamespace(vx=0.,vy=0.,vz=0.,heading=0.)},
        next_step=lambda:advanced.append(True))
    FlightServer.observation_tick(node,100.,100.)
    assert not advanced
    survey.sample=lambda *args:((0.,0.,2.),25.2,True);survey.state='DONE'
    node.position=lambda:(0.,0.,2.)
    node.samples['vehicle_local_position'].vx=.06
    FlightServer.observation_tick(node,101.,101.)
    assert not advanced
    node.samples['vehicle_local_position'].vx=0.
    FlightServer.observation_tick(node,102.,102.)
    assert advanced==[True]
