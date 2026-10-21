import threading
from types import SimpleNamespace, MethodType

import pytest
from rclpy.action import GoalResponse
from uav_mission.px4_flight import FlightServer
from test_flight_gateway import request


def operator():
    calls = []
    f = SimpleNamespace(lock=threading.RLock(), operator_enabled=True, operator_arm_enabled=True,
        operator_navigation_enabled=True, clock_fault_latched=False, active_goal=None, owner='NONE',
        land_committed=False, generation=1, segment=None, phase='IDLE', reason='',
        samples={'vehicle_status': SimpleNamespace(system_id=8,component_id=1,failsafe=False,
            arming_state=1,nav_state=2,pre_flight_checks_pass=True),
            'vehicle_land_detected': SimpleNamespace(landed=True),
            'vehicle_attitude': SimpleNamespace(q=[1.,0.,0.,0.]),
            'battery_status': SimpleNamespace(connected=True,warning=0,remaining=1.),
            'vehicle_local_position': SimpleNamespace(z_valid=True,v_z_valid=True,z=-1.,vz=0.,epv=.1),
            'manual_control_setpoint': SimpleNamespace(valid=True,data_source=2,
                roll=0.,pitch=0.,yaw=0.,throttle=-1.)},
        fresh=lambda *_: True, speed=lambda: 0., healthy=lambda **_:True,
        get_publishers_info_by_topic=lambda _: [object()],
        command=lambda *args:calls.append(args), fault=lambda reason:calls.append(('fault',reason)))
    f.operator_input_ready=lambda: FlightServer.operator_input_ready(f)
    from unique_identifier_msgs.msg import UUID
    f.session=UUID(uuid=[3]*16)
    f.home=(0.,0.,0.)
    f.resets=lambda:(0,)*6
    f.alignment=SimpleNamespace(translation=(0.,0.,0.),yaw=0.)
    f.navigation_backend='DIRECT'
    for name in ('save_return_checkpoint','checkpoint_alignment','checkpoint_map_context'):
        setattr(f,name,MethodType(getattr(FlightServer,name),f))
    return f,calls


def test_sa_requests_arm_only_with_real_qgc_input_and_low_centered_sticks():
    f,calls=operator()
    assert FlightServer.operator_command(f,'ARM')==(True,'ARM_REQUESTED')
    assert calls==[(400,1)]


def test_stabilized_rescue_ignores_vio_and_xy_but_keeps_native_attitude_checks():
    f,calls=operator();f.healthy=lambda **_:False;f.vio_healthy=lambda:False
    f.samples['vehicle_status'].failsafe=True
    f.samples.pop('vehicle_local_position')
    assert FlightServer.operator_command(f,'STABILIZED')==(True,'STABILIZED_MODE_REQUESTED')
    assert calls==[(176,1,7)]  # STABILIZED, never ACRO (main mode 5).
    f.samples['vehicle_status'].failsafe=False
    f.samples['vehicle_status'].nav_state=15
    assert FlightServer.operator_command(f,'ARM')==(True,'ARM_REQUESTED')
    f.samples['vehicle_attitude'].q=[float('nan')]*4
    assert not FlightServer.operator_command(f,'STABILIZED')[0]
    assert calls==[(176,1,7),(400,1)]


def test_native_land_does_not_require_vio_or_horizontal_localization():
    f,calls=operator();f.healthy=lambda **_:False
    f.samples['vehicle_land_detected'].landed=False
    f.samples['vehicle_local_position'].xy_valid=False
    f.change=lambda phase,reason:setattr(f,'phase',phase)
    assert FlightServer.operator_command(f,'LAND')[0]
    assert calls==[(21,)]


@pytest.mark.parametrize('fault',['attitude','height','battery','source_age','ev_height','fake_height','no_aid'])
def test_altitude_only_requires_native_height_and_confirmed_non_vio_source(fault):
    import time
    f,calls=operator()
    f.vio_gate=SimpleNamespace(samples={'flags':SimpleNamespace(cs_ev_hgt=False,cs_fake_hgt=False,cs_rng_hgt=True,cs_baro_hgt=True)},received={'flags':time.monotonic()})
    assert FlightServer.operator_command(f,'ALTITUDE')==(True,'ALTITUDE_MODE_REQUESTED')
    assert calls==[(176,1,2)]
    calls.clear()
    if fault=='attitude':f.samples['vehicle_attitude'].q=[0.]*4
    elif fault=='height':f.samples['vehicle_local_position'].z_valid=False
    elif fault=='battery':f.samples['battery_status'].remaining=.1
    elif fault=='source_age':f.vio_gate.received['flags']=0.
    elif fault=='ev_height':f.vio_gate.samples['flags'].cs_ev_hgt=True
    elif fault=='fake_height':f.vio_gate.samples['flags'].cs_fake_hgt=True
    else:f.vio_gate.samples['flags'].cs_rng_hgt=f.vio_gate.samples['flags'].cs_baro_hgt=False
    assert not FlightServer.operator_command(f,'ALTITUDE')[0]
    assert calls==[]


def test_vio_height_and_yaw_flow_horizontal_is_not_pure_flow():
    f,calls=operator();f.vio_gate=SimpleNamespace(flow_horizontal=True,pure_flow=False)
    f.vio_healthy=lambda:True
    assert FlightServer.operator_command(f,'FLOW_POSITION')==(False,'PURE_FLOW_REQUIRES_INDEPENDENT_FLOW_LOCAL_SESSION')
    assert calls==[]


@pytest.mark.parametrize('pure_flow',[True,False])
def test_assisted_position_accepts_only_a_confirmed_active_source(pure_flow):
    f,calls=operator();f.vio_gate=SimpleNamespace(pure_flow=pure_flow);f.vio_healthy=lambda:True
    assert FlightServer.operator_command(f,'ASSISTED_POSITION')[0]
    assert calls==[(176,1,3)]
    calls.clear();f.vio_healthy=lambda:False
    assert not FlightServer.operator_command(f,'ASSISTED_POSITION')[0]
    assert calls==[]


def test_matrix_sa_requires_confirmed_selected_mode_and_another_physical_edge():
    from pocket_operator_session import arm_from_switch,switches_for
    f,calls=operator();s=switches_for('pocket-rc-v2')
    s.levels={4:True,5:-1,6:1,7:False}
    f.operator_command=lambda verb:FlightServer.operator_command(f,verb)
    f.operator_mode_ready=lambda verb:FlightServer.operator_mode_ready(f,verb)
    assert arm_from_switch(f,s)==(False,'ARM_MANUAL_MODE_REQUESTED_RETRY_SWITCH')
    assert calls==[(176,1,7)]
    f.samples['vehicle_status'].nav_state=15
    assert calls==[(176,1,7)]  # Telemetry confirmation never auto-arms.
    assert arm_from_switch(f,s)==(True,'ARM_REQUESTED')
    assert calls==[(176,1,7),(400,1)]


@pytest.mark.parametrize('fault',['uncalibrated','throttle','airborne','busy','wrong_aircraft','second_writer','freshness'])
def test_arm_rejects_missing_authority_without_outputs(fault):
    f,calls=operator()
    if fault=='uncalibrated':f.samples['manual_control_setpoint'].valid=False
    elif fault=='throttle':f.samples['manual_control_setpoint'].throttle=0.
    elif fault=='airborne':f.samples['vehicle_land_detected'].landed=False
    elif fault=='busy':f.active_goal=object()
    elif fault=='wrong_aircraft':f.samples['vehicle_status'].system_id=1
    elif fault=='second_writer':f.get_publishers_info_by_topic=lambda _: [object(),object()]
    elif fault=='freshness':f.fresh=lambda *_:False
    assert not FlightServer.operator_command(f,'ARM')[0]
    assert calls==[]


@pytest.mark.parametrize('fault,expected',[
    ('busy', 'ARM_CONTROL_BUSY'), ('land', 'ARM_NOT_LANDED_OR_MOVING'),
    ('switch', 'ARM_SWITCH_NOT_ENABLED'), ('health', 'ARM_HEALTH_NOT_READY'),
    ('vio', 'ARM_LOCALIZATION_NOT_READY:VIO_SOURCE_LOST'),
    ('preflight', 'PX4_PREFLIGHT_NOT_READY'), ('throttle', 'ARM_THROTTLE_NOT_LOW'),
    ('sticks', 'ARM_STICKS_NOT_CENTERED'),
])
def test_arm_rejection_identifies_the_failed_check_without_commands(fault, expected):
    f,calls=operator()
    if fault=='busy':f.owner='TASK'
    elif fault=='land':f.samples['vehicle_land_detected'].landed=False
    elif fault=='switch':f.operator_arm_enabled=False
    elif fault in ('health','vio'):
        f.healthy=lambda **_:False
        if fault=='vio':f.vio_gate=SimpleNamespace(fault='VIO_SOURCE_LOST',reason='VIO_SOURCE_INVALID')
    elif fault=='preflight':f.samples['vehicle_status'].pre_flight_checks_pass=False
    elif fault=='throttle':f.samples['manual_control_setpoint'].throttle=0.
    elif fault=='sticks':f.samples['manual_control_setpoint'].roll=.5
    assert FlightServer.operator_command(f,'ARM')==(False,expected)
    assert calls==[]


@pytest.mark.parametrize('mode',[3,4,5,14,17,18,20])
def test_sa_cannot_arm_in_a_native_auto_or_offboard_mode(mode):
    f,calls=operator();f.samples['vehicle_status'].nav_state=mode
    assert FlightServer.operator_command(f,'ARM')==(False,'ARM_REQUIRES_MANUAL_MODE')
    assert calls==[]


@pytest.mark.parametrize('mode',[0,1,2,10,15])
def test_sa_retains_explicit_manual_mode_arming(mode):
    f,calls=operator();f.samples['vehicle_status'].nav_state=mode
    assert FlightServer.operator_command(f,'ARM')==(True,'ARM_REQUESTED')
    assert calls==[(400,1)]


def test_switch_mode_preparation_never_queues_or_retries_an_arm():
    from pocket_operator_session import arm_from_switch
    f,calls=operator();f.samples['vehicle_status'].nav_state=17
    f.operator_command=lambda verb:FlightServer.operator_command(f,verb)
    assert arm_from_switch(f,SimpleNamespace())==(False,'ARM_MANUAL_MODE_REQUESTED_RETRY_SWITCH')
    assert calls==[(176,1,3)]
    # The mode ACK alone is insufficient: current PX4 status must confirm it.
    assert f.samples['vehicle_status'].nav_state==17
    f.samples['vehicle_status'].nav_state=2
    assert calls==[(176,1,3)]
    # Only a new explicit switch call can arm after actual mode confirmation.
    assert arm_from_switch(f,SimpleNamespace())==(True,'ARM_REQUESTED')
    assert calls==[(176,1,3),(400,1)]


def test_manual_takeover_retires_task_before_mode_command():
    f,calls=operator();f.active_goal=object();f.owner='TASK';f.segment=object()
    assert FlightServer.operator_command(f,'MANUAL')[0]
    assert calls==[('fault','PILOT_TAKEOVER'),(176,1,3)]
    assert f.owner=='NONE' and f.segment is None


def test_acro_takeover_uses_same_sole_writer_and_retires_task():
    f,calls=operator();f.active_goal=object();f.owner='TASK';f.segment=object()
    assert FlightServer.operator_command(f,'ACRO')==(True,'ACRO_MODE_REQUESTED')
    assert calls==[('fault','PILOT_TAKEOVER'),(176,1,5)]
    assert f.owner=='NONE' and f.segment is None


def test_acro_rejects_missing_input_and_committed_landing():
    f,calls=operator();f.samples['manual_control_setpoint'].valid=False
    assert not FlightServer.operator_command(f,'ACRO')[0]
    assert calls==[]
    f,calls=operator();f.active_goal=object();f.land_committed=True
    assert FlightServer.operator_command(f,'ACRO')==(False,'LANDING_COMMITTED')
    assert calls==[]


@pytest.mark.parametrize('operation', ['VIO_POSITION', 'FLOW_POSITION'])
def test_requested_localization_cannot_silently_use_gnss(operation):
    f,calls=operator()
    expected=('PURE_FLOW_REQUIRES_INDEPENDENT_FLOW_LOCAL_SESSION' if operation=='FLOW_POSITION'
              else 'REQUESTED_LOCALIZATION_NOT_ACTIVE_OR_READY')
    assert FlightServer.operator_command(f,operation)==(False,expected)
    assert calls==[]


@pytest.mark.parametrize('flow', [True, False])
def test_source_mode_requires_matching_ready_gate(flow):
    f,calls=operator();f.vio_gate=SimpleNamespace(pure_flow=flow);f.vio_healthy=lambda:True
    operation='FLOW_POSITION' if flow else 'VIO_POSITION'
    assert FlightServer.operator_command(f,operation)==(True,'POSITION_MODE_REQUESTED')
    assert calls==[(176,1,3)]


def test_manual_local_land_is_committed_native_output_and_cannot_be_overridden():
    f,calls=operator();f.samples['vehicle_land_detected'].landed=False
    f.change=lambda phase,reason:setattr(f,'phase',phase)
    assert FlightServer.operator_command(f,'LAND')==(True,'NATIVE_LOCAL_LAND_REQUESTED')
    assert calls==[(21,)] and f.owner=='AUTOPILOT' and f.land_committed
    assert FlightServer.operator_command(f,'ACRO')==(False,'LANDING_COMMITTED')


def test_local_land_rejects_active_task_and_missing_position():
    f,calls=operator();f.samples['vehicle_land_detected'].landed=False;f.active_goal=object()
    assert FlightServer.operator_command(f,'LAND')==(False,'ACTIVE_TASK_USE_END_MISSION')
    f.active_goal=None;f.samples['vehicle_local_position'].z_valid=False
    assert FlightServer.operator_command(f,'LAND')==(False,'LAND_POSITION_OR_HEIGHT_UNAVAILABLE')
    assert calls==[]


def rtl_operator():
    from px4_msgs.msg import HomePosition,VehicleLocalPosition,VehicleGlobalPosition,FailsafeFlags,VehicleStatus
    f,calls=operator();f.samples['vehicle_status']=VehicleStatus(system_id=8,component_id=1,arming_state=2)
    f.samples['vehicle_land_detected'].landed=False
    f.samples.update(home_position=HomePosition(timestamp=1000000,x=2.,y=3.,z=0.,valid_lpos=True,
        valid_hpos=True,valid_alt=True,lat=47.,lon=8.,alt=488.),
        vehicle_local_position=VehicleLocalPosition(xy_global=True,z_global=True),
        vehicle_global_position=VehicleGlobalPosition(timestamp=10000000,lat=47.,lon=8.,alt=488.),
        failsafe_flags=FailsafeFlags())
    f.home_frame_epoch_us=0;f.diagnostics=[]
    f.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=10000000000))
    def change(phase,reason):f.phase=phase;f.reason=reason
    f.change=change
    return f,calls


def test_manual_return_requests_native_rtl_without_offboard_or_immediate_success():
    f,calls=rtl_operator()
    assert FlightServer.operator_command(f,'RTL')==(True,'PX4_NATIVE_RTL_REQUESTED')
    assert calls==[(20,)] and f.owner=='AUTOPILOT' and f.phase=='NATIVE_RTL_REQUEST'
    assert not f.land_committed
    assert f.diagnostics[-1]['native_return_home']['position_enu']==[3.,2.,0.]
    FlightServer.native_rtl_tick(f,f.native_rtl_requested+1.)
    assert f.phase=='NATIVE_RTL_REQUEST'  # Sending the command is not mode confirmation.
    f.samples['vehicle_status'].nav_state=5
    FlightServer.native_rtl_tick(f,f.native_rtl_requested+2.)
    assert f.phase=='NATIVE_RTL'
    f.samples['vehicle_land_detected'].landed=True;f.samples['vehicle_status'].arming_state=1
    FlightServer.native_rtl_tick(f,f.native_rtl_requested+3.)
    assert f.phase=='NATIVE_RTL_CLOSED' and f.owner=='NONE' and calls==[(20,)]


@pytest.mark.parametrize('fault',['home','global','busy','writers','ground'])
def test_manual_rtl_has_no_output_when_native_requirements_are_missing(fault):
    f,calls=rtl_operator()
    if fault=='home':f.samples['home_position'].valid_hpos=False
    elif fault=='global':f.samples['vehicle_local_position'].xy_global=False
    elif fault=='busy':f.active_goal=object()
    elif fault=='writers':f.get_publishers_info_by_topic=lambda t:[object()]*(2 if 'home_position' in t else 1)
    elif fault=='ground':f.samples['vehicle_land_detected'].landed=True
    assert not FlightServer.operator_command(f,'RTL')[0]
    assert calls==[] and f.owner=='NONE'


def test_unconfirmed_rtl_releases_gateway_without_claiming_a_return():
    f,calls=rtl_operator();FlightServer.operator_command(f,'RTL')
    FlightServer.native_rtl_tick(f,f.native_rtl_requested+4.)
    assert f.owner=='NONE' and f.reason=='PX4_RTL_NOT_CONFIRMED'


def test_explicit_pilot_takeover_releases_native_rtl_ownership():
    f,calls=rtl_operator();FlightServer.operator_command(f,'RTL')
    assert FlightServer.operator_command(f,'ACRO')[0]
    assert f.owner=='NONE' and f.phase=='IDLE' and calls==[(20,),(176,1,5)]


def test_sa_off_airborne_cannot_cut_motors_and_retires_automation():
    f,calls=operator();f.active_goal=object();f.owner='TASK'
    f.samples['vehicle_land_detected'].landed=False
    assert FlightServer.operator_command(f,'DISARM')==(False,'DISARM_REJECTED_AIRBORNE')
    assert calls==[('fault','OPERATOR_ARM_DISABLED')]
    assert f.owner=='NONE'


def test_manual_switch_cannot_interrupt_committed_landing():
    f,calls=operator();f.active_goal=object();f.land_committed=True
    assert FlightServer.operator_command(f,'MANUAL')==(False,'LANDING_COMMITTED')
    assert calls==[]


def test_native_final_hold_landing_cannot_be_overridden_by_sb():
    f,calls=operator();f.owner='AUTOPILOT';f.phase='FINAL_NATIVE_LAND'
    assert FlightServer.operator_command(f,'MANUAL')==(False,'LANDING_COMMITTED')
    assert calls==[]


def test_prearmed_ground_admission_is_limited_to_explicit_operator_profile():
    f,calls=operator();f.reserved=False;f.phase='IDLE';f.healthy=lambda **_:True
    f.samples['vehicle_status'].arming_state=2
    f.parse=lambda _:[];f.position=lambda:(0.,0.,0.);f.region=lambda _:True
    assert FlightServer.goal(f,request())==GoalResponse.ACCEPT
    f.reserved=False;f.operator_enabled=False
    assert FlightServer.goal(f,request())==GoalResponse.REJECT
    f.operator_enabled=True;f.operator_navigation_enabled=False
    assert FlightServer.goal(f,request())==GoalResponse.REJECT


def test_return_request_retires_before_plan_revision_and_preserves_root_deadline():
    from unique_identifier_msgs.msg import UUID
    from uav_nav_interfaces.srv import RequestReturn
    import time
    f,calls=operator()
    f.instance='current';f.active_goal=SimpleNamespace(goal_id=UUID(uuid=[1]*16))
    f.requests={};f.result=None;f.canceling=False;f.return_override=False
    f.runner_required=True;f.runner_lost=False;f.runner_sequence=2;f.runner_last_tick=time.monotonic()
    f.config={'runner_progress_max_age_s':.5,'alignment_id':'test'};f.deadline=123.;f.phase='NAVIGATE'
    f.step_index=1
    f.steps=[dict(type=t) for t in ('TAKEOFF','NAVIGATE','HOVER','RETURN','HOVER','LAND')]
    def brake(phase):
        calls.append(('retire',phase));f.phase=phase
    f.begin_stop=brake
    req=RequestReturn.Request(mission_uuid=f.active_goal.goal_id,coordinator_instance='current',request_id=UUID(uuid=[2]*16))
    response=FlightServer.service(f,'return',req,RequestReturn.Response())
    assert response.accepted and f.deadline==123.
    assert f.return_override and f.return_after_brake and f.step_index==2
    assert [s['type'] for s in f.steps]==['TAKEOFF','NAVIGATE','HOVER','RETURN','LAND']
    assert f.landing_result==('SUCCEEDED','RETURN_LANDED_AND_DISARMED',True)
    assert calls==[('retire','RETURN_BRAKE')]
    assert FlightServer.service(f,'return',req,RequestReturn.Response()).accepted
    assert calls==[('retire','RETURN_BRAKE')]
    assert not FlightServer.service(f,'pause',req,RequestReturn.Response()).accepted


def test_failed_return_checkpoint_does_not_bind_a_new_home_or_brake():
    from unique_identifier_msgs.msg import UUID
    from uav_nav_interfaces.srv import RequestReturn
    import time
    f,calls=rtl_operator();f.config={'return_home_source':'PX4_HOME_POSITION','runner_progress_max_age_s':.5}
    f.instance='current';f.active_goal=SimpleNamespace(goal_id=UUID(uuid=[1]*16));f.requests={}
    f.result=None;f.canceling=False;f.return_override=False;f.phase='NAVIGATE';f.reference=(0.,0.,3.)
    f.navigation_backend='EGO';f.return_home_reference=None;f.region=lambda _:True
    f.runner_required=False;f.step_index=1
    f.steps=[dict(type=t) for t in ('TAKEOFF','NAVIGATE','RETURN','LAND')]
    def invalid(*_):raise ValueError('CHECKPOINT_CHANGED')
    f.save_return_checkpoint=invalid
    f.begin_stop=lambda *_:calls.append('brake')
    req=RequestReturn.Request(mission_uuid=f.active_goal.goal_id,coordinator_instance='current',request_id=UUID(uuid=[2]*16))
    result=FlightServer.service(f,'return',req,RequestReturn.Response())
    assert not result.accepted and f.return_home_reference is None and not f.diagnostics and not calls
