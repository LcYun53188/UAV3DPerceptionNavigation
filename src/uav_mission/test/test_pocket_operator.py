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
            arming_state=1,pre_flight_checks_pass=True),
            'vehicle_land_detected': SimpleNamespace(landed=True),
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
    assert FlightServer.operator_command(f,operation)==(False,'REQUESTED_LOCALIZATION_NOT_ACTIVE_OR_READY')
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
    f.active_goal=None;f.healthy=lambda:False
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
