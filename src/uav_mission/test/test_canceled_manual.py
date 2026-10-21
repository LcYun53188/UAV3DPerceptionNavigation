"""Real gateway handoff methods: canceled root -> held reference -> native sticks."""
from types import MethodType, SimpleNamespace as NS
import time

import pytest
from px4_msgs.msg import VehicleStatus
from uav_mission.px4_flight import FlightServer
from test_return_checkpoint import gateway


def stopped():
    f,_,events,_=gateway(backend='EGO')
    f.last_terminal_goal_id=bytes(f.active_goal.goal_id.uuid)
    f.active_goal=None
    f.result=('CANCELED','STOPPED_AND_HOLDING',True)
    f.phase='COMPLETE';f.owner='HOLD_CONTROLLER'
    f.operator_enabled=f.operator_arm_enabled=True
    f.operator_navigation_enabled=True
    f.pilot_abort_reason=None
    f.pilot_control=f.pilot_pending=None
    f.cancel_manual_state=None
    f.vio_gate=NS(joint_sim=True,pose_only=True)
    f.final_hold_until=time.monotonic()+30.
    f.samples['vehicle_status']=NS(system_id=8,component_id=1,failsafe=False,
        nav_state=VehicleStatus.NAVIGATION_STATE_OFFBOARD,arming_state=2)
    f.samples['manual_control_setpoint']=NS(roll=0.,pitch=0.,yaw=0.,throttle=-1.)
    f.operator_input_ready=lambda:True
    f.fresh=lambda *_:True
    f.stream=lambda:events.append(('stream',f.reference))
    f.update_clock_watchdog=lambda *_:False
    f.publish_control=lambda *_:None
    f.clock_advance=time.monotonic()
    f.last_graph_check=time.monotonic()
    for name in ('pilot_sticks_centered','begin_canceled_manual','canceled_manual_tick','operator_command'):
        setattr(f,name,MethodType(getattr(FlightServer,name),f))
    return f,events


def begin(f):
    assert f.begin_canceled_manual(f.last_terminal_goal_id)[0]


def test_low_throttle_waits_in_hold_past_old_deadline_without_land_or_old_plan():
    f,events=stopped();root=f.last_terminal_goal_id;result=f.result
    begin(f)
    assert f.phase=='CANCEL_WAIT_CENTER' and f.owner=='HOLD_CONTROLLER'
    assert f.segment is None and f.step_grant is None
    assert not hasattr(f,'final_hold_until')
    f.deadline=time.monotonic()-100
    for dt in [0.,31.,120.]:f.canceled_manual_tick(time.monotonic()+dt)
    assert not [e for e in events if e[0]=='command']
    assert len([e for e in events if e[0]=='stream'])==3
    assert f.last_terminal_goal_id==root and f.result==result and f.active_goal is None


def test_centered_handoff_keeps_hold_until_actual_position_then_no_streams():
    f,events=stopped();begin(f)
    f.samples['manual_control_setpoint'].throttle=0.
    now=time.monotonic();f.canceled_manual_tick(now)
    assert f.cancel_manual_state=='POSITION_REQUEST' and f.owner=='HOLD_CONTROLLER'
    assert [e[1] for e in events if e[0]=='command']==[(176,1,3)]
    f.canceled_manual_tick(now+.2)
    f.samples['vehicle_status'].nav_state=VehicleStatus.NAVIGATION_STATE_POSCTL
    f.canceled_manual_tick(now+.3)
    assert f.owner=='PILOT' and f.phase=='CANCELED_MANUAL'
    count=len(events)
    f.samples['manual_control_setpoint'].throttle=.5
    f.canceled_manual_tick(now+120.)
    FlightServer.tick(f)
    assert len(events)==count and f.owner=='PILOT'
    assert f.result==('CANCELED','STOPPED_AND_HOLDING',True)


@pytest.mark.parametrize('bad',['root','active','aborted','owner','phase','landing','lease',
    'pilot_abort','input','sa','health','vio_only','no_pose','mode','disarmed'])
def test_only_clean_explicit_terminal_can_enter_manual_hold(bad):
    f,events=stopped();root=f.last_terminal_goal_id
    if bad=='root':root=b'other-root'
    elif bad=='active':f.active_goal=object()
    elif bad=='aborted':f.result=('ABORTED','BT_PROGRESS_TIMEOUT',True)
    elif bad=='owner':f.owner='NONE'
    elif bad=='phase':f.phase='FAULT'
    elif bad=='landing':f.land_committed=True
    elif bad=='lease':f.runner_lost=True
    elif bad=='pilot_abort':f.pilot_abort_reason='PILOT_INTERVENTION_TIMEOUT'
    elif bad=='input':f.operator_input_ready=lambda:False
    elif bad=='sa':f.operator_arm_enabled=False
    elif bad=='health':f.healthy=lambda:False
    elif bad=='vio_only':f.vio_gate.joint_sim=False
    elif bad=='no_pose':f.vio_gate.pose_only=False
    elif bad=='mode':f.samples['vehicle_status'].nav_state=18
    elif bad=='disarmed':f.samples['vehicle_status'].arming_state=1
    assert not f.begin_canceled_manual(root)[0]
    assert not events and hasattr(f,'final_hold_until')


@pytest.mark.parametrize('bad',['roll','pitch','yaw','throttle','moving'])
def test_handoff_never_uses_uncentered_sticks_or_moving_stop(bad):
    f,events=stopped();begin(f)
    f.samples['manual_control_setpoint'].throttle=0.
    if bad=='moving':f.speed=lambda:.3
    else:setattr(f.samples['manual_control_setpoint'],bad,.3)
    f.canceled_manual_tick(time.monotonic())
    assert f.cancel_manual_state=='WAIT_CENTER'
    assert not [e for e in events if e[0]=='command']


@pytest.mark.parametrize('bad',['input','vio','reset','clock','writer','region','owner'])
def test_waiting_hold_withdraws_on_failed_protection_without_rewriting_root(bad):
    f,events=stopped();begin(f);result=f.result
    if bad=='input':f.operator_input_ready=lambda:False
    elif bad=='vio':f.healthy=lambda:False
    elif bad=='reset':f.resets=lambda:(1,)*6
    elif bad=='clock':f.clock_fault_latched=True
    elif bad=='writer':f.get_publishers_info_by_topic=lambda _:[1,2]
    elif bad=='region':f.region=lambda _:False
    elif bad=='owner':f.owner='NONE'
    f.canceled_manual_tick(time.monotonic())
    assert f.owner=='NONE' and f.phase=='FAULT' and f.cancel_manual_state is None
    assert f.result==result and not [e for e in events if e[0]in ('command','stream')]


def test_unconfirmed_position_uses_existing_native_landing_fallback():
    f,events=stopped();begin(f)
    f.samples['manual_control_setpoint'].throttle=0.
    f.canceled_manual_tick(time.monotonic())
    now=f.cancel_manual_transition_until+.01
    f.canceled_manual_tick(now)
    assert f.cancel_manual_state is None and f.owner=='HOLD_CONTROLLER'
    assert f.phase=='COMPLETE' and f.final_hold_until==now
    f.final_hold_until=time.monotonic()-1
    FlightServer.tick(f)
    assert f.phase=='FINAL_NATIVE_LAND' and f.owner=='AUTOPILOT'
    assert [e[1]for e in events if e[0]=='command']==[(176,1,3),(21,)]


@pytest.mark.parametrize('native_position',[False,True])
def test_sd_can_land_both_waiting_hold_and_canceled_manual(native_position):
    f,events=stopped();begin(f)
    if native_position:
        f.samples['vehicle_status'].nav_state=VehicleStatus.NAVIGATION_STATE_POSCTL
        f.canceled_manual_tick(time.monotonic())
    f.samples['vehicle_attitude']=NS(q=[1.,0.,0.,0.])
    f.samples['battery_status']=NS(connected=True,warning=0,remaining=.8)
    f.samples['vehicle_local_position']=NS(z_valid=True,v_z_valid=True,z=-2.,vz=0.,epv=.1)
    f.samples['vehicle_land_detected']=NS(landed=False)
    assert f.operator_command('LAND')==(True,'NATIVE_LOCAL_LAND_REQUESTED')
    assert f.owner=='AUTOPILOT' and f.phase=='FINAL_NATIVE_LAND'
    assert f.cancel_manual_state is None and f.land_committed
    assert [e[1]for e in events if e[0]=='command']==[(21,)]


@pytest.mark.parametrize('exit_kind',['mode','disarm'])
def test_native_mode_exit_does_not_reacquire_offboard_or_restart_root(exit_kind):
    f,events=stopped();begin(f)
    f.samples['vehicle_status'].nav_state=VehicleStatus.NAVIGATION_STATE_POSCTL
    f.canceled_manual_tick(time.monotonic())
    if exit_kind=='mode':f.samples['vehicle_status'].nav_state=18
    else:f.samples['vehicle_status'].arming_state=1
    f.canceled_manual_tick(time.monotonic())
    assert f.owner=='NONE' and f.cancel_manual_state is None
    assert f.phase=='CANCELED_MANUAL_EXITED' and f.active_goal is None
    assert not [e for e in events if e[0]in ('command','stream')]


def test_late_fault_retires_manual_handoff_without_rewriting_canceled_result():
    f,_=stopped();begin(f);result=f.result
    f.received={key:time.monotonic() for key in f.samples};f.diagnostics=[]
    for sample in f.samples.values():sample.timestamp=1_000_000_000
    FlightServer.fault(f,'OPERATOR_ARM_DISABLED')
    assert f.result==result and f.cancel_manual_state is None
    assert f.owner=='NONE' and f.phase=='FAULT'
