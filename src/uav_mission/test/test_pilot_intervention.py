from types import SimpleNamespace as NS
import time
import pytest
from uav_mission.pilot_intervention import PilotIntervention
from px4_msgs.msg import VehicleStatus


class Harness(PilotIntervention):
    def __init__(self):
        from unique_identifier_msgs.msg import UUID
        self.instance='owned-test'
        self.operator_enabled=self.operator_arm_enabled=self.operator_navigation_enabled=True
        self.active_goal=NS(goal_id=UUID(uuid=[1]*16));self.result=None
        self.land_committed=self.canceling=False
        self.navigation_backend='EGO';self.vio_gate=NS(pose_only=True)
        self.samples={'vehicle_status':NS(nav_state=14),
            'manual_control_setpoint':NS(roll=0.,pitch=0.,yaw=0.,throttle=0.),
            'vehicle_local_position':NS(heading=1.2)}
        self.phase='PAUSED';self.owner='HOLD_CONTROLLER';self.generation=4
        self.saved_hover=7.;self.saved_step=3;self.target=(1.,2.,3.)
        self.reference=self.target;self.point=(1.,2.,3.)
        self.pilot_control=None;self.commands=[];self.streams=[];self.services=[];self.faults=[];self.heartbeats=0
        self.pause_until=time.monotonic()+60;self.deadline=time.monotonic()+100
        self.home=NS(position_enu=(0.,0.,0.));self.stopped=[]
        self.config={'body_radius_m':.5,'tracking_margin_m':.3}
        self.planned=NS(gate=NS(grid=NS(resolution=.2,collision=lambda p,r:False)))
        self.alignment=NS(to_map=lambda p:p)
        self.region=lambda _:True
    def operator_input_ready(self):return True
    def healthy(self):return True
    def speed(self):return 0.
    def planning_ready(self):pass
    def capture_return_home(self):return self.home
    def px4_home(self):return self.home
    def position(self):return self.point
    def command(self,*args):self.commands.append(args)
    def change(self,phase,reason):self.phase=phase;self.reason=reason
    def stream(self):self.streams.append(self.reference)
    def offboard_heartbeat(self):self.heartbeats+=1
    def fault(self,reason):self.faults.append(reason)
    def begin_stop(self,phase):self.phase=phase;self.stopped.append(phase)
    def service(self,verb,request,response):self.services.append((verb,request));response.accepted=True;return response


def active():
    h=Harness();assert h.enter_pilot_control()[0]
    h.samples['vehicle_status'].nav_state=VehicleStatus.NAVIGATION_STATE_POSCTL
    assert h.pilot_tick(time.monotonic())
    assert h.owner=='PILOT' and h.pilot_control=='ACTIVE'
    return h


@pytest.mark.parametrize('fault',['uncentered_throttle','uncentered_roll','no_vio','wrong_backend','no_task','landing','not_stopped','not_paused'])
def test_native_pilot_handoff_requires_stopped_vio_task_and_centered_controls(fault):
    h=Harness()
    if fault=='uncentered_throttle':h.samples['manual_control_setpoint'].throttle=-1.
    elif fault=='uncentered_roll':h.samples['manual_control_setpoint'].roll=.3
    elif fault=='no_vio':h.vio_gate=None
    elif fault=='wrong_backend':h.navigation_backend='DIRECT'
    elif fault=='no_task':h.active_goal=None
    elif fault=='landing':h.land_committed=True
    elif fault=='not_stopped':h.speed=lambda:.2
    elif fault=='not_paused':h.phase='NAVIGATE'
    assert not h.enter_pilot_control()[0] and not h.commands


def test_position_confirmation_releases_stream_and_retains_hover_anchor():
    h=Harness();assert h.enter_pilot_control()[0]
    assert h.commands==[(176,1,3)] and h.owner=='HOLD_CONTROLLER'
    h.pilot_tick(time.monotonic());assert len(h.streams)==1
    h.samples['vehicle_status'].nav_state=VehicleStatus.NAVIGATION_STATE_POSCTL
    h.pilot_tick(time.monotonic());h.point=(2.,3.,3.);h.pilot_tick(time.monotonic())
    assert len(h.streams)==1 and h.owner=='PILOT'
    assert h.saved_hover_join==(3,(1.,2.,3.),7.)


@pytest.mark.parametrize('verb',['resume','finish','CANCEL','LEASE_LOST'])
def test_resume_brakes_from_displaced_pose_before_replanning(verb):
    h=active();identity=h.active_goal;deadline=h.deadline
    h.point=(2.,3.,3.);now=time.monotonic()
    assert h.request_pilot_reacquire(verb,now)[0]
    h.pilot_tick(now+1.)
    assert not h.streams and h.heartbeats==1 and h.commands==[(176,1,3)]
    h.pilot_tick(now+1.6);assert h.commands[-1]==(176,1,6) and not h.stopped and not h.streams
    h.samples['vehicle_status'].nav_state=14;h.pilot_tick(now+1.7)
    assert h.streams[-1]==h.point
    assert h.stopped==['LEASE_BRAKE' if verb=='LEASE_LOST' else 'CANCEL_BRAKE' if verb=='CANCEL' else 'PAUSING']
    assert not h.pilot_tick(now+2.) and not h.services
    if verb in ('resume','finish'):
        h.phase='PAUSED';h.owner='HOLD_CONTROLLER';h.pilot_tick(now+4.)
        assert h.services[0][0]==verb and h.services[0][1].mission_uuid is identity.goal_id
        assert h.pilot_control is None
    assert h.active_goal is identity and h.deadline==deadline and not h.faults


def test_home_change_during_reacquire_cannot_retarget_return():
    h=active();now=time.monotonic();assert h.request_pilot_reacquire('finish',now)[0]
    h.home=NS(position_enu=(9.,9.,0.));h.pilot_control='BRAKING'
    h.pilot_tick(now+4.)
    assert h.faults==['PILOT_RETURN:PX4_RETURN_HOME_CHANGED'] and not h.services


def test_resume_rejects_unready_sticks_or_map_without_ownership_change():
    h=active();h.samples['manual_control_setpoint'].throttle=-1.
    assert not h.request_pilot_reacquire('resume',time.monotonic())[0]
    assert h.pilot_control=='ACTIVE' and h.owner=='PILOT'
    h.samples['manual_control_setpoint'].throttle=0.
    def unavailable():raise ValueError('UNKNOWN_MAP')
    h.planning_ready=unavailable
    assert not h.request_pilot_reacquire('resume',time.monotonic())[0]
    assert h.pilot_control=='ACTIVE' and h.owner=='PILOT'


@pytest.mark.parametrize('fault',['start_volume','target_volume','moving'])
def test_unsafe_resume_keeps_native_pilot_owner_and_never_prestreams(fault):
    h=active();h.point=(2.,3.,3.);before=h.generation
    if fault=='moving':h.speed=lambda:.2
    else:h.planned.gate.grid.collision=lambda p,r:p==(h.point if fault=='start_volume' else h.target)
    assert not h.request_pilot_reacquire('resume',time.monotonic())[0]
    assert h.owner=='PILOT' and h.pilot_control=='ACTIVE' and h.generation==before
    assert h.commands==[(176,1,3)] and not h.streams


def test_unconfirmed_modes_do_not_grant_control():
    h=Harness();h.enter_pilot_control();h.pilot_tick(h.pilot_transition_until+.01)
    assert h.faults==['PILOT_POSITION_NOT_CONFIRMED']
    h=active();now=time.monotonic();h.request_pilot_reacquire('resume',now)
    h.pilot_tick(now+2);h.pilot_tick(now+6)
    assert h.faults==['PILOT_OFFBOARD_NOT_CONFIRMED'] and not h.services


def test_bounded_pilot_timeout_aborts_without_fabricating_action_cancel():
    h=active();h.pilot_tick(h.pause_until+.01)
    assert h.canceling and h.pilot_pending=='CANCEL' and h.pilot_control=='PRESTREAM'
    assert h.pilot_abort_reason=='PILOT_INTERVENTION_TIMEOUT'


def test_gateway_resume_service_is_identity_bound_and_idempotent():
    import threading
    from uav_mission.px4_flight import FlightServer
    from uav_nav_interfaces.srv import ResumeMission
    from unique_identifier_msgs.msg import UUID
    h=active();h.lock=threading.RLock();h.requests={}
    request=ResumeMission.Request(mission_uuid=h.active_goal.goal_id,coordinator_instance=h.instance,
        request_id=UUID(uuid=[4]*16))
    accepted=FlightServer.service(h,'resume',request,ResumeMission.Response())
    assert accepted.accepted and h.pilot_control=='PRESTREAM'
    generation=h.generation
    replay=FlightServer.service(h,'resume',request,ResumeMission.Response())
    assert replay.accepted and h.generation==generation
    request.coordinator_instance='old'
    assert FlightServer.service(h,'resume',request,ResumeMission.Response()).reason=='STALE_IDENTITY'
