"""SD edges drive the real return/checkpoint gateway; no aircraft outputs."""
from copy import deepcopy
import time
from types import SimpleNamespace as NS

import pytest
from unique_identifier_msgs.msg import UUID
from uav_nav_interfaces.srv import RequestReturn, ResumeOriginalMission
from uav_mission.px4_flight import FlightServer
from pocket_operator_session import SdReturn, switches_for
from test_return_checkpoint import gateway, grant, request, restore_request


def state():
    f, p, events, starts = gateway(backend='EGO')
    f.pilot_control = None
    switches = NS(arm=True,navigation=True,aux_mode=0,safety_held=True)
    verbs = []
    def dispatch(verb):
        verbs.append(verb)
        operation = {'RETURN':'return','PAUSE':'pause','RESUME':'resume','RESUME_MISSION':'restore'}[verb]
        response = f.service(operation,request(f,20+len(verbs)),RequestReturn.Response())
        return response.accepted,response.reason
    return f,p,events,starts,switches,verbs,dispatch


@pytest.mark.parametrize('profile',['pocket-rc-v2','uav-aetr-v1'])
def test_sd_edges_are_debounced_prioritized_and_initial_high_cannot_resume(profile):
    s = switches_for(profile)
    def event(axis,value,at,initial=False):
        s.receive(dict(kind='axis',index=axis,value=value,initial=initial),at)
    for axis,value in ((4,32767),(5,32767),(6,0),(7,32767)):
        event(axis,value,0.,True)
    assert s.tick(.2)==[]
    event(7,-32767,1.)
    assert s.tick(1.2)==[]
    event(7,32767,2.)
    assert s.tick(2.1)==[]
    assert s.tick(2.2)==['SD_RETURN']
    assert s.tick(10.)==[]
    event(7,-32767,11.)
    assert s.tick(11.2)==['SD_RELEASE']
    assert s.tick(12.)==[]
    event(7,32767,13.)
    s.receive(dict(kind='button',index=2 if profile=='pocket-rc-v2' else 1,value=1,initial=False),13.)
    assert s.tick(13.2)==['SD_RETURN']


@pytest.mark.parametrize('main',[-32767,0])
def test_manual_land_release_does_not_send_any_control(main):
    s = switches_for('pocket-rc-v2')
    for axis,value in ((4,32767),(5,main),(6,0),(7,-32767)):
        s.receive(dict(kind='axis',index=axis,value=value,initial=True),0.)
    for at,value,expected in ((1.,32767,['LAND']),(2.,-32767,[])):
        s.receive(dict(kind='axis',index=7,value=value,initial=False),at)
        assert s.tick(at+.2)==expected


@pytest.mark.parametrize('early',[True,False])
def test_release_brakes_and_restores_original_task_without_repeating_takeoff(monkeypatch,early):
    monkeypatch.setenv('UAV_NAVIGATION_BACKEND','EGO')
    f,p,events,starts,s,verbs,dispatch = state()
    sd = SdReturn();now=time.monotonic();original=deepcopy(f.steps);root=f.active_goal;deadline=f.deadline
    assert sd.press(f,'pocket-rc-v2',dispatch,now)[0]
    assert f.return_checkpoint is not None and f.phase=='RETURN_BRAKE'
    if not early:
        f.return_after_brake=False;f.next_step()
        assert f.phase=='PLAN_REQUEST'
    # A long held return still has a fresh release deadline.
    now+=30.;s.safety_held=False
    assert sd.release(now)[0]
    assert sd.tick(f,s,dispatch,now)[0:2]==('PAUSE',True)
    assert not f.return_after_brake
    assert sd.tick(f,s,dispatch,now+.1) is None
    f.phase='PAUSED';f.owner='HOLD_CONTROLLER';f.pause_until=time.monotonic()+30.
    p[:]=[1.,0.,2.]
    assert sd.tick(f,s,dispatch,now+.2)[0:2]==('RESUME_MISSION',True)
    assert f.phase=='AWAIT_STEP' and f.steps==original and f.plan_revision==2
    assert f.active_goal is root and f.deadline==deadline
    assert verbs==['RETURN','PAUSE','RESUME_MISSION']
    assert grant(f,1).accepted and f.phase=='PLAN_REQUEST'
    assert starts[-1].pose.position.x==4.
    assert not any(kind=='command' for kind,_ in events)
    assert sd.tick(f,s,dispatch,now+.3) is None


@pytest.mark.parametrize('fault',['root','session','coordinator','land','cancel','terminal','arm','mode','sc','no_checkpoint','timeout'])
def test_release_never_restores_an_invalid_or_unrelated_return(monkeypatch,fault):
    monkeypatch.setenv('UAV_NAVIGATION_BACKEND','EGO')
    f,_,_,_,s,verbs,dispatch=state();sd=SdReturn();now=time.monotonic()
    assert sd.press(f,'uav-aetr-v1',dispatch,now)[0]
    sd.release(now);s.safety_held=False
    if fault=='root':f.active_goal.goal_id=UUID(uuid=[9]*16)
    elif fault=='session':f.session=UUID(uuid=[9]*16)
    elif fault=='coordinator':f.instance='other'
    elif fault=='land':f.land_committed=True
    elif fault=='cancel':f.canceling=True
    elif fault=='terminal':f.result=('ABORTED','test',True)
    elif fault=='arm':s.arm=False
    elif fault=='mode':s.navigation=False
    elif fault=='sc':s.aux_mode=1
    elif fault=='no_checkpoint':f.return_checkpoint=None
    elif fault=='timeout':now+=17.
    result=sd.tick(f,s,dispatch,now)
    assert result[1] is False and sd.identity is None and verbs==['RETURN']


def test_pending_return_feedback_can_be_retried_without_repeating_return(monkeypatch):
    monkeypatch.setenv('UAV_NAVIGATION_BACKEND','EGO')
    f,_,_,_,s,verbs,dispatch=state();sd=SdReturn();now=time.monotonic()
    assert sd.press(f,'uav-aetr-v1',dispatch,now)[0]
    sd.release(now);s.safety_held=False
    assert sd.tick(f,s,dispatch,now,pending=True) is None
    sd.pausing=True
    sd.admission('PAUSE',False,'COMMAND_PENDING',now)
    assert sd.tick(f,s,dispatch,now+.1) is None
    assert sd.tick(f,s,dispatch,now+.3)[0]=='PAUSE'
    sd.admission('PAUSE',False,'RESTORE_CONTEXT_CHANGED')
    assert sd.tick(f,s,dispatch,now+.4) is None
    assert verbs==['RETURN','PAUSE']


def test_repress_during_return_brake_pause_resumes_return_instead_of_ending_root(monkeypatch):
    monkeypatch.setenv('UAV_NAVIGATION_BACKEND','EGO')
    f,_,_,_,s,verbs,dispatch=state();sd=SdReturn();now=time.monotonic()
    assert sd.press(f,'uav-aetr-v1',dispatch,now)[0]
    sd.release(now);s.safety_held=False
    assert sd.tick(f,s,dispatch,now)[0]=='PAUSE'
    s.safety_held=True
    assert sd.press(f,'uav-aetr-v1',dispatch,now+.1)[0]
    assert sd.tick(f,s,dispatch,now+.2) is None
    f.phase='PAUSED';f.owner='HOLD_CONTROLLER';f.pause_until=time.monotonic()+30.
    assert sd.tick(f,s,dispatch,now+.3)[0:2]==('RESUME',True)
    assert f.phase=='PLAN_REQUEST' and f.return_checkpoint is not None
    assert verbs==['RETURN','PAUSE','RESUME']


@pytest.mark.parametrize('phase',['TAKEOFF','OBSERVE'])
def test_early_or_unready_map_sd_retains_irreversible_land(monkeypatch,phase):
    monkeypatch.setenv('UAV_NAVIGATION_BACKEND','EGO')
    f,_,_,_,s,verbs,_=state();f.phase=phase
    def unavailable():raise ValueError('MAP_NOT_READY')
    f.planning_ready=unavailable
    sd=SdReturn()
    assert sd.press(f,'pocket-rc-v2',lambda v:(verbs.append(v) or True,'REQUESTED'),time.monotonic())[0]
    assert verbs==['END_MISSION'] and not sd.release()[0]


def observation_gateway():
    f,p,events,starts=gateway(backend='EGO')
    f.phase='OBSERVE';f.steps[1]=dict(type='OBSERVE',timeout_s=50.)
    f.yaw=0.;f.known_region=lambda _:True
    f.config.update(observation_offsets_enu=[[0.,0.,2.],[1.,0.,2.]],observation_yaw_rate_rps=.5,
                    max_speed_mps=.5,max_acceleration_mps2=.5,max_jerk_mps3=1.)
    return f,p,events,starts


def test_scan_restore_plans_back_to_original_anchor_and_restarts_unfinished_scan():
    f,p,events,starts=observation_gateway();anchor=f.target;root=f.active_goal;deadline=f.deadline
    assert f.service('return',request(f),RequestReturn.Response()).accepted
    assert f.return_checkpoint.observation_anchor==anchor
    assert f.service('pause',request(f,4),RequestReturn.Response()).accepted
    f.phase='PAUSED';f.owner='HOLD_CONTROLLER';f.pause_until=time.monotonic()+30.
    p[:]=[1.,0.,2.]
    assert f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted
    assert grant(f,1).accepted and f.phase=='PLAN_REQUEST'
    assert f.join_observation_anchor==anchor and f.target==anchor
    FlightServer.planned_result(f,NS())
    assert f.phase=='NAVIGATE'  # OBSERVE cannot start while returning to its anchor.
    f.reference=anchor;p[:]=anchor
    f.complete_motion(time.monotonic())
    assert f.phase=='OBSERVE' and f.step_index==1 and f.survey.anchor==anchor
    assert f.survey.index==0 and f.observation_samples==[]
    assert f.active_goal is root and f.deadline==deadline and not f.return_override
    assert not any(kind=='command' for kind,_ in events)


def test_pausing_or_returning_again_during_scan_rejoin_keeps_original_anchor():
    f,p,_,_=observation_gateway();anchor=f.target
    assert f.service('return',request(f),RequestReturn.Response()).accepted
    assert f.service('pause',request(f,4),RequestReturn.Response()).accepted
    f.phase='PAUSED';f.owner='HOLD_CONTROLLER';f.pause_until=time.monotonic()+30.
    p[:]=[1.,0.,2.]
    assert f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted
    assert grant(f,1).accepted
    assert f.service('pause',request(f,9),RequestReturn.Response()).accepted
    f.phase='PAUSED';f.owner='HOLD_CONTROLLER';f.pause_until=time.monotonic()+30.
    assert f.service('resume',request(f,10),RequestReturn.Response()).accepted
    assert f.phase=='PLAN_REQUEST' and f.join_observation_anchor==anchor
    assert f.service('return',request(f,11),RequestReturn.Response()).accepted
    assert f.return_checkpoint.observation_anchor==anchor


def test_unready_scan_map_rejects_temporary_return_without_changing_work():
    f,_,events,_=observation_gateway()
    def unavailable():raise ValueError('MAP_NOT_READY')
    f.planning_ready=unavailable
    response=f.service('return',request(f),RequestReturn.Response())
    assert not response.accepted and response.reason=='RETURN_CONTEXT_INVALID:MAP_NOT_READY'
    assert f.phase=='OBSERVE' and not events and f.return_checkpoint is None


def test_second_return_before_restored_scan_authorization_keeps_its_original_anchor():
    f,p,_,_=observation_gateway();anchor=f.target
    assert f.service('return',request(f),RequestReturn.Response()).accepted
    assert f.service('pause',request(f,4),RequestReturn.Response()).accepted
    f.phase='PAUSED';f.owner='HOLD_CONTROLLER';f.pause_until=time.monotonic()+30.
    p[:]=[1.,0.,2.];f.reference=tuple(p)
    assert f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted
    assert f.phase=='AWAIT_STEP'
    assert f.service('return',request(f,11),RequestReturn.Response()).accepted
    assert f.return_checkpoint.observation_anchor==anchor
