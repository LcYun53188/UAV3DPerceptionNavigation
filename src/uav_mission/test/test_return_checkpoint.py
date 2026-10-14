"""Exercise the real gateway revision/trajectory methods without PX4 or a radio."""
from copy import deepcopy
import threading
import time
from types import MethodType, SimpleNamespace

import pytest
from rclpy.time import Time
from unique_identifier_msgs.msg import UUID
from uav_nav_interfaces.msg import ControlSession
from uav_nav_interfaces.srv import AdvanceFlightStep, RequestReturn, ResumeOriginalMission
from uav_mission.flight_geometry import Alignment, Segment
from uav_mission.px4_flight import FlightServer


def gateway(kind='NAVIGATE', backend='DIRECT'):
    events, starts = [], []
    now = time.monotonic()
    position = [3., 1., 2.]
    f = SimpleNamespace(lock=threading.RLock(), instance='current', active_goal=SimpleNamespace(goal_id=UUID(uuid=[1]*16)),
        session=UUID(uuid=[2]*16), home=(0.,0.,0.), deadline=now+120.,
        phase=kind, result=None, canceling=False, land_committed=False, owner='TASK', generation=3,
        config={'alignment_id':'alignment','runner_progress_max_age_s':.5,'position_tolerance_m':.15},
        alignment=Alignment((0.,0.,0.),0.), navigation_backend=backend, requests={}, step_requests={},
        step_controlled=True, step_grant=None, step_index=1 if kind=='NAVIGATE' else 2,
        return_override=False, return_after_brake=False, return_checkpoint=None, plan_revision=0,
        restored_hover=None, join_hover_remaining=None, runner_required=True, runner_lost=False,
        runner_sequence=5, runner_last_tick=now, reset_baseline=(0,)*6, clock_fault_latched=False,
        healthy=lambda **_:True, resets=lambda:(0,)*6, region=lambda _:True, speed=lambda:0.,
        get_publishers_info_by_topic=lambda _:[object()],
        position=lambda:tuple(position), operator_enabled=False,
        samples={'vehicle_status':SimpleNamespace(nav_state=14,arming_state=2),
                 'vehicle_local_position':SimpleNamespace(vx=0.,vy=0.,vz=0.)},
        reference=tuple(position), target=tuple(position), hover_until=now+30.,
        segment=Segment((0.,0.,2.),tuple(position)), child=UUID(uuid=[7]*16),
        get_clock=lambda:SimpleNamespace(now=lambda:Time(seconds=1000)),
        planned=None, planning_ready=lambda:None)
    f.steps=[dict(type='TAKEOFF',height_m=2.),dict(type='NAVIGATE',target_map=[4.,2.,2.]),
             dict(type='HOVER',duration_s=30.),dict(type='RETURN'),dict(type='LAND')]
    def change(phase, reason=''):
        f.phase=phase;events.append((phase,reason));f.stable_since=None
    f.change=change
    f.command=lambda *args:events.append(('command',args))
    f.planning_goal_pub=SimpleNamespace(get_subscription_count=lambda:1,publish=starts.append)
    if backend=='EGO':
        f.planned=SimpleNamespace(gate=SimpleNamespace(inputs={
            'map':(SimpleNamespace(map_id='map',epoch=1,version=5),now),
            'alignment':(SimpleNamespace(alignment_id='alignment',generation=1),now)}),
            retire=lambda:events.append(('retire',None)),start=lambda *args:starts.append(args),
            admit=lambda *args:None,curve=None)
        f.accepted_plans=[]
    for name in ('service','begin_stop','next_step','complete_motion','save_return_checkpoint',
                 'checkpoint_alignment','checkpoint_map_context','restore_original_mission','planning_authorization'):
        setattr(f,name,MethodType(getattr(FlightServer,name),f))
    return f,position,events,starts


def request(f, key=3, finish=False):
    return RequestReturn.Request(mission_uuid=f.active_goal.goal_id,coordinator_instance=f.instance,
        request_id=UUID(uuid=[key]*16),finish_mission=finish)


def restore_request(f, key=5):
    return ResumeOriginalMission.Request(mission_uuid=f.active_goal.goal_id,coordinator_instance=f.instance,
        request_id=UUID(uuid=[key]*16))


def paused_return(f, finish=False):
    assert f.service('finish' if finish else 'return',request(f,finish=finish),RequestReturn.Response()).accepted
    # Confirm the same physical brake/hold boundary a live gateway must cross.
    f.return_after_brake=False
    f.next_step()
    assert f.phase in ('RETURN','PLAN_REQUEST')
    assert f.service('pause',request(f,4),RequestReturn.Response()).accepted
    f.phase='PAUSED';f.owner='HOLD_CONTROLLER';f.generation+=1
    f.pause_until=min(time.monotonic()+60.,f.deadline-8.)


def grant(f, index, key=6):
    req=AdvanceFlightStep.Request(mission_uuid=f.active_goal.goal_id,coordinator_instance=f.instance,
        control_session=ControlSession(session_id=f.session,generation=f.generation,owner=f.owner),
        request_id=UUID(uuid=[key]*16),step_index=index,step_type=f.steps[index]['type'])
    return FlightServer.advance_step(f,req,AdvanceFlightStep.Response())


@pytest.mark.parametrize('backend',['DIRECT','EGO'])
def test_restore_replans_from_current_pose_without_takeoff_or_new_root(backend):
    f,p,events,starts=gateway(backend=backend)
    original=deepcopy(f.steps);root=f.active_goal;deadline=f.deadline;home=f.home
    paused_return(f)
    old_child=bytes(f.child.uuid)
    generation=f.generation
    p[:]=[1.,0.,2.]
    response=f.service('restore',restore_request(f),ResumeOriginalMission.Response())
    assert response.accepted and response.reason=='ORIGINAL_MISSION_RESTORED'
    assert f.steps==original and f.phase=='AWAIT_STEP' and f.step_index==0
    assert f.active_goal is root and f.deadline==deadline and f.home==home
    assert f.plan_revision==2 and f.generation>generation and not f.return_override
    assert f.return_checkpoint is None and f.segment is None and not any(f.child.uuid)
    assert f.landing_result==('SUCCEEDED','LANDED_AND_DISARMED',True)
    assert grant(f,1).accepted
    assert bytes(f.child.uuid)!=old_child and any(f.child.uuid)
    if backend=='DIRECT':
        assert f.segment.start==tuple(p) and f.segment.end==(4.,2.,2.)
    else:
        assert f.phase=='PLAN_REQUEST' and starts[-2][1][3]==f.generation
        assert starts[-1].pose.position.x==4.
    assert not any(e[0]=='command' for e in events)
    before=(f.generation,f.plan_revision,len(events))
    assert f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted
    assert (f.generation,f.plan_revision,len(events))==before
    assert not f.service('resume',restore_request(f),ResumeOriginalMission.Response()).accepted


@pytest.mark.parametrize('kind',['NAVIGATE','HOVER'])
def test_ordinary_resume_after_return_pause_only_continues_return(kind):
    f,_,_,_=gateway(kind)
    paused_return(f)
    checkpoint=f.return_checkpoint
    assert f.service('resume',request(f,7),RequestReturn.Response()).accepted
    assert f.phase=='RETURN' and f.steps[f.step_index]['type']=='RETURN'
    assert f.return_override and f.return_checkpoint is checkpoint and f.plan_revision==1


@pytest.mark.parametrize('backend',['DIRECT','EGO'])
def test_hover_restoration_rejoins_anchor_then_consumes_only_remaining_hover(backend):
    f,p,_,_=gateway('HOVER',backend)
    anchor=f.target
    paused_return(f)
    remaining=f.return_checkpoint.hover_remaining
    assert 29.<remaining<=30.
    p[:]=[1.,0.,2.]
    assert f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted
    assert grant(f,2).accepted
    assert f.target==anchor and f.join_hover_remaining==remaining
    assert f.steps[f.step_index]['type']=='HOVER'
    assert f.phase==('PLAN_REQUEST' if backend=='EGO' else 'NAVIGATE')
    if backend=='EGO':
        FlightServer.planned_result(f,SimpleNamespace())
        assert f.phase=='NAVIGATE'
    # The hover timer begins after rejoining, not while travelling back.
    f.complete_motion(200.)
    assert f.phase=='HOVER' and f.hover_until==200.+remaining and f.reference==anchor
    assert f.join_hover_remaining is None


def test_pause_during_hover_rejoin_preserves_anchor_and_remaining_time():
    f,p,_,_=gateway('HOVER');anchor=f.target
    paused_return(f);p[:]=[1.,0.,2.]
    assert f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted
    assert grant(f,2).accepted
    remaining=f.join_hover_remaining
    assert f.service('pause',request(f,9),RequestReturn.Response()).accepted
    f.phase='PAUSED';f.owner='HOLD_CONTROLLER';f.pause_until=time.monotonic()+30.
    p[:]=[1.5,0.,2.]
    assert f.service('resume',request(f,10),RequestReturn.Response()).accepted
    assert f.target==anchor and f.join_hover_remaining==remaining and f.segment.start==tuple(p)


@pytest.mark.parametrize('fault', ['moving','cancel','land','terminal','clock','health','reset','expired',
    'deadline','owner','mode','disarm','runner','progress','session','home','alignment','root','coordinator','operator','writer'])
def test_restore_rejects_unsafe_or_changed_context_without_mutating_plan(fault):
    f,_,events,_=gateway();paused_return(f)
    req=restore_request(f)
    if fault=='moving':f.phase='RETURN'
    elif fault=='cancel':f.canceling=True
    elif fault=='land':f.land_committed=True
    elif fault=='terminal':f.result=('ABORTED','test',False)
    elif fault=='clock':f.clock_fault_latched=True
    elif fault=='health':f.healthy=lambda **_:False
    elif fault=='reset':f.resets=lambda:(1,)*6
    elif fault=='expired':f.pause_until=time.monotonic()-1.
    elif fault=='deadline':f.deadline=time.monotonic()+1.
    elif fault=='owner':f.owner='TASK'
    elif fault=='mode':f.samples['vehicle_status'].nav_state=2
    elif fault=='disarm':f.samples['vehicle_status'].arming_state=1
    elif fault=='runner':f.runner_lost=True
    elif fault=='progress':f.runner_last_tick-=1.
    elif fault=='session':f.session=UUID(uuid=[8]*16)
    elif fault=='home':f.home=(1.,0.,0.)
    elif fault=='alignment':f.alignment=Alignment((1.,0.,0.),0.)
    elif fault=='root':req.mission_uuid=UUID(uuid=[8]*16)
    elif fault=='coordinator':req.coordinator_instance='old'
    elif fault=='operator':
        f.operator_enabled=True;f.operator_arm_enabled=False
    elif fault=='writer':f.get_publishers_info_by_topic=lambda _:[object(),object()]
    before=(deepcopy(f.steps),f.generation,f.plan_revision,len(events))
    assert not f.service('restore',req,ResumeOriginalMission.Response()).accepted
    assert (f.steps,f.generation,f.plan_revision,len(events))==before


@pytest.mark.parametrize('fault',['map','epoch','alignment_generation','stale'])
def test_ego_restore_rejects_map_context_change_but_allows_new_map_versions(fault):
    f,_,_,_=gateway(backend='EGO');paused_return(f)
    if fault=='map':f.planned.gate.inputs['map'][0].map_id='other'
    elif fault=='epoch':f.planned.gate.inputs['map'][0].epoch+=1
    elif fault=='alignment_generation':f.planned.gate.inputs['alignment'][0].generation+=1
    else:
        def stale():raise ValueError('STALE_MAP_SOURCE')
        f.planning_ready=stale
    assert not f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted
    assert f.return_override


def test_map_version_update_is_rechecked_and_replanned():
    f,_,_,_=gateway(backend='EGO');paused_return(f)
    f.planned.gate.inputs['map'][0].version+=1
    assert f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted


@pytest.mark.parametrize('already_returning',[False,True])
def test_end_mission_permanently_discards_checkpoint(already_returning):
    f,_,_,_=gateway()
    if already_returning:
        paused_return(f)
        assert f.return_checkpoint is not None
        assert f.service('finish',request(f,8,True),RequestReturn.Response()).accepted
    else:paused_return(f,finish=True)
    assert f.return_checkpoint is None
    assert f.landing_result[1]=='MISSION_ENDED_LANDED_AND_DISARMED'
    assert not f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted
    assert f.service('resume',request(f,7),RequestReturn.Response()).accepted
    assert f.phase=='RETURN'


def test_await_step_checkpoint_skips_completed_work_and_repeated_cycles_keep_deadline():
    f,_,_,_=gateway();f.phase='AWAIT_STEP'
    deadline=f.deadline
    paused_return(f)
    assert f.return_checkpoint.next_index==2
    assert f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted
    assert grant(f,2).accepted and f.phase=='HOVER'
    # A second temporary return creates a fresh checkpoint for the same root.
    assert f.service('return',request(f,11),RequestReturn.Response()).accepted
    assert f.return_checkpoint.next_index==2 and f.plan_revision==3 and f.deadline==deadline


def test_natural_recipe_return_has_no_unfinished_work_to_restore():
    f,_,_,_=gateway();f.step_index=3;f.phase='RETURN'
    paused_return(f)
    assert f.return_checkpoint is None
    assert not f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted


def test_landing_commit_retires_temporary_checkpoint_and_cannot_restore():
    f,_,events,_=gateway();paused_return(f)
    assert f.return_checkpoint is not None
    assert f.service('resume',request(f,7),RequestReturn.Response()).accepted
    f.next_step()
    assert f.phase=='LAND_REQUEST' and f.land_committed and f.return_checkpoint is None
    assert events[-2:]==[('command',(21,)),('LAND_REQUEST','')]
    assert not f.service('restore',restore_request(f),ResumeOriginalMission.Response()).accepted
