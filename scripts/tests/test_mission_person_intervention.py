"""Contract tests use explicit synthetic proofs, never physical stop/flight evidence."""
from dataclasses import replace
from pathlib import Path
import sys
import uuid

import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation/vio/runtime'))
from mission_person_intervention import (ExecutorCapability,FollowStartProof,HandoffProof,
    MissionCheckpoint,MissionInterlude,ResumeProof,ResumeRouteProof)

CONTEXT=('map',1,'loc','alignment',1,'context')


def identity():
    return str(uuid.uuid4())


@pytest.fixture
def model():
    cp=MissionCheckpoint(identity(),'coordinator',identity(),1,2,3,identity(),CONTEXT,'NAVIGATE',(5.,2.,1.),100.,4.)
    capability=ExecutorCapability(identity(),cp.root,cp.context,0.,True,True,True)
    state=MissionInterlude()
    state.set_mode(True,0.)
    return state,cp,capability


def cap(value,now):
    return replace(value,received_mono=now)


def proof(cp,operation,generation,now,owner='HOLD_CONTROLLER'):
    return HandoffProof(cp.root,cp.coordinator,cp.control_session,cp.plan_revision,
        cp.context,generation,now,True,True,owner,operation.request_id)


def enter_following(model):
    state,cp,c=model
    pause=state.begin(cp,'person',cap(c,0.),0.)
    assert pause.command=='PAUSE'
    state.response(pause.request_id,True,.1)
    assert state.phase=='WAIT_PAUSE'
    start=state.handoff(proof(cp,pause,2,.2),.2,cap(c,.2))
    assert start.command=='START_FOLLOW'
    state.response(start.request_id,True,.3,cap(c,.3))
    assert state.phase=='WAIT_FOLLOW_START'  # ACK alone is not an ownership proof.
    state.started(FollowStartProof(proof(cp,start,3,.4,'TASK'),'person'),.4,cap(c,.4))
    assert state.phase=='FOLLOWING' and state.generation==3
    return state,cp,c


def finish_stop(state,cp,c,now):
    stop=state.operation
    assert stop.command=='STOP_FOLLOW'
    state.response(stop.request_id,True,now)
    assert state.phase=='WAIT_STOP'
    return state.handoff(proof(cp,stop,state.generation+1,now+.1),now+.1,cap(c,now+.1))


def test_missing_executor_does_not_pause_original_task(model):
    state,cp,_=model
    assert state.begin(cp,'person',None,0.) is None
    assert state.phase=='BLOCKED' and state.operation is None
    assert 'DYNAMIC_SAFETY_UNQUALIFIED' in state.reason


@pytest.mark.parametrize('field',['supports_follow','dynamic_safety_ready','checkpoint_ready'])
def test_any_missing_capability_prevents_pause(model,field):
    state,cp,c=model
    assert state.begin(cp,'person',replace(c,**{field:False}),0.) is None
    assert state.operation is None


def route_admitted(state,cp,now,**changes):
    query=state.operation
    assert query.command=='CHECK_RESUME_ROUTE'
    state.response(query.request_id,True,now)
    route=ResumeRouteProof(query.request_id,cp.identity,now,cp.step_index,
        cp.step_type,cp.target_map,True,True,True)
    return state.route_checked(replace(route,**changes),now)


def test_static_stop_five_second_hold_then_replan_same_unfinished_step(model):
    state,cp,c=enter_following(model)
    state.target(.5,False,.5,.5)
    assert state.operation.command=='STOP_FOLLOW'
    finish_stop(state,cp,c,.6)
    assert state.phase=='STATIC_OBSERVE'
    for i in range(1,51):
        now=.7+i*.1
        state.target(now,False,now,now)
        state.tick(now,cap(c,now),cp.identity)
    assert state.phase=='WAIT_RESUME_ROUTE'
    resume=route_admitted(state,cp,5.8)
    assert resume.command=='RESUME' and resume.root==cp.root and resume.generation==4
    state.response(resume.request_id,True,5.9)
    resumed=ResumeProof(proof(cp,resume,5,6.,'TASK'),cp.step_index,identity(),cp.step_type,cp.target_map,cp.deadline,3.)
    assert state.resumed(resumed,6.) and state.phase=='COMPLETE'
    assert cp.generation==1 and cp.step_index==3 and cp.hover_remaining==4.


def test_unconfirmed_or_wrong_pause_proof_never_starts_follow(model):
    state,cp,c=model
    pause=state.begin(cp,'person',c,0.)
    state.response(pause.request_id,True,.1)
    valid=proof(cp,pause,2,.2)
    for invalid in (replace(valid,stopped=False),replace(valid,hold_confirmed=False),
                    replace(valid,root=identity()),replace(valid,operation_id=identity()),
                    replace(valid,generation=1),replace(valid,received_mono=-1.)):
        assert state.handoff(invalid,.2,cap(c,.2)) is None
        assert state.phase=='WAIT_PAUSE'


def test_proof_before_ack_and_old_response_replay(model):
    state,cp,c=model
    pause=state.begin(cp,'person',c,0.)
    assert state.handoff(proof(cp,pause,2,.1),.1,cap(c,.1)) is None
    assert state.phase=='WAIT_PAUSE'
    start=state.response(pause.request_id,True,.2,cap(c,.2))
    assert start.command=='START_FOLLOW'
    assert state.response(pause.request_id,True,.3) is None
    assert state.operation==start


def test_lost_capability_after_pause_resumes_without_starting_follow(model):
    state,cp,c=model
    pause=state.begin(cp,'person',c,0.)
    state.response(pause.request_id,True,.1)
    request=state.handoff(proof(cp,pause,2,1.),1.,c)  # Capability expired during pause.
    assert request.command=='CHECK_RESUME_ROUTE'
    assert not any(event['phase']=='FOLLOWING' for event in state.events)


def test_disable_during_pause_does_not_start_follow(model):
    state,cp,c=model
    pause=state.begin(cp,'person',c,0.)
    state.set_mode(False,.1)
    state.response(pause.request_id,True,.2)
    assert state.handoff(proof(cp,pause,2,.3),.3,cap(c,.3)).command=='CHECK_RESUME_ROUTE'


def test_target_loss_stops_then_waits_at_most_five_seconds(model):
    state,cp,c=enter_following(model)
    state.tick(1.,cap(c,1.),cp.identity)
    assert state.operation.command=='STOP_FOLLOW'
    assert finish_stop(state,cp,c,1.1) is None
    assert state.phase=='LOST_WAIT'
    state.target_lost(4.)
    state.tick(6.19,cap(c,6.19),cp.identity)
    assert state.phase=='LOST_WAIT'
    state.tick(6.2,cap(c,6.2),cp.identity)
    assert state.operation.command=='CHECK_RESUME_ROUTE'


def test_target_replay_cannot_extend_static_evidence(model):
    state,cp,c=enter_following(model)
    state.target(10.,False,.5,.5)
    finish_stop(state,cp,c,.6)
    state.target(10.,False,.9,.9)
    state.tick(1.1,cap(c,1.1),cp.identity)
    assert state.phase=='LOST_WAIT'


def test_new_mission_or_missing_identity_never_resumes_old_work(model):
    state,cp,c=enter_following(model)
    state.tick(.5,cap(c,.5),(identity(),*cp.identity[1:]))
    assert state.phase=='FAULT' and state.operation is None


def test_cumulative_moving_time_survives_static_pause_and_restart(model):
    state,cp,c=enter_following(model)
    for i in range(41):
        now=.5+i*.1
        state.target(now,True,now,now)
        state.tick(now,cap(c,now),cp.identity)
    assert state.clock.moving_seconds==pytest.approx(4.)
    state.target(4.6,False,4.6,4.6)
    finish_stop(state,cp,c,4.7)
    for i in range(1,11):
        now=4.8+i*.1
        state.target(now,False,now,now)
        state.tick(now,cap(c,now),cp.identity)
    start=state.target(5.9,True,5.9,5.9,capability=cap(c,5.9))
    state.response(start.request_id,True,6.)
    state.started(FollowStartProof(proof(cp,start,5,6.1,'TASK'),'person'),6.1,cap(c,6.1))
    for i in range(61):
        now=6.2+i*.1
        state.target(now,True,now,now)
        state.tick(now,cap(c,now),cp.identity)
    assert state.phase=='WAIT_STOP' and state.reason=='MOVING_TIME_BUDGET_REACHED'
    assert state.clock.moving_seconds==pytest.approx(10.) and cp.deadline==100.


@pytest.mark.parametrize('change',[dict(child=None),dict(step_index=9),dict(deadline=110.),dict(hover_remaining=5.),dict(target_map=(9.,9.,9.))])
def test_resume_requires_new_child_and_unchanged_checkpoint(model,change):
    state,cp,c=enter_following(model)
    state.set_mode(False,.5)
    finish_stop(state,cp,c,.6)
    resume=route_admitted(state,cp,.75)
    state.response(resume.request_id,True,.8)
    resumed=ResumeProof(proof(cp,resume,5,.9,'TASK'),cp.step_index,identity(),cp.step_type,cp.target_map,cp.deadline,3.)
    if change.get('child') is None and 'child' in change:
        change=dict(child=cp.child)
    assert not state.resumed(replace(resumed,**change),.9)
    assert state.phase=='WAIT_RESUME'


def test_late_ack_does_not_bypass_handoff_deadline(model):
    state,cp,c=model
    pause=state.begin(cp,'person',c,0.)
    state.response(pause.request_id,True,6.)
    assert state.phase=='FAULT' and state.operation is None


def test_follow_start_ack_without_ownership_event_times_out_to_scoped_stop(model):
    state,cp,c=model
    pause=state.begin(cp,'person',c,0.)
    state.response(pause.request_id,True,.1)
    start=state.handoff(proof(cp,pause,2,.2),.2,cap(c,.2))
    state.response(start.request_id,True,.3)
    stop=state.tick(5.3,cap(c,5.3),cp.identity)
    assert stop.command=='STOP_FOLLOW' and not state.resume_allowed


def test_insufficient_original_budget_cannot_pause(model):
    state,cp,c=model
    cp=replace(cp,deadline=20.)
    assert state.begin(cp,'person',c,0.) is None
    assert state.reason=='INSUFFICIENT_ORIGINAL_MISSION_BUDGET'


def test_static_wait_needs_capture_progress_after_stop(model):
    state,cp,c=enter_following(model)
    state.target(10.,False,.5,.5)
    finish_stop(state,cp,c,.6)
    for index in range(1,65):
        now=.7+index*.1
        state.target(10.+index*.001,False,now,now)
        state.tick(now,cap(c,now),cp.identity)
    assert state.phase=='STATIC_OBSERVE'


def test_follow_start_proof_before_ack_is_order_independent(model):
    state,cp,c=model
    pause=state.begin(cp,'person',c,0.)
    state.response(pause.request_id,True,.1)
    start=state.handoff(proof(cp,pause,2,.2),.2,cap(c,.2))
    state.started(FollowStartProof(proof(cp,start,3,.3,'TASK'),'person'),.3,cap(c,.3))
    assert state.phase=='WAIT_FOLLOW_START'
    state.response(start.request_id,True,.4,cap(c,.4))
    assert state.phase=='FOLLOWING' and state.generation==3


def test_readiness_inspection_proposes_no_operations(model):
    state,cp,c=model
    status=state.inspect(None,.1)
    assert status['phase']=='BLOCKED'
    assert state.operation is None and state.checkpoint is None


@pytest.mark.parametrize('kind',['TAKEOFF','OBSERVE','RETURN','LAND','PLAN_REQUEST'])
def test_checkpoint_requires_an_explicit_interruptible_original_step(model,kind):
    _,cp,_=model
    with pytest.raises(ValueError):
        replace(cp,step_type=kind)


def test_reacquire_same_target_preserves_total_and_rejects_wrong_id(model):
    state,cp,c=enter_following(model)
    for i in range(11):
        now=.5+i*.1
        state.target(now,True,now,now)
    before=state.clock.moving_seconds
    state.target_lost(1.6)
    finish_stop(state,cp,c,1.7)
    assert state.phase=='LOST_WAIT'
    assert state.target(2.,True,2.,2.,target_id='other',capability=cap(c,2.)) is None
    assert state.phase=='LOST_WAIT'
    start=state.target(2.1,True,2.1,2.1,target_id='person',capability=cap(c,2.1))
    assert start.command=='START_FOLLOW'
    assert state.clock.moving_seconds==before


def test_multi_locks_for_ten_seconds_after_stop_despite_single_target(model):
    state,cp,c=enter_following(model)
    state.multiple(.5)
    finish_stop(state,cp,c,.6)
    assert state.phase=='MULTI_OBSERVE'
    state.multiple(5.)
    state.target(6.,True,6.,6.,capability=cap(c,6.))
    state.tick(10.69,cap(c,10.69),cp.identity)
    assert state.phase=='MULTI_OBSERVE'
    state.tick(10.7,cap(c,10.7),cp.identity)
    assert state.operation.command=='CHECK_RESUME_ROUTE'


def test_initial_multi_requires_pause_proof_and_never_starts_follow(model):
    state,cp,c=model
    pause=state.begin(cp,None,c,0.,multiple=True)
    state.response(pause.request_id,True,.1)
    state.handoff(proof(cp,pause,2,.2),.2,cap(c,.2))
    assert state.phase=='MULTI_OBSERVE' and state.target_id is None


@pytest.mark.parametrize('change',[dict(route_safe=False),dict(reachable=False),dict(replanned_from_current=False)])
def test_unsafe_or_unreplanned_route_cannot_resume(model,change):
    state,cp,c=enter_following(model)
    state.set_mode(False,.5)
    finish_stop(state,cp,c,.6)
    assert route_admitted(state,cp,.8,**change) is None
    assert state.phase=='HOLDING' and state.operation is None


def test_route_proof_cannot_target_another_step_or_position(model):
    state,cp,c=enter_following(model)
    state.set_mode(False,.5)
    finish_stop(state,cp,c,.6)
    assert route_admitted(state,cp,.8,target_map=(1.,1.,1.)) is None
    assert state.phase=='WAIT_RESUME_ROUTE'


def test_route_timeout_holds_without_replaying_old_trajectory(model):
    state,cp,c=enter_following(model)
    state.set_mode(False,.5)
    finish_stop(state,cp,c,.6)
    state.tick(5.8,cap(c,5.8),cp.identity)
    assert state.phase=='HOLDING' and state.operation is None


def test_late_start_proof_cannot_refresh_missing_target_evidence(model):
    state,cp,c=model
    pause=state.begin(cp,'person',c,0.)
    state.response(pause.request_id,True,.1)
    start=state.handoff(proof(cp,pause,2,.2),.2,cap(c,.2))
    state.response(start.request_id,True,.3)
    stop=state.started(FollowStartProof(proof(cp,start,3,1.,'TASK'),'person'),1.,cap(c,1.))
    assert stop.command=='STOP_FOLLOW' and state.after_stop=='LOST'
    assert state.reason=='TARGET_LOST_DURING_START'
