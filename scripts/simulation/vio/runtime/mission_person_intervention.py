"""Pure same-root mission interlude protocol; contains no ROS or flight output.

A transport adapter must supply fresh executor capability, a trusted checkpoint,
and measured stop/hold proofs. A service ACK alone never advances a handoff.
"""
from collections import deque
from dataclasses import dataclass
import math
import uuid
from person_follow_exit_rules import FollowExitClock,FollowExitLimits


def valid_id(value):
    try:
        return isinstance(value,str) and uuid.UUID(value).int != 0
    except ValueError:
        return False


@dataclass(frozen=True)
class MissionCheckpoint:
    root: str
    coordinator: str
    control_session: str
    generation: int
    plan_revision: int
    step_index: int
    child: str
    context: tuple
    step_type: str
    target_map: tuple
    deadline: float
    hover_remaining: float | None = None

    def __post_init__(self):
        if (not all(valid_id(value) for value in (self.root,self.control_session,self.child))
                or not isinstance(self.coordinator,str) or not self.coordinator
                or any(type(v) is not int or v<0 for v in (self.plan_revision,self.step_index))
                or type(self.generation) is not int or self.generation<=0
                or not isinstance(self.context,tuple) or not self.context
                or self.step_type not in ('NAVIGATE','HOVER')
                or not isinstance(self.target_map,tuple) or len(self.target_map)!=3
                or not all(type(v) in (float,int) and math.isfinite(v) for v in self.target_map)
                or type(self.deadline) not in (float,int) or not math.isfinite(self.deadline)
                or self.hover_remaining is not None and (type(self.hover_remaining) not in (float,int)
                    or not math.isfinite(self.hover_remaining) or self.hover_remaining<0)):
            raise ValueError('INVALID_INTERLUDE_CHECKPOINT')

    @property
    def identity(self):
        return self.root,self.coordinator,self.control_session,self.plan_revision,self.context


@dataclass(frozen=True)
class ExecutorCapability:
    executor_instance: str
    root: str
    context: tuple
    received_mono: float
    supports_follow: bool
    dynamic_safety_ready: bool
    checkpoint_ready: bool


@dataclass(frozen=True)
class HandoffProof:
    root: str
    coordinator: str
    control_session: str
    plan_revision: int
    context: tuple
    generation: int
    received_mono: float
    stopped: bool
    hold_confirmed: bool
    owner: str = 'HOLD_CONTROLLER'
    operation_id: str = ''

    @property
    def identity(self):
        return self.root,self.coordinator,self.control_session,self.plan_revision,self.context


@dataclass(frozen=True)
class FollowStartProof:
    ownership: HandoffProof
    target_id: str


@dataclass(frozen=True)
class ResumeProof:
    handoff: HandoffProof
    step_index: int
    child: str
    step_type: str
    target_map: tuple
    deadline: float
    hover_remaining: float | None = None


@dataclass(frozen=True)
class ResumeRouteProof:
    operation_id: str
    identity: tuple
    received_mono: float
    step_index: int
    step_type: str
    target_map: tuple
    route_safe: bool
    reachable: bool
    replanned_from_current: bool


@dataclass(frozen=True)
class Operation:
    request_id: str
    interlude_id: str
    command: str
    root: str
    coordinator: str
    control_session: str
    generation: int
    target_id: str | None
    deadline: float
    issued_mono: float


class MissionInterlude:
    """Operation proposals only; acknowledgements never replace measured handoff proofs."""
    ACTIVE=('WAIT_PAUSE','WAIT_FOLLOW_START','FOLLOWING','STATIC_OBSERVE','LOST_WAIT',
            'MULTI_OBSERVE','WAIT_STOP','WAIT_RESUME_ROUTE','WAIT_RESUME')

    def __init__(self,moving_s=10.,stationary_s=5.,lost_s=5.,multiple_s=10.,handoff_s=5.,fresh_s=.5):
        self.limits=FollowExitLimits(moving_s,stationary_s,lost_s,multiple_s,fresh_s)
        if type(handoff_s) not in (int,float) or not math.isfinite(handoff_s) or handoff_s<=0:
            raise ValueError('INVALID_INTERLUDE_LIMITS')
        self.handoff_s,self.fresh_s=handoff_s,fresh_s
        self.clock=FollowExitClock(self.limits)
        self.enabled=False
        self.phase,self.reason='DISABLED','MODE_OFF'
        self.checkpoint=self.operation=None
        self.executor_instance=self.interlude_id=self.target_id=None
        self.generation=0
        self.ack=False
        self.proof=None
        self.target_at=self.last_source=None
        self.last_motion='MOVING'
        self.after_stop='RESUME'
        self.resume_allowed=False
        self.events=deque(maxlen=256)
        self._last_now=None

    def _time(self,now):
        if type(now) not in (float,int) or not math.isfinite(now):
            raise ValueError('INVALID_MONOTONIC_TIME')
        if self._last_now is not None and now<self._last_now:
            self.operation=None
            self.phase,self.reason='FAULT','MONOTONIC_TIME_REVERSED'
            return False
        self._last_now=now
        return True

    def _state(self,phase,reason,now):
        self.phase,self.reason=phase,reason
        self.events.append(dict(phase=phase,reason=reason,mono=now))

    def set_mode(self,value,now):
        if type(value) is not bool:
            raise ValueError('INVALID_MODE')
        if not self._time(now) or self.enabled==value:
            return
        self.enabled=value
        if not value and self.phase in ('FOLLOWING','WAIT_FOLLOW_START'):
            self._stop('MODE_DISABLED',now)
        elif not value and self.phase in ('STATIC_OBSERVE','LOST_WAIT','MULTI_OBSERVE'):
            self._request('CHECK_RESUME_ROUTE','WAIT_RESUME_ROUTE',now)
        elif not value and self.phase=='WAIT_PAUSE':
            self.reason='MODE_DISABLED_WAITING_PAUSE_HANDOFF'
        elif not value and self.phase=='WAIT_STOP' and self.after_stop!='FAULT':
            self.after_stop='RESUME'
        elif self.phase in ('DISABLED','MONITORING','BLOCKED','COMPLETE'):
            self._state('MONITORING' if value else 'DISABLED','WAITING_TARGET' if value else 'MODE_OFF',now)

    def blockers(self,capability,now,checkpoint=None):
        checkpoint=checkpoint or self.checkpoint
        if capability is None:
            return ['FOLLOW_EXECUTOR_UNAVAILABLE','DYNAMIC_SAFETY_UNQUALIFIED','CHECKPOINT_PROVIDER_UNAVAILABLE']
        if not isinstance(capability,ExecutorCapability):
            return ['INVALID_EXECUTOR_CAPABILITY']
        result=[]
        if not valid_id(capability.executor_instance):
            result.append('INVALID_EXECUTOR_IDENTITY')
        if (type(capability.received_mono) not in (int,float) or not math.isfinite(capability.received_mono)
                or not 0<=now-capability.received_mono<=self.fresh_s):
            result.append('STALE_EXECUTOR_CAPABILITY')
        for field,reason in (('supports_follow','FOLLOW_EXECUTOR_UNAVAILABLE'),
                             ('dynamic_safety_ready','DYNAMIC_SAFETY_UNQUALIFIED'),
                             ('checkpoint_ready','CHECKPOINT_PROVIDER_UNAVAILABLE')):
            if getattr(capability,field) is not True:
                result.append(reason)
        if checkpoint is None or (capability.root,capability.context)!=(checkpoint.root,checkpoint.context):
            result.append('EXECUTOR_BINDING_MISMATCH')
        return result

    def _capable(self,capability,now):
        return not self.blockers(capability,now) and capability.executor_instance==self.executor_instance

    def inspect(self,capability,now):
        if self._time(now) and self.enabled and self.phase in ('MONITORING','BLOCKED'):
            missing=self.blockers(capability,now)
            value=('BLOCKED',';'.join(missing)) if missing else ('MONITORING','WAITING_TRUSTED_CHECKPOINT_AND_TARGET')
            if (self.phase,self.reason)!=value:
                self._state(*value,now)
        return self.status()

    def begin(self,checkpoint,target_id,capability,now,*,multiple=False):
        if not self._time(now):
            return None
        if not self.enabled or self.phase not in ('MONITORING','BLOCKED','COMPLETE'):
            self.reason='MODE_OFF_OR_INTERLUDE_BUSY'
            return None
        if (not isinstance(checkpoint,MissionCheckpoint) or type(multiple) is not bool
                or not multiple and (not isinstance(target_id,str) or not target_id)):
            self._state('BLOCKED','INVALID_CHECKPOINT_OR_TARGET',now)
            return None
        missing=self.blockers(capability,now,checkpoint)
        if missing:
            self._state('BLOCKED',';'.join(missing),now)
            return None
        required=max(self.limits.moving_s+self.limits.stationary_s,self.limits.multiple_s,self.limits.lost_s)+5*self.handoff_s
        if checkpoint.deadline-now<required:
            self._state('BLOCKED','INSUFFICIENT_ORIGINAL_MISSION_BUDGET',now)
            return None
        self.checkpoint=checkpoint
        self.target_id,self.executor_instance=None if multiple else target_id,capability.executor_instance
        self.interlude_id=str(uuid.uuid4())
        self.generation=checkpoint.generation
        self.clock=FollowExitClock(self.limits)
        self.target_at,self.last_source,self.last_motion=now,None,'MOVING'
        self.after_stop,self.resume_allowed='MULTI' if multiple else 'RESUME',True
        return self._request('PAUSE','WAIT_PAUSE',now)

    def _request(self,command,phase,now):
        self.ack,self.proof=False,None
        self.operation=Operation(str(uuid.uuid4()),self.interlude_id,command,
            self.checkpoint.root,self.checkpoint.coordinator,self.checkpoint.control_session,
            self.generation,self.target_id,min(now+self.handoff_s,self.checkpoint.deadline),now)
        self._state(phase,command+'_REQUESTED',now)
        return self.operation

    def response(self,request_id,accepted,now,capability=None):
        if not self._time(now) or self.operation is None or request_id!=self.operation.request_id:
            return None
        if self._expire(now):
            return self.operation
        if type(accepted) is not bool:
            raise ValueError('INVALID_ACK')
        if self.ack:
            return None
        if not accepted:
            phase='BLOCKED' if self.phase=='WAIT_PAUSE' else 'HOLDING' if self.phase=='WAIT_RESUME_ROUTE' else 'FAULT'
            self.operation=None
            self._state(phase,'EXECUTOR_REJECTED_OPERATION',now)
            return None
        self.ack=True
        if self.phase=='WAIT_FOLLOW_START':
            return self._advance_start(now,capability)
        if self.phase=='WAIT_RESUME_ROUTE':
            return self._advance_route(now)
        return self._advance_handoff(now,capability)

    def _proof_valid(self,proof,now,owner):
        return (isinstance(proof,HandoffProof) and proof.identity==self.checkpoint.identity
            and proof.owner==owner and proof.operation_id==self.operation.request_id
            and type(proof.generation) is int and proof.generation>self.generation
            and type(proof.received_mono) in (int,float) and math.isfinite(proof.received_mono)
            and self.operation.issued_mono<=proof.received_mono<=now
            and now-proof.received_mono<=self.fresh_s)

    def handoff(self,proof,now,capability=None):
        if not self._time(now) or self.operation is None or self.phase not in ('WAIT_PAUSE','WAIT_STOP'):
            return None
        if self._expire(now):
            return self.operation
        if not self._proof_valid(proof,now,'HOLD_CONTROLLER') or proof.stopped is not True or proof.hold_confirmed is not True:
            return None
        self.proof=proof
        return self._advance_handoff(now,capability)

    def _advance_handoff(self,now,capability):
        if not self.ack or self.proof is None or not self._proof_valid(self.proof,now,'HOLD_CONTROLLER'):
            return None
        self.generation=self.proof.generation
        if self.phase=='WAIT_PAUSE':
            if not self.enabled or not self._capable(capability,now):
                return self._request('CHECK_RESUME_ROUTE','WAIT_RESUME_ROUTE',now)
            if self.after_stop in ('MULTI','LOST'):
                return self._held(self.after_stop,now,capability)
            if now-self.target_at>self.fresh_s:
                return self._held('LOST',now,capability)
            if self.last_motion=='STATIONARY':
                return self._held('STATIC',now,capability)
            return self._request('START_FOLLOW','WAIT_FOLLOW_START',now)
        return self._held(self.after_stop,now,capability)

    def _held(self,action,now,capability):
        self.operation=None
        if action=='FAULT':
            self._state('HOLDING','FOLLOW_STOPPED_OPERATOR_REVIEW_REQUIRED',now)
        elif not self.enabled or action=='RESUME':
            return self._request('CHECK_RESUME_ROUTE','WAIT_RESUME_ROUTE',now)
        elif action=='MULTI':
            self.clock.multiple(now)
            self._state('MULTI_OBSERVE','MULTIPLE_PEOPLE_LOCKED',now)
        elif action=='LOST' or self.target_at is None or now-self.target_at>self.fresh_s:
            self.clock.lost(now)
            self._state('LOST_WAIT','RECONFIRM_ORIGINAL_TARGET_WITHIN_5S',now)
        elif self.last_motion=='MOVING':
            if self._capable(capability,now):
                return self._request('START_FOLLOW','WAIT_FOLLOW_START',now)
            self._state('HOLDING','FOLLOW_CAPABILITY_LOST',now)
        else:
            self.clock.pause_motion()
            if self.last_source is not None:
                self.clock.sample(self.last_source,now,'STATIONARY')
            self._state('STATIC_OBSERVE','PERSON_STATIC_OBSERVING',now)
        return None

    def started(self,proof,now,capability=None):
        if not self._time(now) or self.phase!='WAIT_FOLLOW_START' or self.operation is None:
            return None
        if self._expire(now):
            return self.operation
        if not isinstance(proof,FollowStartProof) or not self._proof_valid(proof.ownership,now,'TASK') or proof.target_id!=self.target_id:
            return None
        self.proof=proof
        return self._advance_start(now,capability)

    def _advance_start(self,now,capability):
        if not self.ack or not isinstance(self.proof,FollowStartProof):
            return None
        if not self._proof_valid(self.proof.ownership,now,'TASK') or not self._capable(capability,now):
            return self._stop('FOLLOW_ADMISSION_LOST',now,action='FAULT')
        self.generation=self.proof.ownership.generation
        if self.target_at is None or now-self.target_at>self.fresh_s:
            return self._stop('TARGET_LOST_DURING_START',now,action='LOST')
        self.operation=None
        self.target_at=now
        self.clock.pause_motion()
        self._state('FOLLOWING','FOLLOW_OWNERSHIP_CONFIRMED',now)
        return None

    def target(self,source_stamp,moving,received_mono,now,*,target_id=None,confirmed=True,capability=None):
        if not self._time(now) or self.phase not in self.ACTIVE or self.phase in ('WAIT_RESUME_ROUTE','WAIT_RESUME','MULTI_OBSERVE'):
            return None
        if (not all(type(v) in (int,float) and math.isfinite(v) for v in (source_stamp,received_mono))
                or moving not in (True,False) or type(moving) is not bool or type(confirmed) is not bool
                or not 0<=now-received_mono<=self.fresh_s):
            return self.target_lost(now)
        if target_id is not None and target_id!=self.target_id or not confirmed:
            return self.target_lost(now)
        if self.last_source is not None and source_stamp<=self.last_source:
            if source_stamp<self.last_source:
                return self._stop('TARGET_TIME_REVERSED',now,action='FAULT')
            return None
        if self.phase=='LOST_WAIT' and self.clock.reason(now):
            return self._request('CHECK_RESUME_ROUTE','WAIT_RESUME_ROUTE',now)
        if (self.phase in ('FOLLOWING','STATIC_OBSERVE') and self.target_at is not None
                and (now-self.target_at>self.fresh_s or self.last_source is not None
                     and source_stamp-self.last_source>self.fresh_s)):
            return self.target_lost(now)
        self.last_source,self.target_at,self.last_motion=source_stamp,received_mono,'MOVING' if moving else 'STATIONARY'
        if self.phase=='LOST_WAIT':
            if not self._capable(capability,now) or not self.clock.reacquire(now):
                return None
            if moving:
                return self._request('START_FOLLOW','WAIT_FOLLOW_START',now)
            self.clock.sample(source_stamp,received_mono,'STATIONARY')
            self._state('STATIC_OBSERVE','TARGET_RECONFIRMED_STATIONARY',now)
        elif self.phase=='STATIC_OBSERVE':
            if moving:
                self.clock.pause_motion()
                if self._capable(capability,now):
                    return self._request('START_FOLLOW','WAIT_FOLLOW_START',now)
                self._state('HOLDING','FOLLOW_CAPABILITY_LOST',now)
            else:
                self.clock.sample(source_stamp,received_mono,'STATIONARY')
        elif self.phase=='FOLLOWING':
            if moving:
                self.clock.sample(source_stamp,received_mono,'MOVING')
            else:
                return self._stop('PERSON_STATIC_STOP_BEFORE_OBSERVING',now,action='STATIC')
        elif self.phase=='WAIT_FOLLOW_START' and not moving:
            return self._stop('PERSON_STATIC_DURING_START',now,action='STATIC')
        return self.operation

    def target_lost(self,now):
        if not self._time(now):
            return None
        if self.phase in ('FOLLOWING','WAIT_FOLLOW_START'):
            return self._stop('TARGET_LOST_STOP_FIRST',now,action='LOST')
        if self.phase=='STATIC_OBSERVE':
            self.clock.lost(now)
            self._state('LOST_WAIT','TARGET_LOST_DURING_STATIC_OBSERVATION',now)
        elif self.phase=='WAIT_STOP' and self.after_stop=='STATIC':
            self.after_stop='LOST'
        elif self.phase=='WAIT_PAUSE' and self.after_stop!='MULTI':
            self.after_stop='LOST'
        return self.operation

    def multiple(self,now):
        if not self._time(now) or not self.enabled:
            return None
        if self.phase in ('FOLLOWING','WAIT_FOLLOW_START'):
            return self._stop('MULTIPLE_PEOPLE_STOP_FIRST',now,action='MULTI')
        if self.phase in ('STATIC_OBSERVE','LOST_WAIT'):
            self.clock.multiple(now)
            self._state('MULTI_OBSERVE','MULTIPLE_PEOPLE_LOCKED',now)
        elif self.phase in ('WAIT_STOP','WAIT_PAUSE') and self.after_stop!='FAULT':
            self.after_stop='MULTI'
        return self.operation

    def _stop(self,reason,now,*,action='RESUME',resume=None):
        if resume is False:
            action='FAULT'
        self.after_stop=action
        self.resume_allowed=action!='FAULT'
        self.clock.pause_motion()
        request=self._request('STOP_FOLLOW','WAIT_STOP',now)
        self.reason=reason
        return request

    def route_checked(self,proof,now):
        if not self._time(now) or self.phase!='WAIT_RESUME_ROUTE' or self.operation is None:
            return None
        if self._expire(now):
            return None
        if (not isinstance(proof,ResumeRouteProof) or proof.operation_id!=self.operation.request_id
                or proof.identity!=self.checkpoint.identity or type(proof.received_mono) not in (int,float)
                or not self.operation.issued_mono<=proof.received_mono<=now or now-proof.received_mono>self.fresh_s
                or proof.step_index!=self.checkpoint.step_index or proof.step_type!=self.checkpoint.step_type
                or proof.target_map!=self.checkpoint.target_map):
            return None
        if not (proof.route_safe is True and proof.reachable is True and proof.replanned_from_current is True):
            self.operation=None
            self._state('HOLDING','ORIGINAL_TARGET_ROUTE_NOT_ADMITTED',now)
            return None
        self.proof=proof
        return self._advance_route(now)

    def _advance_route(self,now):
        if not self.ack or not isinstance(self.proof,ResumeRouteProof):
            return None
        if now-self.proof.received_mono>self.fresh_s:
            return None
        return self._request('RESUME','WAIT_RESUME',now)

    def tick(self,now,capability=None,mission_identity=None):
        if not self._time(now) or self.phase not in self.ACTIVE:
            return self.operation
        if mission_identity is None or tuple(mission_identity)!=self.checkpoint.identity or now>=self.checkpoint.deadline:
            self.operation=None
            self._state('FAULT','ORIGINAL_MISSION_OR_CONTEXT_UNAVAILABLE',now)
            return None
        if self._expire(now):
            return self.operation
        if self.phase in ('FOLLOWING','STATIC_OBSERVE','LOST_WAIT','MULTI_OBSERVE'):
            if not self._capable(capability,now):
                return self._stop('FOLLOW_CAPABILITY_LOST',now,action='FAULT')
            if self.phase in ('FOLLOWING','STATIC_OBSERVE') and now-self.target_at>self.fresh_s:
                return self.target_lost(now)
            reason=self.clock.reason(now)
            if reason:
                if self.phase=='FOLLOWING':
                    return self._stop(reason,now)
                return self._request('CHECK_RESUME_ROUTE','WAIT_RESUME_ROUTE',now)
        return self.operation

    def _expire(self,now):
        if self.operation is None or now<self.operation.deadline:
            return False
        if self.phase=='WAIT_FOLLOW_START':
            self._stop('FOLLOW_START_UNCONFIRMED',now,action='FAULT')
        else:
            phase='HOLDING' if self.phase=='WAIT_RESUME_ROUTE' else 'FAULT'
            self.operation=None
            self._state(phase,'OPERATION_UNCONFIRMED',now)
        return True

    def resumed(self,proof,now):
        if not self._time(now) or not self.ack or self.phase!='WAIT_RESUME' or self.operation is None:
            return False
        if self._expire(now) or not isinstance(proof,ResumeProof) or not self._proof_valid(proof.handoff,now,'TASK'):
            return False
        valid=(type(proof.step_index) is int and proof.step_index==self.checkpoint.step_index
            and valid_id(proof.child) and proof.child!=self.checkpoint.child and proof.step_type==self.checkpoint.step_type
            and proof.target_map==self.checkpoint.target_map and proof.deadline==self.checkpoint.deadline)
        if self.checkpoint.hover_remaining is not None:
            valid=valid and type(proof.hover_remaining) in (int,float) and 0<=proof.hover_remaining<=self.checkpoint.hover_remaining
        if not valid:
            return False
        self.generation=proof.handoff.generation
        self.operation=None
        self._state('COMPLETE','ORIGINAL_UNFINISHED_STEP_REPLANNED_AND_RESUMED',now)
        return True

    def status(self):
        now=self._last_now if self._last_now is not None else 0.
        return dict(phase=self.phase,reason=self.reason,enabled=self.enabled,interlude_id=self.interlude_id,
            operation=self.operation.command if self.operation else None,checkpoint_bound=self.checkpoint is not None,
            resume_proof_validated=self.phase=='COMPLETE',timers=self.clock.status(now))
