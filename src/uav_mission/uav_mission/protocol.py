"""Single-owner S1 protocol model. No hardware I/O or inferred flight facts.

All times are monotonic seconds supplied by the caller. The owner must serialize
calls; ROS mock does so with one lock. Evidence is identity/generation scoped.
"""
from dataclasses import dataclass
import hashlib
import math
import uuid


def new_id():
    return str(uuid.uuid4())


@dataclass(frozen=True)
class Context:
    mission: str
    instance: str
    session: str
    generation: int
    child: str = ''


@dataclass(frozen=True)
class Decision:
    accepted: bool
    reason: str
    phase: str


class Protocol:
    TERMINAL = ('SUCCEEDED', 'CANCELED', 'ABORTED')

    def __init__(self, lease_s=.5, cleanup_s=1., final_hold_s=30., pause_s=60.):
        if not all(math.isfinite(x) and x > 0 for x in
                   (lease_s, cleanup_s, final_hold_s, pause_s)):
            raise ValueError('Invalid protocol limits')
        self.instance = new_id()
        self.lease_s, self.cleanup_s = lease_s, cleanup_s
        self.final_hold_s, self.pause_s = final_hold_s, pause_s
        self.mission = self.session = self.child = ''
        self.phase = 'IDLE'
        self.owner = 'NONE'
        self.generation = self.sequence = 0
        self.deadline = self.lease_deadline = self.hold_deadline = 0.
        self.stop_deadline = self.pause_deadline = 0.
        self.reason = self.result_code = ''
        self.cleanup_confirmed = False
        self.requests = {}
        self.seen_missions = set()
        self.last_time = None
        self.map_session = ''
        self.operation = 'NAVIGATE'
        self.pending_terminal = None
        self.hold_ack_pending = False
        self.definition_hash = ''

    def _time(self, now):
        if not math.isfinite(now) or self.last_time is not None and now < self.last_time:
            self._fault('CLOCK_FAULT')
            raise ValueError('Invalid monotonic time')
        self.last_time = now

    @property
    def context(self):
        return Context(self.mission, self.instance, self.session, self.generation, self.child)

    def matches(self, context, child=False):
        return (context.mission == self.mission and context.instance == self.instance
                and context.session == self.session and context.generation == self.generation
                and (not child or bool(self.child) and context.child == self.child))

    def _change(self, phase, reason=''):
        self.phase, self.reason = phase, reason
        self.sequence += 1

    def begin(self, mission, now, timeout_s, *, authorized, ready, map_session, definition=''):
        self._time(now)
        if self.phase not in ('IDLE', *self.TERMINAL) or self.owner not in ('NONE', 'HOLD_CONTROLLER'):
            return Decision(False, 'BUSY', self.phase)
        try:
            if str(uuid.UUID(mission)) != mission or uuid.UUID(mission).int == 0:
                raise ValueError()
        except (ValueError, AttributeError):
            return Decision(False, 'INVALID_UUID', self.phase)
        if mission in self.seen_missions:
            return Decision(False, 'REUSED_UUID', self.phase)
        if (not math.isfinite(timeout_s) or timeout_s <= self.cleanup_s
                or not math.isfinite(now + timeout_s)):
            return Decision(False, 'INVALID_DEADLINE', self.phase)
        if not authorized or not ready or not map_session:
            return Decision(False, 'NOT_AUTHORIZED_OR_READY', self.phase)
        self.mission, self.session, self.child = mission, new_id(), new_id()
        self.seen_missions.add(mission)
        self.requests.clear()
        self.generation += 1
        self.owner = 'TASK'
        self.deadline = now + timeout_s
        self.lease_deadline = now + self.lease_s
        self.hold_deadline = self.pause_deadline = 0.
        self.map_session = map_session
        self.operation = 'NAVIGATE'
        self.pending_terminal = None
        self.hold_ack_pending = self.cleanup_confirmed = False
        self.result_code = ''
        self.definition_hash = hashlib.sha256(definition.encode()).hexdigest()
        self._change('RUNNING')
        return Decision(True, 'ACCEPTED', self.phase)

    def progress(self, context, now, *, healthy):
        self.tick(now)
        if (not self.matches(context) or not healthy
                or self.phase not in ('RUNNING', 'RESUMING')):
            return False
        self.lease_deadline = now + self.lease_s
        return True

    def _stop(self, now, terminal, reason):
        # Revocation of the child also represents retiring queued local tokens.
        self.child = ''
        self.pending_terminal = terminal
        self.stop_deadline = min(now + self.cleanup_s, self.deadline)
        self.hold_ack_pending = False
        self._change('PAUSING' if terminal is None else 'STOPPING', reason)

    def cancel(self, context, now):
        self.tick(now)
        if not self.matches(context):
            return Decision(False, 'STALE_IDENTITY', self.phase)
        if self.operation == 'LANDING_COMMITTED':
            return Decision(False, 'CANCEL_REJECTED_LANDING', self.phase)
        if self.phase in self.TERMINAL or self.phase == 'IDLE':
            return Decision(False, 'ALREADY_TERMINAL', self.phase)
        if self.phase == 'STOPPING' and self.pending_terminal != 'CANCELED':
            return Decision(False, 'TERMINATION_COMMITTED', self.phase)
        if self.phase != 'STOPPING':
            self._stop(now, 'CANCELED', 'CANCEL_REQUESTED')
        return Decision(True, 'ACCEPTED', self.phase)

    def command(self, verb, mission, instance, request_id, now, *, ready=False, map_session=''):
        self.tick(now)
        # Identity check precedes cache access; old commands never act on a new root.
        if mission != self.mission or instance != self.instance:
            return Decision(False, 'STALE_IDENTITY', self.phase)
        try:
            if uuid.UUID(request_id).int == 0:
                raise ValueError()
        except (ValueError, AttributeError):
            return Decision(False, 'INVALID_REQUEST_ID', self.phase)
        key = (mission, instance, request_id)
        if key in self.requests:
            old_verb, old_decision = self.requests[key]
            return old_decision if verb == old_verb else Decision(False, 'REQUEST_ID_CONFLICT', self.phase)
        decision = Decision(False, 'INVALID_PHASE', self.phase)
        if verb == 'pause' and self.phase == 'RUNNING':
            if self.operation != 'NAVIGATE':
                decision = Decision(False, 'PAUSE_UNSUPPORTED_OPERATION', self.phase)
            else:
                self._stop(now, None, 'PAUSE_REQUESTED')
                decision = Decision(True, 'ACCEPTED', self.phase)
        elif verb == 'resume' and self.phase == 'PAUSED':
            if not ready or map_session != self.map_session:
                decision = Decision(False, 'RESUME_NOT_READY_OR_SESSION_CHANGED', self.phase)
            else:
                self.generation += 1
                self.owner, self.child = 'TASK', new_id()
                self.lease_deadline = now + self.lease_s
                self.hold_deadline = 0.
                self._change('RESUMING')
                decision = Decision(True, 'ACCEPTED', self.phase)
        self.requests[key] = (verb, decision)
        return decision

    def child_started(self, context, now):
        self.tick(now)
        if self.phase != 'RESUMING' or not self.matches(context, child=True):
            return False
        self._change('RUNNING')
        return True

    def child_finished(self, context, now, *, final_reached, stable):
        self.tick(now)
        if (self.phase != 'RUNNING' or self.operation != 'NAVIGATE'
                or not self.matches(context, child=True)):
            return False
        if not final_reached or not stable:
            return False
        self._stop(now, 'SUCCEEDED', 'FINAL_TARGET_CONFIRMED')
        return True

    def stopped(self, context, now, *, sample_time, stable):
        self.tick(now)
        if (self.phase not in ('PAUSING', 'STOPPING') or self.hold_ack_pending
                or not self.matches(context) or not stable or not math.isfinite(sample_time)
                or not 0 <= now - sample_time <= self.lease_s):
            return False
        # Atomic owner+generation update; old gateway references are now invalid.
        self.owner = 'HOLD_CONTROLLER'
        self.generation += 1
        self.hold_ack_pending = True
        self.sequence += 1
        return True

    def hold_ack(self, context, now, *, reference_time, healthy):
        self.tick(now)
        if (not self.hold_ack_pending or not self.matches(context)
                or not healthy or not math.isfinite(reference_time)
                or not 0 <= now - reference_time <= self.lease_s):
            return False
        self.hold_ack_pending = False
        self.lease_deadline = now + self.lease_s
        if self.pending_terminal is None:
            self.pause_deadline = min(now + self.pause_s, self.deadline - self.cleanup_s)
            self.hold_deadline = self.pause_deadline
            self._change('PAUSED')
        else:
            self.hold_deadline = now + self.final_hold_s
            self.cleanup_confirmed = True
            self.result_code = self.pending_terminal
            self._change(self.pending_terminal, self.reason)
        return True

    def renew_hold(self, context, now, *, reference_time, healthy):
        self.tick(now)
        if (self.owner != 'HOLD_CONTROLLER' or self.hold_ack_pending
                or not self.matches(context) or not healthy
                or not math.isfinite(reference_time) or not 0 <= now - reference_time <= self.lease_s
                or now >= self.hold_deadline):
            return False
        self.lease_deadline = min(now + self.lease_s, self.hold_deadline)
        return True

    def takeover(self, context, now, *, failsafe=False):
        self._time(now)
        if not self.matches(context):
            return False
        self.owner = 'FAILSAFE' if failsafe else 'PILOT'
        self.generation += 1
        self.child = ''
        self.hold_ack_pending = False
        if self.phase not in self.TERMINAL:
            self.result_code = 'CONTROL_LOST'
            self._change('ABORTED', 'CONTROL_LOST')
        else:
            self.reason = 'CONTROL_LOST'
            self.sequence += 1
        return True

    def landing_committed(self, context, now):
        self.tick(now)
        if self.phase != 'RUNNING' or not self.matches(context):
            return False
        self.operation = 'LANDING_COMMITTED'
        self.sequence += 1
        return True

    def _fault(self, reason):
        self.owner, self.child = 'NONE', ''
        self.generation += 1
        self.hold_ack_pending = False
        if self.phase not in self.TERMINAL:
            self.result_code, self.cleanup_confirmed = reason, False
            self._change('ABORTED', reason)
        else:
            self.reason = reason
            self.sequence += 1

    def tick(self, now):
        self._time(now)
        if self.phase in ('PAUSING', 'STOPPING'):
            if now >= self.stop_deadline:
                self._fault('CANCEL_TIMEOUT')
        elif self.phase in ('RUNNING', 'RESUMING', 'PAUSED'):
            if self.operation == 'LANDING_COMMITTED':
                if now >= self.deadline:
                    self._fault('LANDING_DEADLINE_EXCEEDED')
                elif now >= self.lease_deadline:
                    self._fault('LANDING_OBSERVATION_LOST')
                return
            if now >= self.deadline - self.cleanup_s:
                self._stop(now, 'ABORTED', 'TOTAL_DEADLINE_EXCEEDED')
            elif self.phase == 'PAUSED' and now >= self.pause_deadline:
                self._stop(now, 'ABORTED', 'PAUSE_DEADLINE_EXCEEDED')
            elif now >= self.lease_deadline:
                self._stop(now, 'ABORTED', 'PROGRESS_LEASE_EXPIRED')
        elif self.owner == 'HOLD_CONTROLLER' and (
                now >= self.hold_deadline or now >= self.lease_deadline):
            self._fault('FINAL_HOLD_EXPIRED' if now >= self.hold_deadline else 'HOLD_LEASE_EXPIRED')
