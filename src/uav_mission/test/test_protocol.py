from dataclasses import replace
import math

import pytest

from uav_mission.protocol import Protocol, new_id


def running(**kwargs):
    p = Protocol(**kwargs)
    assert p.begin(new_id(), 0., 100., authorized=True, ready=True,
                   map_session='map-1', definition='target-A').accepted
    return p


def command(p, verb, now, request=None, **kwargs):
    return p.command(verb, p.mission, p.instance, request or new_id(), now, **kwargs)


def handoff(p, now):
    before = p.context
    assert p.stopped(before, now, sample_time=now, stable=True)
    assert p.generation == before.generation + 1
    assert p.owner == 'HOLD_CONTROLLER'
    assert not p.hold_ack(before, now, reference_time=now, healthy=True)
    assert p.hold_ack(p.context, now, reference_time=now, healthy=True)


def test_unknown_not_authorized_or_ready():
    p = Protocol()
    for auth, ready in ((False, True), (True, False), (False, False)):
        assert not p.begin(new_id(), 0., 10., authorized=auth, ready=ready,
                           map_session='map-1').accepted
    assert p.owner == 'NONE'


def test_pause_resume_new_child_generation_and_fixed_total_deadline():
    p = running()
    old, deadline = p.context, p.deadline
    assert command(p, 'pause', .1).phase == 'PAUSING'
    assert not p.child and p.phase not in p.TERMINAL
    assert not p.child_finished(old, .1, final_reached=True, stable=True)
    handoff(p, .2)
    assert p.phase == 'PAUSED' and not p.cleanup_confirmed
    assert command(p, 'resume', .3, ready=True, map_session='map-1').accepted
    assert p.phase == 'RESUMING' and p.child != old.child
    assert not p.child_started(old, .3)
    assert p.child_started(p.context, .3)
    assert p.phase == 'RUNNING' and p.deadline == deadline


def test_pause_idempotency_conflict_and_stale_instance():
    p = running()
    request = new_id()
    decision = command(p, 'pause', .1, request)
    handoff(p, .2)
    assert command(p, 'pause', .3, request) == decision
    assert command(p, 'resume', .3, request).reason == 'REQUEST_ID_CONFLICT'
    assert p.command('resume', p.mission, new_id(), new_id(), .3).reason == 'STALE_IDENTITY'
    assert p.phase == 'PAUSED'


def test_cancel_preempts_pause_and_requires_fresh_hold_ack():
    p = running()
    command(p, 'pause', .1)
    assert p.cancel(p.context, .2).accepted and p.phase == 'STOPPING'
    before = p.context
    assert not p.stopped(before, .2, sample_time=-1., stable=True)
    assert not p.stopped(before, .2, sample_time=.2, stable=False)
    assert p.stopped(before, .2, sample_time=.2, stable=True)
    assert p.phase == 'STOPPING' and not p.cleanup_confirmed
    assert not p.hold_ack(p.context, .3, reference_time=math.nan, healthy=True)
    assert not p.hold_ack(p.context, .3, reference_time=.3, healthy=False)
    assert p.hold_ack(p.context, .3, reference_time=.3, healthy=True)
    assert p.phase == 'CANCELED' and p.cleanup_confirmed
    seq = p.sequence
    assert not p.hold_ack(p.context, .3, reference_time=.3, healthy=True)
    assert not p.cancel(p.context, .3).accepted
    assert p.sequence == seq


def test_cancel_timeout_never_claims_cleanup():
    p = running()
    p.cancel(p.context, .1)
    p.tick(1.1)
    assert (p.phase, p.result_code) == ('ABORTED', 'CANCEL_TIMEOUT')
    assert not p.cleanup_confirmed and p.owner == 'NONE'


@pytest.mark.parametrize('attribute', ['mission', 'instance', 'session', 'generation', 'child'])
def test_old_events_rejected_by_each_identity_layer(attribute):
    p = running()
    bad = replace(p.context, **{attribute: 999 if attribute == 'generation' else new_id()})
    assert not p.child_finished(bad, .1, final_reached=True, stable=True)
    assert p.phase == 'RUNNING'


def test_new_root_rejects_old_cancel_and_old_request():
    p = running()
    old = p.context
    p.cancel(old, .1)
    handoff(p, .2)
    assert p.begin(new_id(), .3, 10., authorized=True, ready=True, map_session='map-2').accepted
    assert p.cancel(old, .3).reason == 'STALE_IDENTITY'
    assert p.command('pause', old.mission, old.instance, new_id(), .3).reason == 'STALE_IDENTITY'
    assert p.phase == 'RUNNING'


def test_final_hold_does_not_slide_on_renewal_and_survives_root():
    p = running(final_hold_s=1.)
    p.cancel(p.context, .1)
    handoff(p, .2)
    final = p.hold_deadline
    for now in (.3, .6, .9, 1.1):
        assert p.renew_hold(p.context, now, reference_time=now, healthy=True)
        assert p.hold_deadline == final and p.phase == 'CANCELED'
    p.tick(1.2)
    assert p.owner == 'NONE' and p.reason == 'FINAL_HOLD_EXPIRED'
    assert p.phase == 'CANCELED'  # Past Action result is immutable.


def test_progress_stall_starts_bounded_cleanup():
    p = running()
    p.tick(.5)
    assert p.phase == 'STOPPING' and p.reason == 'PROGRESS_LEASE_EXPIRED'
    handoff(p, .6)
    assert p.phase == 'ABORTED' and p.cleanup_confirmed


def test_pause_limit_and_total_deadline_include_cleanup():
    p = running(pause_s=.6)
    command(p, 'pause', .1)
    handoff(p, .2)
    assert p.renew_hold(p.context, .5, reference_time=.5, healthy=True)
    p.tick(.8)
    assert p.phase == 'STOPPING' and p.reason == 'PAUSE_DEADLINE_EXCEEDED'
    q = Protocol(cleanup_s=1.)
    q.begin(new_id(), 0., 2., authorized=True, ready=True, map_session='map-1')
    q.progress(q.context, .4, healthy=True)
    q.progress(q.context, .8, healthy=True)
    q.tick(1.)
    assert q.phase == 'STOPPING' and q.stop_deadline == 2.
    q.tick(2.)
    assert q.phase == 'ABORTED' and not q.cleanup_confirmed


def test_resume_rejects_map_change_and_unknown_readiness():
    p = running()
    command(p, 'pause', .1)
    handoff(p, .2)
    assert not command(p, 'resume', .3, ready=False, map_session='map-1').accepted
    assert not command(p, 'resume', .3, ready=True, map_session='map-2').accepted
    assert p.owner == 'HOLD_CONTROLLER'


@pytest.mark.parametrize('failsafe,owner', [(False, 'PILOT'), (True, 'FAILSAFE')])
def test_actual_takeover_revokes_all_automatic_owners(failsafe, owner):
    p = running()
    old = p.context
    assert p.takeover(old, .1, failsafe=failsafe)
    assert p.phase == 'ABORTED' and p.result_code == 'CONTROL_LOST'
    assert p.owner == owner and not p.child
    assert not p.renew_hold(old, .2, reference_time=.2, healthy=True)
    assert not p.begin(new_id(), .2, 10., authorized=True, ready=True, map_session='map-1').accepted


def test_landing_committed_rejects_pause_and_cancel():
    p = running()
    assert p.landing_committed(p.context, .1)
    assert command(p, 'pause', .2).reason == 'PAUSE_UNSUPPORTED_OPERATION'
    assert p.cancel(p.context, .2).reason == 'CANCEL_REJECTED_LANDING'
    assert p.phase == 'RUNNING'


def test_final_success_requires_final_identity_and_stable_then_cleanup():
    p = running()
    assert not p.child_finished(p.context, .1, final_reached=False, stable=True)
    assert not p.child_finished(p.context, .1, final_reached=True, stable=False)
    assert p.child_finished(p.context, .1, final_reached=True, stable=True)
    assert p.phase == 'STOPPING'
    handoff(p, .2)
    assert p.phase == 'SUCCEEDED' and p.cleanup_confirmed


@pytest.mark.parametrize('bad', [math.nan, math.inf, -1.])
def test_bad_monotonic_time_fails_closed(bad):
    p = running()
    with pytest.raises(ValueError):
        p.tick(bad)

    assert p.owner == 'NONE' and p.phase == 'ABORTED'


def test_completion_cancel_race_uses_owner_event_order():
    p = running()
    child = p.context
    assert p.child_finished(child, .1, final_reached=True, stable=True)
    assert p.cancel(p.context, .1).reason == 'TERMINATION_COMMITTED'
    handoff(p, .2)
    assert p.phase == 'SUCCEEDED'
    q = running()
    child = q.context
    assert q.cancel(child, .1).accepted
    assert not q.child_finished(child, .1, final_reached=True, stable=True)
    handoff(q, .2)
    assert q.phase == 'CANCELED'


@pytest.mark.parametrize('timeout', [0., .5, 1., math.nan, math.inf])
def test_invalid_total_deadline_never_acquires_control(timeout):
    p = Protocol()
    assert not p.begin(new_id(), 0., timeout, authorized=True, ready=True, map_session='map-1').accepted
    assert p.owner == 'NONE'


def test_nil_uuid_is_not_an_identity_or_request():
    p = Protocol()
    nil = '00000000-0000-0000-0000-000000000000'
    assert not p.begin(nil, 0., 10., authorized=True, ready=True, map_session='map-1').accepted
    p = running()
    assert command(p, 'pause', .1, nil).reason == 'INVALID_REQUEST_ID'


def test_hold_lease_loss_does_not_change_completed_action_result():
    p = running()
    p.cancel(p.context, .1)
    handoff(p, .2)
    p.tick(.7)
    assert p.owner == 'NONE' and p.reason == 'HOLD_LEASE_EXPIRED'
    assert p.phase == 'CANCELED' and p.result_code == 'CANCELED'


def test_landing_observation_loss_cannot_request_offboard_hold():
    p = running()
    child = p.context
    assert p.landing_committed(child, .1)
    assert not p.child_finished(child, .2, final_reached=True, stable=True)
    p.tick(.5)
    assert p.phase == 'ABORTED' and p.reason == 'LANDING_OBSERVATION_LOST'
    assert p.owner == 'NONE' and not p.hold_ack_pending


def test_landing_deadline_cannot_request_offboard_hold():
    p = running(lease_s=200.)
    assert p.landing_committed(p.context, .1)
    p.tick(100.)
    assert p.phase == 'ABORTED' and p.reason == 'LANDING_DEADLINE_EXCEEDED'
    assert p.owner == 'NONE'
