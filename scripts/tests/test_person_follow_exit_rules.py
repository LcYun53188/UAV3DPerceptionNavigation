"""Exit timing with synthetic capture/receipt clocks, no flight evidence."""
from pathlib import Path
import sys
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation/vio/runtime'))
from person_follow_exit_rules import FollowExitClock,FollowExitLimits


def samples(clock,start,end,motion):
    for i in range(round((end-start)*10)+1):
        t=round(start+i*.1,6)
        clock.sample(10+t,100+t,motion)


def test_moving_time_accumulates_across_pauses_without_counting_the_pause():
    c=FollowExitClock()
    samples(c,0,4,'MOVING')
    samples(c,4.1,6,'STATIONARY')
    assert c.reason(106) is None
    c.pause_motion()
    samples(c,8,14,'MOVING')
    assert c.moving_seconds==pytest.approx(10)
    assert c.reason(114)=='MOVING_TIME_BUDGET_REACHED'


def test_static_requires_five_continuous_seconds_of_source_and_receipt():
    c=FollowExitClock()
    samples(c,0,4.9,'STATIONARY')
    assert c.reason(104.9) is None
    c.sample(15,105,'STATIONARY')
    assert c.reason(105)=='PERSON_STATIC_OBSERVED'
    c=FollowExitClock()
    for i in range(61):
        c.sample(10+i*.001,100+i*.1,'STATIONARY')
    assert c.reason(106) is None


def test_loss_deadline_does_not_reset_and_reacquisition_keeps_moving_total():
    c=FollowExitClock()
    samples(c,0,3,'MOVING')
    c.lost(104)
    c.lost(108)
    assert c.reason(108.99) is None
    assert c.reason(109)=='TARGET_LOST_TIMEOUT'
    assert not c.reacquire(109)
    assert c.moving_seconds==pytest.approx(3)


def test_reacquired_target_does_not_credit_the_missing_interval():
    c=FollowExitClock()
    samples(c,0,3,'MOVING')
    c.lost(104)
    assert c.reacquire(106)
    samples(c,6,7,'MOVING')
    assert c.moving_seconds==pytest.approx(4)


def test_multiple_lock_ignores_motion_loss_and_repeated_multi_events():
    c=FollowExitClock()
    c.multiple(100)
    c.multiple(105)
    c.lost(106)
    samples(c,0,9,'MOVING')
    assert not c.reacquire(109)
    assert c.reason(109.99) is None
    assert c.reason(110)=='MULTIPLE_PEOPLE_OBSERVED'
    assert c.moving_seconds==0


def test_replayed_frames_and_capture_gaps_do_not_add_moving_time():
    c=FollowExitClock()
    c.sample(10,100,'MOVING')
    c.sample(10,104,'MOVING')
    c.sample(20,105,'MOVING')
    assert c.moving_seconds==0


def test_capture_gap_breaks_continuous_static_dwell():
    c=FollowExitClock()
    samples(c,0,4,'STATIONARY')
    c.sample(20,110,'STATIONARY')
    assert c.reason(110) is None
    assert c.status(110)['stationary_seconds']==0
