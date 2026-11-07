"""Task-interruption shadow policy: motion confirmation, bounded episodes and history."""
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation/vio/runtime'))
from mission_person_observation import EpisodeConfig,MissionPersonObservation

CONTEXT=('map',1,'loc','align',1,'context')
MISSION=('root','coordinator','control',0)


def observe(policy,t,x,**overrides):
    policy.sample(track_id=overrides.get('track_id','person'),point=[x,0.,1.],
                  source_stamp=10.+t,mono=100.+t,context=overrides.get('context',CONTEXT),
                  mission=overrides.get('mission',MISSION))


def test_off_by_default_and_no_trigger_for_stationary_person():
    p=MissionPersonObservation()
    observe(p,0.,0.)
    assert not p.episodes
    p.enable(True,100.)
    for i in range(30):
        observe(p,i*.1,0.)
    assert p.phase=='MONITORING' and not p.episodes


def test_moving_then_stationary_observe_and_finish_without_changing_mission():
    p=MissionPersonObservation()
    p.enable(True,100.)
    for i in range(20):
        observe(p,i*.1,i*.04)
    assert p.phase=='SHADOW_FOLLOW'
    assert p.active['mission']==list(MISSION)
    assert p.active['points'][0]['stamp']<p.active['started_mono']-90.
    for i in range(20,95):
        observe(p,i*.1,.76)
    assert p.phase=='COOLDOWN'
    assert p.episodes[-1]['end_reason']=='PERSON_STATIC_OBSERVED'
    assert p.status()['mission_interrupted'] is False
    assert p.status()['execution_ready'] is False
    assert all(a['stamp']<b['stamp'] for a,b in zip(p.episodes[-1]['points'],p.episodes[-1]['points'][1:]))


def test_continuous_motion_has_time_budget_and_cooldown():
    p=MissionPersonObservation(EpisodeConfig(moving_follow_s=2.,cooldown_s=3.))
    p.enable(True,100.)
    for i in range(45):
        observe(p,i*.1,i*.04)
    assert len(p.episodes)==1 and p.phase=='COOLDOWN'
    assert p.episodes[-1]['end_reason']=='MOVING_TIME_BUDGET_REACHED'


def test_target_and_mission_changes_end_old_episode():
    for changes,reason in [({'mission':('new','c','s',1)},'MISSION_IDENTITY_CHANGED'),
                           ({'context':('other',2,'l','a',2,'c')},'MAP_CONTEXT_CHANGED')]:
        p=MissionPersonObservation()
        p.enable(True,100.)
        for i in range(20):
            observe(p,i*.1,i*.04)
        observe(p,2.,.8,**changes)
        assert p.active is None
        assert p.episodes[-1]['end_reason']==reason


def test_duplicate_capture_never_extends_history_and_conflict_invalidates():
    p=MissionPersonObservation()
    p.enable(True,100.)
    observe(p,0.,0.)
    observe(p,0.,0.)
    assert len(p.samples)==1
    observe(p,0.,1.)
    assert p.reason=='CONFLICTING_CAPTURE' and not p.samples


def test_mode_off_finishes_episode_and_history_is_bounded():
    p=MissionPersonObservation(EpisodeConfig(max_points=5))
    p.enable(True,100.)
    for i in range(25):
        observe(p,i*.1,i*.04)
    assert len(p.active['points'])==5 and p.active['truncated']
    p.enable(False,102.5)
    assert p.phase=='DISABLED' and p.active is None
    assert p.episodes[-1]['end_reason']=='MODE_DISABLED'


def test_receipt_replay_cannot_extend_episode_or_reseed_an_expired_capture():
    p=MissionPersonObservation()
    p.enable(True,100.)
    for i in range(20):
        observe(p,i*.1,i*.04)
    assert p.active is not None
    p.advance(102.6)
    assert p.active is not None and p.phase=='SHADOW_LOST_WAIT'
    p.advance(107.6)
    assert p.active is None and p.episodes[-1]['end_reason']=='TARGET_LOST_TIMEOUT'
    p.sample(track_id='person',point=[.76,0,1.],source_stamp=11.9,mono=107.7,context=CONTEXT,mission=MISSION)
    assert not p.samples and p.last_mono is None


def test_new_mission_requires_explicit_mode_reauthorization():
    p=MissionPersonObservation()
    p.enable(True,100.)
    observe(p,0.,0.)
    new=('next-root','coordinator','control',0)
    observe(p,.1,.01,mission=new)
    assert p.enabled is False and p.phase=='DISABLED'
    assert p.reason=='MISSION_IDENTITY_CHANGED_REAUTHORIZE_MODE'
    p.enable(True,100.2)
    observe(p,.2,.02,mission=new)
    assert p.enabled and p.mission==new


def test_static_dwell_needs_capture_progress_as_well_as_wall_time():
    p=MissionPersonObservation(EpisodeConfig(motion_confirm_s=.1,motion_displacement_m=.02,min_samples=3,
        stationary_observe_s=.6,moving_follow_s=10.))
    p.enable(True,100.)
    for i in range(5):
        observe(p,i*.05,i*.02)
    assert p.active is not None
    # Repeated static captures cannot accumulate a six-tenths source interval.
    for i in range(1,10):
        p.sample(track_id='person',point=[.08,0,1.],source_stamp=10.2+i*.02,
                 mono=100.2+i*.1,context=CONTEXT,mission=MISSION)
    assert p.episodes[-1]['end_reason'] is None


def test_no_frame_watchdog_enters_loss_wait_instead_of_crediting_movement():
    p=MissionPersonObservation()
    p.enable(True,100.)
    for i in range(20):
        observe(p,i*.1,i*.04)
    total=p.clock.moving_seconds
    p.advance(102.6)
    assert p.phase=='SHADOW_LOST_WAIT'
    p.lost(105.)
    p.advance(107.6)
    assert p.active is None and p.episodes[-1]['end_reason']=='TARGET_LOST_TIMEOUT'
    assert p.clock.moving_seconds==total


def test_different_identity_waits_five_seconds_without_retargeting():
    p=MissionPersonObservation()
    p.enable(True,100.)
    for i in range(20):
        observe(p,i*.1,i*.04)
    observe(p,2.,.8,track_id='other')
    assert p.phase=='SHADOW_LOST_WAIT' and p.active['track_id']=='person'
    for i in range(21,70):
        observe(p,i*.1,i*.04,track_id='other')
    assert p.active is not None
    p.advance(107.)
    assert p.episodes[-1]['end_reason']=='TARGET_LOST_TIMEOUT'


def test_reconfirmation_preserves_moving_total_and_splits_trajectory():
    p=MissionPersonObservation()
    p.enable(True,100.)
    for i in range(30):
        observe(p,i*.1,i*.04)
    total=p.clock.moving_seconds
    p.lost(103.)
    for i in range(31,49):
        observe(p,i*.1,i*.04)
    assert p.phase=='SHADOW_FOLLOW'
    assert p.clock.moving_seconds>=total
    assert p.clock.moving_seconds<total+.5
    assert {q['segment'] for q in p.active['points']}=={0,1}


def test_multiple_observation_stays_locked_when_only_one_person_remains():
    p=MissionPersonObservation()
    p.enable(True,100.)
    p.multiple(mono=100.,context=CONTEXT,mission=MISSION,count=2)
    for i in range(1,100):
        observe(p,i*.1,i*.04)
    assert p.phase=='SHADOW_MULTI_OBSERVE'
    p.advance(110.)
    assert p.active is None and p.episodes[-1]['end_reason']=='MULTIPLE_PEOPLE_OBSERVED'
    assert not p.episodes[-1]['points']
