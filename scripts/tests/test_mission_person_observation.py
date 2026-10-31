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
    p=MissionPersonObservation(EpisodeConfig(follow_s=2.,cooldown_s=3.))
    p.enable(True,100.)
    for i in range(45):
        observe(p,i*.1,i*.04)
    assert len(p.episodes)==1 and p.phase=='COOLDOWN'
    assert p.episodes[-1]['end_reason']=='FOLLOW_TIME_BUDGET_REACHED'


def test_target_and_mission_changes_end_old_episode():
    for changes,reason in [({'track_id':'other'},'TARGET_CHANGED'),({'mission':('new','c','s',1)},'MISSION_IDENTITY_CHANGED'),
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
