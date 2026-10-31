"""Follow candidate geometry and lifecycle are independent of any flight output."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'simulation/vio/runtime'))
from person_follow_core import FollowPreview, candidate_goal


def test_candidate_preserves_height_and_bounds_step():
    goal,distance,reason = candidate_goal([0,0,1.5],[-5,0,2.])
    assert goal == [-4.5,0.,2.]
    assert distance == 5.
    assert reason == 'CANDIDATE_ONLY_NOT_PLANNED'


@pytest.mark.parametrize('aircraft,reason',[([2,0,1.],'TARGET_TOO_CLOSE'),([3.1,0,1.],'WITHIN_STANDOFF')])
def test_no_automatic_retreat_or_motion_inside_standoff(aircraft,reason):
    goal,_,result = candidate_goal([0,0,0],aircraft)
    assert goal is None and result == reason


def test_invalid_geometry():
    with pytest.raises(ValueError):
        candidate_goal([float('nan'),0,0],[3,0,1])


def test_preview_pause_resume_cancel_and_execution_refusal():
    preview = FollowPreview()
    assert not preview.command('START')[0]
    preview.observe('1:person',[0,0,1],[-5,0,2],10.)
    assert preview.state == 'READY' and preview.goal is None
    assert preview.command('START')[0]
    preview.observe('1:person',[.02,0,1],[-5,0,2],10.2)
    assert preview.state == 'PREVIEW' and preview.goal is not None
    assert preview.command('PAUSE')[0]
    preview.observe('1:person',[.04,0,1],[-5,0,2],10.4)
    assert preview.state == 'PAUSED' and preview.goal is None
    assert preview.command('RESUME')[0]
    assert preview.command('CANCEL')[0]
    assert preview.goal is None and not preview.active
    accepted,reason = preview.command('EXECUTE')
    assert not accepted and 'DYNAMIC_SAFETY_UNQUALIFIED' in reason


def test_track_change_requires_new_selection_not_automatic_rebinding():
    preview = FollowPreview()
    preview.observe('original',[0,0,1],[-5,0,2],10.)
    preview.command('START')
    preview.observe('other',[0,0,1],[-5,0,2],10.2)
    assert preview.state == 'LOST' and preview.goal is None
    preview.observe('original',[0,0,1],[-5,0,2],10.4)
    assert preview.state == 'LOST' and not preview.command('RESUME')[0]


def test_velocity_uses_capture_time_and_does_not_accept_large_jump():
    preview = FollowPreview()
    for stamp,x in [(10.,0.),(10.2,.1),(10.4,.2)]:
        preview.observe('person',[x,0,1],[-5,0,2],stamp)
    assert preview.motion == 'MOVING'
    assert preview.velocity == pytest.approx([.5,0.])
    preview.observe('person',[5,0,1],[-5,0,2],10.5)
    assert preview.state == 'LOST'
