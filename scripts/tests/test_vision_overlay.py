"""Geometry/age regressions: stale data must never look like live AR guidance."""
import sys
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'simulation/vio/runtime'))
from vision_overlay import fresh, image_bgr, project_segments

K = [100,0,100,0,100,100,0,0,1]


def project(points, **kwargs):
    return project_segments(points, kwargs.get('translation', (0,0,0)),
                            kwargs.get('quaternion', (0,0,0,1)), K, 200, 200)


def test_projection_pinhole_and_camera_translation():
    assert project([(0,0,2),(1,0,2)]) == [((100,100),(150,100))]
    assert project([(0,0,2),(1,0,2)], translation=(-1,0,0)) == [((50,100),(100,100))]


def test_projection_rotation_and_behind_camera():
    assert project([(0,0,2),(1,0,2)], quaternion=(0,0,1,0)) == [((100,100),(50,100))]
    assert project([(0,0,-2),(1,0,-2)]) == []


def test_projection_clips_near_plane_and_image_without_joining_hidden_segments():
    segments = project([(0,0,-1),(.1,0,2),(100,0,2)])
    assert len(segments) == 2
    assert all(0 <= v < 200 for segment in segments for point in segment for v in point)
    assert project([(100,0,2),(200,0,2)]) == []


@pytest.mark.parametrize('points', [[(np.nan,0,1),(0,0,1)], [(0,0),(0,1)]])
def test_invalid_geometry(points):
    with pytest.raises(ValueError):
        project(points)


def test_zero_quaternion_is_rejected():
    with pytest.raises(ValueError):
        project([(0,0,1),(1,0,1)], quaternion=(0,0,0,0))


@pytest.mark.parametrize('source,now,received,mono', [
    (1,3,10,10), (1,1,10,12), (2,1,10,10), (1,1,12,10), (float('nan'),1,1,1)])
def test_age_rejects_old_source_old_arrival_future_and_clock_reset(source,now,received,mono):
    assert not fresh(source,now,received,mono)


def test_age_accepts_fresh():
    assert fresh(10,10.1,20,20.1)


def test_image_padding_and_rgb_channel_conversion():
    msg = NS(encoding='rgb8',width=1,height=2,step=4,data=bytes([255,0,0,99,0,255,0,99]))
    assert image_bgr(msg).tolist() == [[[0,0,255]],[[0,255,0]]]


def test_malformed_image_is_rejected():
    with pytest.raises(ValueError):
        image_bgr(NS(encoding='mono8',width=2,height=1,step=1,data=b'x'))


from vision_overlay import TargetSelection


def test_selection_follows_geometry_across_detection_reordering():
    selection = TargetSelection()
    first = [(0,0,100,100), (200,0,300,100)]
    selection.choose(first, 0, ('camera', 1.), 10.)
    selection.update([(202,0,302,100), (5,0,105,100)], ('camera', 1.1), 10.1)
    assert selection.state == 'SELECTED'
    assert selection.index == 1
    assert selection.sequence == 1


def test_selection_repaint_does_not_renew_deadline():
    selection = TargetSelection()
    boxes = [(0,0,100,100)]
    selection.choose(boxes, 0, ('camera', 1.), 10.)
    selection.update(boxes, ('camera', 1.), 10.9)
    selection.update(boxes, ('camera', 1.), 11.1)
    assert selection.state == 'LOST'
    selection.update(boxes, ('camera', 2.), 11.2)
    assert selection.state == 'LOST'  # Never auto-reacquire another person.


@pytest.mark.parametrize('boxes,serial', [
    ([], ('camera', 1.1)),
    ([(200,0,300,100)], ('camera', 1.1)),
    ([(0,0,100,100),(5,0,105,100)], ('camera', 1.1)),
    ([(0,0,100,100)], ('other-camera', 1.1)),
    ([(0,0,100,100)], ('camera', .9)),
    ([(0,0,100,100)], None),
])
def test_selection_loses_on_ambiguity_missing_target_or_context(boxes,serial):
    selection = TargetSelection()
    selection.choose([(0,0,100,100)], 0, ('camera', 1.), 10.)
    selection.update(boxes, serial, 10.1)
    assert selection.state == 'LOST'
    assert selection.index is None


def test_selection_rejects_candidate_also_associated_with_other_person():
    selection = TargetSelection()
    selection.choose([(0,0,100,100),(60,0,160,100)],0,('camera',1.),10.)
    selection.update([(30,0,130,100)], ('camera',1.1), 10.1)
    assert selection.state == 'LOST'
    selection.clear()
    assert selection.state == 'NONE'
