import math

import numpy as np
import pytest

from uav_target_perception.core import (ContextGate, PersonTracker, SurfacePoint,
                                         fresh, locate_surface, transform_point)


def observation(x=0., score=.8):
    return SurfacePoint(np.array([x, 0., 3.]), 30, .02), score


def test_surface_uses_supported_pixels_instead_of_a_missing_box_centre():
    depth = np.full((100, 100), 3., dtype=np.float32)
    depth[50, 50] = np.nan
    result = locate_surface(depth, [0., 0., 100., 100.], (100., 100., 49.5, 39.5))
    assert result.samples == 1599
    assert result.spread == 0.
    assert np.allclose(result.position, [0., 0., 3.], atol=.015)


@pytest.mark.parametrize('value', [0., np.nan, np.inf, .1, 20.])
def test_invalid_or_out_of_range_depth_is_not_a_position(value):
    assert locate_surface(np.full((80, 80), value), [0, 0, 80, 80], (100, 100, 40, 40)) is None


def test_mixed_background_and_foreground_is_rejected():
    depth = np.full((100, 100), 3.)
    depth[:, 50:] = 7.
    assert locate_surface(depth, [0, 0, 100, 100], (100, 100, 50, 50)) is None


def test_sparse_depth_does_not_pass_by_a_single_valid_centre():
    depth = np.zeros((100, 100))
    depth[40, 50] = 3.
    assert locate_surface(depth, [0, 0, 100, 100], (100, 100, 50, 50)) is None


@pytest.mark.parametrize('box', [[90, 20, 10, 80], [-100, -100, -50, -50], [1, 1, 2, 2]])
def test_invalid_or_tiny_boxes_never_wrap_numpy_indices(box):
    assert locate_surface(np.ones((100, 100)), box, (100, 100, 50, 50)) is None


def test_transform_rotates_before_translation():
    q = [0, 0, math.sqrt(.5), math.sqrt(.5)]
    assert np.allclose(transform_point([1, 2, 3], [10, 20, 30], q), [8, 21, 33])


@pytest.mark.parametrize('q', [[0, 0, 0, 0], [0, 0, 0, 2], [np.nan, 0, 0, 1]])
def test_bad_transform_cannot_generate_a_marker(q):
    with pytest.raises(ValueError):
        transform_point([1, 2, 3], [0, 0, 0], q)


def test_context_rejects_queued_images_across_map_changes_and_wall_timeout():
    gate = ContextGate(2.)
    assert gate.update(('map1', 1), True, 10., 100.)
    assert gate.accepts(10.1, 10.2, 100.2, .5)
    assert gate.update(('map2', 2), True, 10.2, 100.2)
    assert not gate.accepts(10.1, 10.3, 100.3, .5)
    assert not gate.ready(10.3, 103.)
    assert not gate.update(('old', 0), True, 9., 103.)
    assert gate.key == ('map2', 2)


def test_invalid_context_recovery_also_sets_a_capture_barrier():
    gate = ContextGate()
    gate.update(('map', 1), True, 10., 100.)
    gate.update(('map', 1), False, 10.1, 100.1)
    assert not gate.ready(10.2, 100.2)
    gate.update(('map', 1), True, 10.3, 100.3)
    assert not gate.accepts(10.2, 10.4, 100.4, .5)


@pytest.mark.parametrize('stamp,now', [(0., 0.), (11., 10.), (9., 10.), (float('nan'), 10.)])
def test_zero_future_stale_and_nonfinite_time_are_rejected(stamp, now):
    assert not fresh(stamp, now, .5)


def test_confirmations_require_unique_frames_and_restart_after_a_miss():
    tracker = PersonTracker()
    tracker.update([observation()], 10., 100.)
    tracker.update([observation()], 10., 100.)
    assert not tracker.confirmed()
    tracker.update([observation()], 10.2, 100.2)
    tracker.update([observation()], 10.4, 100.4)
    assert len(tracker.confirmed()) == 1
    tracker.update([], 10.6, 100.6)
    assert not tracker.confirmed()
    tracker.update([observation()], 10.8, 100.8)
    assert not tracker.confirmed()


def test_two_nearby_people_cannot_merge_into_one_track_in_the_same_frame():
    tracker = PersonTracker(min_hits=2)
    tracker.update([observation(0), observation(.4)], 10., 100.)
    tracker.update([observation(.1), observation(.3)], 10.2, 100.2)
    assert len(tracker.confirmed()) == 2
    assert len({track.id for track in tracker.confirmed()}) == 2


def test_simulation_pause_and_time_rewind_expire_tracks():
    tracker = PersonTracker(min_hits=1)
    tracker.update([observation()], 10., 100.)
    tracker.expire(10., 102.)
    assert not tracker.tracks
    tracker.clear()
    tracker.update([observation()], 10., 103.)
    tracker.expire(9., 103.1)
    assert not tracker.tracks


def test_tracker_is_bounded_and_ids_are_not_reused_after_context_clear():
    tracker = PersonTracker(max_tracks=2, min_hits=1)
    tracker.update([observation(0), observation(3), observation(6)], 10., 100.)
    assert len(tracker.tracks) == 2
    tracker.clear()
    tracker.update([observation()], 11., 101.)
    assert list(tracker.tracks) == [3]
