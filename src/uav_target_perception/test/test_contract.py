import copy
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException

from uav_target_perception.core import ContextGate, PersonTracker
from uav_target_perception.node import PersonMapper, check_registration


def inputs():
    rgb = Image(width=100, height=100, encoding='rgb8')
    rgb.header.frame_id = 'camera_optical'
    rgb.header.stamp = Time(seconds=10).to_msg()
    depth = copy.deepcopy(rgb)
    depth.encoding = '16UC1'
    info = CameraInfo(width=100, height=100, header=rgb.header)
    info.k = [100., 0., 50., 0., 100., 50., 0., 0., 1.]
    return rgb, depth, info


def test_registered_millimetre_depth_contract():
    assert check_registration(*inputs(), .03) == (100., 100., 50., 50.)


@pytest.mark.parametrize('field,value,reason', [
    ('width', 99, 'SIZE_MISMATCH'), ('encoding', 'mono8', 'DEPTH_ENCODING'),
])
def test_depth_registration_rejects_incompatible_input(field, value, reason):
    rgb, depth, info = inputs()
    setattr(depth, field, value)
    with pytest.raises(ValueError, match=reason):
        check_registration(rgb, depth, info, .03)


def test_unregistered_frame_and_unsynchronized_depth_are_rejected():
    rgb, depth, info = inputs()
    depth.header.frame_id = 'other_camera'
    with pytest.raises(ValueError, match='FRAME_MISMATCH'):
        check_registration(rgb, depth, info, .03)
    depth.header.frame_id = rgb.header.frame_id
    depth.header.stamp = Time(seconds=10.1).to_msg()
    with pytest.raises(ValueError, match='TIME_MISMATCH'):
        check_registration(rgb, depth, info, .03)


def test_distorted_camera_cannot_be_backprojected_as_rectified():
    rgb, depth, info = inputs()
    info.d = [.1, 0., 0., 0., 0.]
    with pytest.raises(ValueError, match='RECTIFIED'):
        check_registration(rgb, depth, info, .03)


@pytest.mark.parametrize('field,index,value', [('k', 1, .1), ('p', 0, 90.), ('r', 1, .1)])
def test_unsupported_skew_and_rectification_mismatch_are_rejected(field, index, value):
    rgb, depth, info = inputs()
    getattr(info, field)[index] = value
    with pytest.raises(ValueError, match='INTRINSICS|MISMATCH'):
        check_registration(rgb, depth, info, .03)


def test_old_map_epoch_and_changed_map_id_with_same_epoch_do_not_replace_context():
    gate = ContextGate()
    gate.update(('current', 20, '', '', 0, '', ()), True, 10., 100.)
    state = NS(options={'context_source': 'map_snapshot'}, gate=gate, map_context='current',
               tracker=PersonTracker(), pending='queued', publish_state=Mock())
    for map_id, epoch in [('old', 19), ('other', 20)]:
        message = NS(map_id=map_id, epoch=epoch)
        PersonMapper.context_cb(state, message)
    assert state.map_context == 'current' and state.pending == 'queued'
    state.publish_state.assert_not_called()


def test_image_callback_keeps_only_newest_pair_and_rejects_zero_time():
    rgb, depth, info = inputs()
    state = NS(last_processed=None, now_s=lambda: 10.2, options={'max_image_age': .5},
               pending=None, info=info, gate=NS(key=('map', 1)))
    PersonMapper.images_cb(state, rgb, depth)
    old = state.pending
    stale = copy.deepcopy(rgb)
    stale.header.stamp = Time(seconds=0).to_msg()
    PersonMapper.images_cb(state, stale, depth)
    assert state.pending is old
    newer = copy.deepcopy(rgb)
    newer.header.stamp = Time(seconds=10.1).to_msg()
    PersonMapper.images_cb(state, newer, depth)
    PersonMapper.images_cb(state, rgb, depth)
    assert state.pending['rgb'] is newer


def processing_state():
    rgb, depth, info = inputs()
    gate = ContextGate()
    gate.update(('map', 1), True, 9.9, time.monotonic())
    state = NS(pending=dict(rgb=rgb, depth=depth, info=info, key=gate.key, detections=None),
               gate=gate, now_s=lambda: 10.2, bridge=Mock(),
               options=dict(max_image_age=.5, sync_slop=.03, near_m=.3, far_m=8., depth_spread_m=.35),
               detector=Mock(), tf=Mock(), publish_detections=Mock(), tracker=PersonTracker(min_hits=1))
    state.bridge.imgmsg_to_cv2.side_effect = lambda msg, **kwargs: (
        np.zeros((100, 100, 3), dtype=np.uint8) if msg is rgb else np.full((100, 100), 3000, dtype=np.uint16))
    state.detector.detect.return_value = [(np.array([0., 0., 100., 100.]), .8)]
    state.tf.lookup_transform.return_value = NS(transform=NS(
        translation=NS(x=1., y=2., z=3.), rotation=NS(x=0., y=0., z=0., w=1.)))
    return state


def test_missing_historical_tf_retries_same_capture_without_repeating_inference():
    state = processing_state()
    state.tf.lookup_transform.side_effect = TransformException('not yet available')
    with pytest.raises(TransformException):
        PersonMapper.process_pending(state)
    assert state.pending is not None and not state.tracker.tracks
    assert state.tf.lookup_transform.call_args.args[2].nanoseconds == 10_000_000_000
    state.tf.lookup_transform.side_effect = None
    PersonMapper.process_pending(state)
    state.detector.detect.assert_called_once()
    assert np.allclose(state.tracker.confirmed()[0].surface.position, [.985, 1.685, 6.])


def test_inference_that_misses_source_deadline_cannot_emit_positions():
    state = processing_state()
    def slow(_image):
        state.now_s = lambda: 10.7
        return []
    state.detector.detect.side_effect = slow
    with pytest.raises(ValueError, match='DEADLINE'):
        PersonMapper.process_pending(state)
    state.publish_detections.assert_not_called()
    state.tf.lookup_transform.assert_not_called()
    assert not state.tracker.tracks


def test_non_unique_context_admission_cannot_create_map_tracks():
    state = processing_state()
    with pytest.raises(ValueError, match='CONTEXT'):
        PersonMapper.process_pending(state, allow_map=False)
    state.publish_detections.assert_called_once()
    state.tf.lookup_transform.assert_not_called()
    assert not state.tracker.tracks
