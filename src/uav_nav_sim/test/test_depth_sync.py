"""Depth must wait for its historical TF without crossing input gates."""
from collections import deque
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from rclpy.time import Time
from rclpy.clock import ClockType
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformException
from uav_nav_sim.map_session import MapSession


@pytest.fixture
def session():
    now = [10.1]
    info = CameraInfo(width=2, height=2)
    info.k[0] = info.k[4] = 1.
    node = NS(busy=False, input_enabled=True, mode='mapping', camera_info=info,
              executor_state='HOLD', pending_depth=deque(maxlen=8),
              last_depth_stamp=None, last_color_stamp=None,
              pending_color=deque(maxlen=8), color_timing=dict(received=0, forwarded=0, expired=0, tf_wait=0),
              tf=Mock(), info_pub=Mock(), depth_pub=Mock(),
              get_clock=lambda: NS(now=lambda: Time(seconds=now[0], clock_type=ClockType.ROS_TIME)))
    node.flush_depth = lambda: MapSession.flush_depth(node)
    return node, now


def depth(seconds=10.):
    msg = Image(width=2, height=2, encoding='32FC1')
    msg.header.frame_id = 'oakd_camera_optical_frame'
    msg.header.stamp = Time(seconds=seconds).to_msg()
    return msg


def test_delayed_tf_retries_exact_stamp_and_aligned_intrinsics(session):
    node, _ = session
    node.tf.lookup_transform.side_effect = TransformException('future extrapolation')
    msg = depth()
    MapSession.depth_cb(node, msg)
    node.depth_pub.publish.assert_not_called()
    node.camera_info.k[0] = 2.
    node.tf.lookup_transform.side_effect = None
    node.flush_depth()
    node.depth_pub.publish.assert_called_once_with(msg)
    sent_info = node.info_pub.publish.call_args.args[0]
    assert sent_info.header == msg.header and sent_info.k[0] == 1.
    assert node.tf.lookup_transform.call_args.args[2].nanoseconds == 10_000_000_000
    node.flush_depth()
    MapSession.depth_cb(node, msg)
    assert node.depth_pub.publish.call_count == 1


def test_expired_image_is_not_forwarded_when_tf_arrives(session):
    node, now = session
    node.tf.lookup_transform.side_effect = TransformException('missing')
    MapSession.depth_cb(node, depth())
    now[0] = 10.6
    node.tf.lookup_transform.side_effect = None
    node.flush_depth()
    assert not node.pending_depth
    node.depth_pub.publish.assert_not_called()


def test_gate_discards_pre_teleport_image(session):
    node, _ = session
    node.tf.lookup_transform.side_effect = TransformException('missing')
    MapSession.depth_cb(node, depth())
    assert MapSession.set_input_enabled(node, NS(data=False), NS()).success
    assert MapSession.set_input_enabled(node, NS(data=True), NS()).success
    node.tf.lookup_transform.side_effect = None
    node.flush_depth()
    node.depth_pub.publish.assert_not_called()


@pytest.mark.parametrize('field,value', [('busy',True), ('mode','localization'), ('input_enabled',False)])
def test_disabled_mapping_clears_queue(session, field, value):
    node, _ = session
    node.pending_depth.append((depth(), node.camera_info))
    setattr(node, field, value)
    node.flush_depth()
    assert not node.pending_depth
    node.depth_pub.publish.assert_not_called()


def test_waiting_queue_is_bounded_and_keeps_order(session):
    node, now = session
    now[0] = 10.4
    node.tf.lookup_transform.side_effect = TransformException('missing')
    for i in range(12):
        MapSession.depth_cb(node, depth(10.+i*.02))
    assert len(node.pending_depth) == 8
    node.tf.lookup_transform.side_effect = None
    node.flush_depth()
    stamps = [Time.from_msg(call.args[0].header.stamp).nanoseconds for call in node.depth_pub.publish.call_args_list]
    assert len(stamps) == 8 and stamps == sorted(set(stamps))


def test_depth_timing_tracks_tf_wait_and_forwarding(session):
    node,now=session
    node.timing=dict(received=0,forwarded=0,tf_wait=0,expired=0)
    node.tf.lookup_transform.side_effect=TransformException('future')
    MapSession.depth_cb(node,depth())
    assert node.timing['received']==1 and node.timing['tf_wait']==1
    assert node.last_raw_stamp==10_000_000_000
    node.tf.lookup_transform.side_effect=None
    node.flush_depth()
    assert node.timing['forwarded']==1
    MapSession.depth_cb(node,depth(10.05))
    node.tf.lookup_transform.side_effect=TransformException('future')
    MapSession.depth_cb(node,depth(10.08))
    now[0]=10.7
    node.flush_depth()
    assert node.timing['expired']==1


def test_diagnostics_distinguish_forwarded_and_integrated_source_age(session):
    import json
    node,now=session
    node.last_raw_stamp=10_000_000_000
    node.last_depth_stamp=10_000_000_000
    node.timing=dict(source_stamp_ns=8_000_000_000,query_seconds=.02,parse_seconds=.005)
    node.diagnostics=Mock();node.query_pending=False
    MapSession.publish_diagnostics(node)
    data=json.loads(node.diagnostics.publish.call_args.args[0].data)
    assert data['forwarded_age']==pytest.approx(.1)
    assert data['cached_source_age']==pytest.approx(2.1)
    assert data['query_seconds']==.02 and data['parse_seconds']==.005


@pytest.mark.parametrize('field,value', [('busy', True), ('mode', 'localization'), ('input_enabled', False)])
def test_color_cannot_integrate_when_mapping_is_gated(session, field, value):
    node, _ = session
    node.color_pub = Mock()
    node.color_info_pub = Mock()
    msg = depth()
    msg.encoding = 'rgb8'
    setattr(node, field, value)
    MapSession.color_cb(node, msg)
    MapSession.flush_color(node)
    node.color_pub.publish.assert_not_called()


def test_color_tf_wait_alignment_and_teleport_gate(session):
    node, _ = session
    node.color_pub = Mock()
    node.color_info_pub = Mock()
    node.tf.lookup_transform.side_effect = TransformException('future')
    msg = depth()
    msg.encoding = 'rgb8'
    MapSession.color_cb(node, msg)
    node.color_pub.publish.assert_not_called()
    assert node.color_timing['tf_wait'] == 1
    node.tf.lookup_transform.side_effect = None
    MapSession.flush_color(node)
    node.color_pub.publish.assert_called_once_with(msg)
    assert node.color_info_pub.publish.call_args.args[0].header == msg.header
    assert node.last_depth_stamp is None
    MapSession.color_cb(node, msg)
    assert node.color_pub.publish.call_count == 1
    node.tf.lookup_transform.side_effect = TransformException('future')
    msg = depth(10.05)
    msg.encoding = 'rgb8'
    MapSession.color_cb(node, msg)
    assert MapSession.set_input_enabled(node, NS(data=False), NS()).success
    assert MapSession.set_input_enabled(node, NS(data=True), NS()).success
    node.tf.lookup_transform.side_effect = None
    MapSession.flush_color(node)
    assert node.color_pub.publish.call_count == 1


def test_color_expiration_and_bad_calibration(session):
    node, now = session
    node.color_pub = Mock()
    node.color_info_pub = Mock()
    msg = depth()
    msg.encoding = 'rgb8'
    node.camera_info.width = 3
    MapSession.color_cb(node, msg)
    assert not node.pending_color
    node.camera_info.width = 2
    node.tf.lookup_transform.side_effect = TransformException('future')
    MapSession.color_cb(node, msg)
    now[0] = 10.6
    node.tf.lookup_transform.side_effect = None
    MapSession.flush_color(node)
    assert node.color_timing['expired'] == 1
    node.color_pub.publish.assert_not_called()


def test_exact_tf_failure_is_reported_and_cleared_after_forwarding(session):
    node, _ = session
    node.timing = dict(received=0, forwarded=0, tf_wait=0, expired=0)
    node.tf.lookup_transform.side_effect = TransformException('latest data at 9.9')
    MapSession.depth_cb(node, depth())
    assert node.timing['tf_error'] == 'latest data at 9.9'
    assert node.timing['tf_requested_stamp_ns'] == 10_000_000_000
    node.depth_pub.publish.assert_not_called()
    node.tf.lookup_transform.side_effect = None
    node.flush_depth()
    assert node.timing['tf_error'] is None
    node.depth_pub.publish.assert_called_once()
