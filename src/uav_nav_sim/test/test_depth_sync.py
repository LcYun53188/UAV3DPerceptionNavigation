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
              last_depth_stamp=None, tf=Mock(), info_pub=Mock(), depth_pub=Mock(),
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
