"""Initialization cannot authorize or bias the post-warmup VIO source."""
from types import SimpleNamespace
from unittest.mock import patch
from geometry_msgs.msg import PoseWithCovarianceStamped
from px4_comm_bridge.cuvslam_pose_node import CuvslamPose
from px4_comm_bridge.source_warmup import SourceWarmup
from px4_comm_bridge.vio_input import SourceContinuity


def pose(t, x=0.):
    message = PoseWithCovarianceStamped()
    message.header.frame_id = 'odom'
    message.header.stamp.sec = int(t)
    message.header.stamp.nanosec = round((t - int(t)) * 1e9)
    message.pose.pose.orientation.w = 1.
    message.pose.pose.position.x = x
    for i in range(6):
        message.pose.covariance[i * 7] = .01
    return message


def fixture():
    node = SimpleNamespace(contract='cuvslam15_right_tangent_base_link_v1', fault='', bound=False, receive=None, sample=None,
        stable_since=None, warmup=SourceWarmup(20.), continuity=SourceContinuity(),
        quality_policy='strict', emitted=[], reason='', pose_topic='/sdk/pose')
    node.validate_tracking = lambda *args: None
    node.publisher = lambda topic: 'gid'
    node.trace = lambda *args, **kwargs: None
    node.publish_status = lambda: None
    node.reject = lambda reason: setattr(node, 'fault', reason)
    node.pose_pub = SimpleNamespace(publish=node.emitted.append)
    return node


def feed(node, t, x=0.):
    message = pose(t, x)
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec))
    with patch('px4_comm_bridge.cuvslam_pose_node.time.monotonic', return_value=t):
        CuvslamPose.accept_pose(node, message, None)


def test_startup_motion_excluded_then_full_healthy_window_required():
    node = fixture()
    feed(node, 10., 1.)
    feed(node, 29.96, 2.)
    assert not node.bound and not node.emitted and node.sample is None
    assert node.continuity.last_stamp is None and node.reason == 'VIO_WARMING_UP'
    for i in range(50):
        feed(node, 30. + i * .04)
    assert not node.bound and not node.emitted
    feed(node, 32.)
    assert node.bound and not node.fault and len(node.emitted) == 1
    assert node.emitted[0] == pose(32.)


def test_post_binding_jump_still_retires_without_publishing_new_pose():
    node = fixture()
    feed(node, 10.)
    for i in range(51):
        feed(node, 30. + i * .04)
    feed(node, 32.04, 1.)
    assert node.fault == 'VIO_POSE_DISCONTINUITY'
    assert len(node.emitted) == 1
    feed(node, 32.08)
    assert len(node.emitted) == 1
