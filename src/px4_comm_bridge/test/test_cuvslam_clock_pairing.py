"""Independent /clock and sensor callbacks must not retire an otherwise fresh source."""
from collections import OrderedDict
from types import SimpleNamespace as N
from unittest.mock import patch
import pytest
from px4_comm_bridge.cuvslam_pose_node import CuvslamPose
from test_cuvslam_warmup_node import pose, fixture


def clock_node():
    old=N(header=N(stamp=N(sec=10,nanosec=0)),vo_state=1)
    node=N(fault='',bound=True,tracking_topic='/sdk/status',tracking_gid='gid',tracking=old,
           tracking_receive=100.,tracking_samples=OrderedDict(),pending=OrderedDict(),
           now_ns=10_000_000_000,accepted=[])
    node.get_clock=lambda:N(now=lambda:N(nanoseconds=node.now_ns))
    node.publisher=lambda _: 'gid'
    node.trace=lambda *args,**kw:None
    node.reject=lambda reason:setattr(node,'fault',reason)
    node.waiting_for_clock=lambda message:CuvslamPose.waiting_for_clock(node,message)
    node.drain=lambda:CuvslamPose.drain(node)
    node.accept_pose=lambda message,pair,received=None:node.accepted.append((message,pair,received))
    return node


def test_72ms_clock_delivery_skew_waits_without_replacing_last_good_tracking():
    node=clock_node();old=node.tracking;message=pose(10.072)
    tracking=N(header=message.header,vo_state=1)
    node.pending[10.072]=(message,100.04)
    with patch('px4_comm_bridge.cuvslam_pose_node.time.monotonic',return_value=100.05):
        CuvslamPose.on_tracking(node,tracking,None)
    assert node.tracking is old and not node.accepted and not node.fault
    node.now_ns=10_080_000_000
    with patch('px4_comm_bridge.cuvslam_pose_node.time.monotonic',return_value=100.1):node.drain()
    assert len(node.accepted)==1 and node.accepted[0][2]==100.04
    assert node.tracking is tracking and node.tracking_receive==100.05
    assert not node.pending and not node.fault


def test_future_pair_expires_at_original_receive_deadline():
    node=clock_node();message=pose(11.)
    node.pending[11.]=(message,100.)
    node.tracking_samples[11.]=(N(header=message.header,vo_state=1),100.)
    with patch('px4_comm_bridge.cuvslam_pose_node.time.monotonic',return_value=100.201):node.drain()
    assert node.fault=='VIO_TRACKING_PAIR_STALE' and not node.accepted


def test_50ms_future_boundary_is_unchanged():
    node=clock_node()
    assert not node.waiting_for_clock(pose(10.05))
    assert node.waiting_for_clock(pose(10.050000001))


def test_accepting_a_buffered_pair_does_not_renew_receive_freshness():
    node=fixture()
    from px4_comm_bridge.source_warmup import SourceWarmup
    node.warmup=SourceWarmup(0.);node.bound=True;node.receive=99.9;node.stable_since=97.
    node.get_clock=lambda:N(now=lambda:N(nanoseconds=10_080_000_000))
    with patch('px4_comm_bridge.cuvslam_pose_node.time.monotonic',return_value=100.):
        CuvslamPose.accept_pose(node,pose(10.072),None,received=99.95)
    assert not node.fault and node.receive==99.95 and len(node.emitted)==1


def test_clock_regression_still_rejects_even_with_pending_future_data():
    node=clock_node();node.last_clock=10.1
    with pytest.raises(ValueError,match='VIO_CLOCK_RESET'):CuvslamPose.validate_tracking(node)
