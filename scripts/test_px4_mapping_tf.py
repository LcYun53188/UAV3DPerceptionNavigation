"""Disarmed mapping must settle initialization resets and retire later resets."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
sys.path.insert(0, str(Path(__file__).resolve().parent))
import px4_mapping_tf as module
from px4_msgs.msg import VehicleOdometry, VehicleStatus
import pytest


def fixture(monkeypatch):
    current = [10.]
    monkeypatch.setattr(module.time, 'monotonic', lambda: current[0])
    sent = []
    node = NS(allowed=True, failed=False, status_received=10., reset=None,
              reset_stable_since=None, started=False, reason='SETTLING',
              dynamic=NS(sendTransform=sent.append),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)))
    msg = VehicleOdometry(timestamp=1_000_000, timestamp_sample=1_000_000,
                         pose_frame=1, velocity_frame=1, position=[1., 2., -3.],
                         q=[1., 0., 0., 0.], velocity=[0., 0., 0.], angular_velocity=[0., 0., 0.])
    return node, msg, current, sent


def test_initialization_reset_settles_before_first_tf_then_latches(monkeypatch):
    node, msg, current, sent = fixture(monkeypatch)
    module.MappingTf.pose(node, msg)
    current[0] = 13.; node.status_received = 13.
    msg.reset_counter = 1
    module.MappingTf.pose(node, msg)
    current[0] = 17.; node.status_received = 17.
    module.MappingTf.pose(node, msg)
    assert not sent and not node.failed
    current[0] = 18.; node.status_received = 18.
    module.MappingTf.pose(node, msg)
    assert len(sent) == 1 and node.started
    assert sent[0].header.stamp.sec == 1
    assert sent[0].header.frame_id == 'odom' and sent[0].child_frame_id == 'base_link'
    msg.reset_counter = 2
    module.MappingTf.pose(node, msg)
    assert node.failed and node.reason == 'ODOMETRY_RESET' and len(sent) == 1
    module.MappingTf.status(node, VehicleStatus(arming_state=1))
    module.MappingTf.pose(node, msg)
    assert not node.allowed and len(sent) == 1


@pytest.mark.parametrize('condition', ['armed', 'stale_status', 'stale_pose'])
def test_no_tf_after_invalid_sources(monkeypatch, condition):
    node, msg, current, sent = fixture(monkeypatch)
    module.MappingTf.pose(node, msg)
    current[0] = 16.; node.status_received = 16.
    if condition == 'armed':
        module.MappingTf.status(node, VehicleStatus(arming_state=2))
        module.MappingTf.status(node, VehicleStatus(arming_state=1))
    elif condition == 'stale_status':
        node.status_received = 10.
    else:
        msg.timestamp_sample = 400_000
    module.MappingTf.pose(node, msg)
    assert not sent
