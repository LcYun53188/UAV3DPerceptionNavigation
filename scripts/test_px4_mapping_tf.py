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
              local_counters=(0,0,0,0,0), local_baseline=None, local_received=10.,
              localization_session='owned', localized_pub=NS(publish=lambda m:None),
              dynamic=NS(sendTransform=sent.append),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=1_000_000_000)))
    msg = VehicleOdometry(timestamp=1_000_000, timestamp_sample=1_000_000,
                         pose_frame=1, velocity_frame=1, position=[1., 2., -3.],
                         q=[1., 0., 0., 0.], velocity=[0., 0., 0.], angular_velocity=[0., 0., 0.])
    return node, msg, current, sent


def test_initialization_reset_settles_before_first_tf_then_latches(monkeypatch):
    node, msg, current, sent = fixture(monkeypatch)
    module.MappingTf.pose(node, msg)
    current[0] = 13.; node.status_received = node.local_received = 13.
    msg.reset_counter = 1
    module.MappingTf.pose(node, msg)
    current[0] = 17.; node.status_received = node.local_received = 17.
    module.MappingTf.pose(node, msg)
    assert not sent and not node.failed
    current[0] = 18.; node.status_received = node.local_received = 18.
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
    current[0] = 16.; node.status_received = node.local_received = 16.
    if condition == 'armed':
        module.MappingTf.status(node, VehicleStatus(arming_state=2))
        module.MappingTf.status(node, VehicleStatus(arming_state=1))
    elif condition == 'stale_status':
        node.status_received = 10.
    else:
        msg.timestamp_sample = 400_000
    module.MappingTf.pose(node, msg)
    assert not sent


def test_actual_ros_snapshot_serializes_numpy_shape(monkeypatch, tmp_path):
    import json
    from uav_nav_interfaces.msg import MapSnapshot
    node, _, _, _ = fixture(monkeypatch)
    node.evidence = tmp_path/'map.json'
    node.started = True
    msg = MapSnapshot(valid=True, resolution=.1, shape=[1, 1, 2],
                      distance=[.5, 0.], observed=[1, 1])
    msg.source_stamp.sec = 1
    module.MappingTf.snapshot(node, msg)
    result = json.loads(node.evidence.read_text())
    assert result['passed'] and result['shape'] == [1, 1, 2]
    assert result['observed_positive_distance_voxels'] == 1


def test_local_position_reset_and_invalid_pose_stop_immediately(monkeypatch):
    from px4_msgs.msg import VehicleLocalPosition
    node,msg,current,sent=fixture(monkeypatch)
    module.MappingTf.pose(node,msg)
    current[0]=16.;node.status_received=node.local_received=16.
    module.MappingTf.pose(node,msg)
    assert len(sent)==1
    local=VehicleLocalPosition(timestamp=1_000_000,xy_valid=True,z_valid=True,heading_reset_counter=1)
    module.MappingTf.local(node,local)
    assert node.failed and node.reason=='LOCAL_POSITION_RESET'
    module.MappingTf.pose(node,msg)
    assert len(sent)==1
