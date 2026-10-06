from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from rclpy.clock import ClockType
from rclpy.time import Time
from uav_nav_interfaces.msg import MapSnapshot
from nav_msgs.msg import Odometry
from uav_nav_sim.executor import Executor


@pytest.mark.parametrize('gate', ['map_stamp', 'map_receive', 'odom_stamp', 'odom_receive'])
def test_stale_gate_is_identified_without_relaxing_readiness(monkeypatch, gate):
    monkeypatch.setattr('uav_nav_sim.executor.time.monotonic', lambda: 100.)
    node = NS(map=MapSnapshot(version=8), odom=Odometry(), grid=object(),
              map_wall=100., odom_wall=100., timeout=2.,
              get_clock=lambda: NS(now=lambda: Time(seconds=100, clock_type=ClockType.ROS_TIME)))
    node.map.header.stamp.sec = node.odom.header.stamp.sec = 100
    assert Executor.ready(node)
    name, kind = gate.split('_')
    age = 3 if name == 'map' else 1
    if kind == 'stamp':
        getattr(node, name).header.stamp.sec -= age
    else:
        setattr(node, name+'_wall', 100.-age)
    assert not Executor.ready(node)
    diagnostics = Executor.health_diagnostics(node)
    assert diagnostics['failed_checks'] == [gate]
    assert diagnostics['map_version'] == 8


def test_stop_logs_health_before_clearing_trajectory(monkeypatch):
    monkeypatch.setattr('uav_nav_sim.executor.time.monotonic', lambda: 100.)
    logger = Mock()
    node = NS(map=MapSnapshot(), odom=Odometry(), grid=object(), timeout=2.,
              map_wall=100., odom_wall=99., curve=object(), trajectory=object(), pending=object(),
              navigation=None, command=Mock(), event=Mock(),
              get_logger=lambda: logger,
              get_clock=lambda: NS(now=lambda: Time(seconds=100, clock_type=ClockType.ROS_TIME)))
    node.map.header.stamp.sec = node.odom.header.stamp.sec = 100
    Executor.stop(node, 'STALE_MAP_OR_ODOMETRY')
    assert 'odom_receive' in logger.warning.call_args_list[0].args[0]
    assert node.curve is None and node.pending is None and node.state == 'HOLD'
    assert node.event.publish.call_args.args[0].data == 'STALE_MAP_OR_ODOMETRY'
