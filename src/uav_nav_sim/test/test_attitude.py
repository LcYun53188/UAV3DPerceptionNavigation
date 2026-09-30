from types import SimpleNamespace as NS
from unittest.mock import Mock
import time

import numpy as np
import pytest
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.time import Time
from rclpy.clock import ClockType

from uav_nav_sim.attitude import level_body_rates
from uav_nav_sim.executor import Executor


@pytest.mark.parametrize('yaw_rate', [-.6, 0., .6])
def test_tilt_decays_even_during_yaw_scan(yaw_rate):
    rotation = Rotation.from_euler('xyz', [.35, -.7, 1.2])
    initial_tilt = np.linalg.norm(rotation.as_euler('xyz')[:2])
    dt = .002
    for _ in range(2500):
        rotation = rotation * Rotation.from_rotvec(level_body_rates(rotation, yaw_rate)*dt)
        assert np.linalg.norm(rotation.as_euler('xyz')[:2]) <= initial_tilt + 1e-6
    assert np.max(np.abs(rotation.as_euler('xyz')[:2])) < .001


def test_level_body_yaws_without_roll_or_pitch():
    rotation = Rotation.from_euler('z', 1.5)
    assert np.allclose(level_body_rates(rotation, .6), [0., 0., .6])


@pytest.fixture
def holding():
    odom = Odometry()
    odom.header.stamp.sec = 10
    q = Rotation.from_euler('xyz', [.3, -.5, .8]).as_quat()
    odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w = q
    return NS(odom=odom, odom_wall=time.monotonic(), command=Mock(),
              get_clock=lambda: NS(now=lambda: Time(seconds=10.1, clock_type=ClockType.ROS_TIME)))


def test_terminal_hold_levels_without_translation_or_map(holding):
    Executor.publish_hold(holding, Twist())
    command = holding.command.publish.call_args.args[0]
    assert command.linear.x == command.linear.y == command.linear.z == 0.
    q = holding.odom.pose.pose.orientation
    rotation = Rotation.from_quat([q.x,q.y,q.z,q.w])
    rate = np.array([command.angular.x, command.angular.y, command.angular.z])
    next_rotation = rotation * Rotation.from_rotvec(rate*.01)
    assert np.all(np.abs(next_rotation.as_euler('xyz')[:2]) < np.abs(rotation.as_euler('xyz')[:2]))


@pytest.mark.parametrize('fault', ['wall_stale', 'stamp_stale', 'missing', 'nan', 'zero'])
def test_hold_with_bad_odometry_sends_zero(holding, fault):
    if fault == 'wall_stale':
        holding.odom_wall -= 1.
    elif fault == 'stamp_stale':
        holding.odom.header.stamp.sec = 1
    elif fault == 'missing':
        holding.odom = None
    elif fault == 'nan':
        holding.odom.pose.pose.orientation.w = float('nan')
    else:
        q = holding.odom.pose.pose.orientation
        q.x = q.y = q.z = q.w = 0.
    command = Twist()
    command.angular.z = .6
    Executor.publish_hold(holding, command)
    assert holding.command.publish.call_args.args[0] == Twist()
