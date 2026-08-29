import math

from geometry_msgs.msg import TwistStamped

from px4_comm_bridge.converters import (
    planner_twist_to_ned_velocity,
    planner_twist_to_ned_velocity_and_yawspeed,
)


def test_planner_twist_to_ned_velocity_keeps_legacy_return_shape():
    msg = TwistStamped()
    msg.twist.linear.x = 1.0
    msg.twist.linear.y = 2.0
    msg.twist.linear.z = 3.0
    msg.twist.angular.z = 0.4

    assert planner_twist_to_ned_velocity(msg, "enu") == (2.0, 1.0, -3.0)


def test_planner_twist_to_ned_velocity_and_yawspeed_from_enu():
    msg = TwistStamped()
    msg.twist.linear.x = 1.0
    msg.twist.linear.y = 2.0
    msg.twist.linear.z = 3.0
    msg.twist.angular.z = 0.4

    vx, vy, vz, yawspeed = planner_twist_to_ned_velocity_and_yawspeed(msg, "enu")

    assert (vx, vy, vz) == (2.0, 1.0, -3.0)
    assert math.isclose(yawspeed, -0.4)


def test_planner_twist_to_ned_velocity_and_yawspeed_from_ned():
    msg = TwistStamped()
    msg.twist.linear.x = 1.0
    msg.twist.linear.y = 2.0
    msg.twist.linear.z = 3.0
    msg.twist.angular.z = 0.4

    assert planner_twist_to_ned_velocity_and_yawspeed(msg, "ned") == (
        1.0,
        2.0,
        3.0,
        0.4,
    )
