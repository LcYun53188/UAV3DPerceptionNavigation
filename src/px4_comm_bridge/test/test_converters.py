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


def test_vehicle_odometry_ned_to_enu_and_body_twist():
    from px4_msgs.msg import VehicleOdometry
    from px4_comm_bridge.converters import vehicle_odometry_to_ros
    m=VehicleOdometry(timestamp=1000000,timestamp_sample=900000,pose_frame=1,velocity_frame=1,
                      position=[1.,2.,-3.],q=[1.,0.,0.,0.],velocity=[1.,0.,0.],
                      angular_velocity=[.1,.2,.3],position_variance=[1.,4.,9.],
                      orientation_variance=[.1,.2,.3],velocity_variance=[1.,4.,9.])
    r=vehicle_odometry_to_ros(m)
    assert r.header.frame_id=='odom' and r.child_frame_id=='base_link'
    assert r.header.stamp.nanosec==900000000
    assert (r.pose.pose.position.x,r.pose.pose.position.y,r.pose.pose.position.z)==(2.,1.,3.)
    assert math.isclose(r.twist.twist.linear.x,1.,abs_tol=1e-6)
    assert math.isclose(r.twist.twist.linear.y,0.,abs_tol=1e-6)
    assert math.isclose(r.pose.pose.orientation.w,math.sqrt(.5),abs_tol=1e-6)
    assert math.isclose(r.pose.pose.orientation.z,math.sqrt(.5),abs_tol=1e-6)
    assert math.isclose(r.twist.covariance[0],1.,abs_tol=1e-6)
    assert math.isclose(r.twist.covariance[7],4.,abs_tol=1e-6)
    m.pose_frame=2
    import pytest
    with pytest.raises(ValueError):vehicle_odometry_to_ros(m)
    m.pose_frame=1;m.q=[0.,0.,0.,0.]
    with pytest.raises(ValueError):vehicle_odometry_to_ros(m)
