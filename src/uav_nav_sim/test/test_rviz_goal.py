import math
import time

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from uav_nav_sim.rviz_goal import RvizGoal, fixed_height_goal


def goal(frame='map'):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x = 2.0
    msg.pose.position.y = -3.0
    msg.pose.orientation.w = 1.0
    return msg


def test_planar_goal_height_and_input_preserved():
    message = goal()
    converted = fixed_height_goal(message, 1.2)
    assert (converted.pose.position.x, converted.pose.position.y, converted.pose.position.z) == (2., -3., 1.2)
    assert message.pose.position.z == 0.


@pytest.mark.parametrize('height', [0., -1., math.nan, math.inf])
def test_invalid_height_rejected(height):
    with pytest.raises(ValueError):
        fixed_height_goal(goal(), height)


def test_wrong_frame_and_nonfinite_position_rejected():
    with pytest.raises(ValueError):
        fixed_height_goal(goal('odom'), 1.2)
    message = goal()
    message.pose.position.x = math.nan
    with pytest.raises(ValueError):
        fixed_height_goal(message, 1.2)


def test_ros_delivery_and_live_height_update():
    # Remap both ends so this test can never send a goal to the live simulator.
    rclpy.init(args=['--ros-args', '-r', '/uav/goal:=/test/rviz_goal/output',
                    '-r', '/uav/rviz/goal_2d:=/test/rviz_goal/input'])
    bridge = RvizGoal()
    probe = rclpy.create_node('rviz_goal_test_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(probe)
    received = []
    sub = probe.create_subscription(PoseStamped, '/test/rviz_goal/output', received.append, 10)
    pub = probe.create_publisher(PoseStamped, '/test/rviz_goal/input', 10)

    def wait_for(predicate):
        deadline = time.monotonic() + 5
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert predicate()

    try:
        wait_for(lambda: pub.get_subscription_count() > 0 and bridge.publisher.get_subscription_count() > 0)
        pub.publish(goal())
        wait_for(lambda: len(received) == 1)
        assert received[-1].pose.position.z == 1.2
        assert received[-1].header.frame_id == 'map'
        assert received[-1].header.stamp.sec > 0
        assert bridge.set_parameters([Parameter('height', value=2.2)])[0].successful
        pub.publish(goal())
        wait_for(lambda: len(received) == 2)
        assert received[-1].pose.position.z == 2.2
        assert not bridge.set_parameters([Parameter('height', value=-1.)])[0].successful
        assert bridge.get_parameter('height').value == 2.2
        pub.publish(goal('odom'))
        deadline = time.monotonic() + .3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert len(received) == 2
    finally:
        executor.shutdown()
        probe.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()
