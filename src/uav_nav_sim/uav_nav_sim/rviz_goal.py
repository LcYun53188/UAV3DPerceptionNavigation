"""Convert RViz planar goals into map-frame goals at a configurable height."""
import copy
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


def fixed_height_goal(message, height):
    if not math.isfinite(height) or height <= 0:
        raise ValueError('height must be finite and greater than zero')
    if message.header.frame_id != 'map':
        raise ValueError('RViz Fixed Frame must be map')
    p, q = message.pose.position, message.pose.orientation
    if not all(math.isfinite(v) for v in (p.x, p.y, q.x, q.y, q.z, q.w)):
        raise ValueError('goal position and orientation must be finite')
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if norm < 1e-9:
        raise ValueError('goal orientation must be a nonzero quaternion')
    goal = copy.deepcopy(message)
    goal.pose.position.z = height
    for axis in 'xyzw':
        setattr(goal.pose.orientation, axis, getattr(q, axis) / norm)
    return goal


class RvizGoal(Node):
    def __init__(self):
        super().__init__('rviz_fixed_height_goal')
        self.declare_parameter('height', 1.2)
        height = self.get_parameter('height').value
        if not math.isfinite(height) or height <= 0:
            raise ValueError('height must be finite and greater than zero')
        self.add_on_set_parameters_callback(self.validate_parameters)
        self.publisher = self.create_publisher(PoseStamped, '/uav/goal', 10)
        self.subscription = self.create_subscription(
            PoseStamped, '/uav/rviz/goal_2d', self.on_goal, 10)
        self.get_logger().info(f'RViz goals enabled: map height = {height:.2f} m')

    def validate_parameters(self, parameters):
        for parameter in parameters:
            if parameter.name == 'height':
                value = parameter.value
                if not isinstance(value, float) or not math.isfinite(value) or value <= 0:
                    return SetParametersResult(
                        successful=False, reason='height must be a finite positive double')
        return SetParametersResult(successful=True)

    def on_goal(self, message):
        try:
            goal = fixed_height_goal(message, self.get_parameter('height').value)
        except ValueError as exc:
            self.get_logger().warning(f'Goal rejected: {exc}')
            return
        goal.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(goal)
        p = goal.pose.position
        self.get_logger().info(f'Navigation goal sent: ({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) m')


def main(args=None):
    rclpy.init(args=args)
    node = RvizGoal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
