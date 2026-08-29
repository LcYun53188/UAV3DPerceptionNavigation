"""Fake VIO odometry publisher for EKF validation without camera hardware."""

from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class FakeVioOdometry(Node):
    def __init__(self):
        super().__init__('fake_vio_odometry')

        self.declare_parameter('rate_hz', 30.0)
        self.declare_parameter('topic', '/vio/odometry')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('motion_enabled', True)
        self.declare_parameter('speed_mps', 0.15)
        self.declare_parameter('yaw_rate_rps', 0.03)

        topic = str(self.get_parameter('topic').value)
        self.odom_pub = self.create_publisher(Odometry, topic, 10)
        self.start_time = self.get_clock().now()

        rate = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(max(0.01, 1.0 / rate), self.publish_odometry)
        self.get_logger().info(f'Fake VIO odometry started on {topic}')

    def elapsed_sec(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def publish_odometry(self):
        now = self.get_clock().now().to_msg()
        t = self.elapsed_sec()

        motion_enabled = bool(self.get_parameter('motion_enabled').value)
        speed = float(self.get_parameter('speed_mps').value) if motion_enabled else 0.0
        yaw_rate = float(self.get_parameter('yaw_rate_rps').value) if motion_enabled else 0.0
        yaw = yaw_rate * t

        msg = Odometry()
        msg.header.stamp = now
        msg.header.frame_id = str(self.get_parameter('odom_frame').value)
        msg.child_frame_id = str(self.get_parameter('base_frame').value)

        msg.pose.pose.position.x = speed * t
        msg.pose.pose.position.y = 0.05 * math.sin(0.2 * t) if motion_enabled else 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.z = math.sin(yaw * 0.5)
        msg.pose.pose.orientation.w = math.cos(yaw * 0.5)

        msg.twist.twist.linear.x = speed
        msg.twist.twist.linear.y = 0.01 * math.cos(0.2 * t) if motion_enabled else 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.z = yaw_rate

        # Position and yaw are trusted more than roll/pitch, which come from
        # the fake PX4 attitude source in the dual EKF config.
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[14] = 0.05
        msg.pose.covariance[21] = 1e3
        msg.pose.covariance[28] = 1e3
        msg.pose.covariance[35] = 0.03
        msg.twist.covariance[0] = 0.05
        msg.twist.covariance[7] = 0.05
        msg.twist.covariance[14] = 0.05
        msg.twist.covariance[35] = 0.05

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FakeVioOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
