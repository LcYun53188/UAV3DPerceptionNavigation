"""Fake PX4 sensor publishers for ROS-side validation without PX4 firmware."""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


class FakePx4Sensors(Node):
    def __init__(self):
        super().__init__('fake_px4_sensors')

        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('publish_gps', True)
        self.declare_parameter('publish_px4_odom', True)
        self.declare_parameter('include_gravity_in_accel', False)
        self.declare_parameter('latitude', 31.2304)
        self.declare_parameter('longitude', 121.4737)
        self.declare_parameter('altitude', 10.0)

        self.imu_pub = self.create_publisher(Imu, '/px4/imu', 10)
        self.attitude_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/px4/attitude', 10
        )
        self.gps_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.odom_pub = self.create_publisher(Odometry, '/px4/odom', 10)

        self.start_time = self.get_clock().now()
        rate = float(self.get_parameter('rate_hz').value)
        self.timer = self.create_timer(max(0.01, 1.0 / rate), self.publish_sensors)
        self.get_logger().info('Fake PX4 sensors started (attitude + imu + gps + odom)')

    def elapsed_sec(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def publish_sensors(self):
        now = self.get_clock().now().to_msg()
        t = self.elapsed_sec()

        # Small deterministic roll/pitch motion, yaw kept zero so VIO owns yaw.
        roll = 0.03 * math.sin(0.5 * t)
        pitch = 0.02 * math.cos(0.4 * t)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)

        attitude = PoseWithCovarianceStamped()
        attitude.header.stamp = now
        attitude.header.frame_id = 'odom'
        attitude.pose.pose.orientation.w = cr * cp
        attitude.pose.pose.orientation.x = sr * cp
        attitude.pose.pose.orientation.y = cr * sp
        attitude.pose.pose.orientation.z = -sr * sp
        attitude.pose.covariance[0] = 1e6
        attitude.pose.covariance[7] = 1e6
        attitude.pose.covariance[14] = 1e6
        attitude.pose.covariance[21] = 0.05
        attitude.pose.covariance[28] = 0.05
        attitude.pose.covariance[35] = 1e6
        self.attitude_pub.publish(attitude)

        imu = Imu()
        imu.header.stamp = now
        imu.header.frame_id = 'base_link'
        imu.angular_velocity.x = 0.015 * math.cos(0.5 * t)
        imu.angular_velocity.y = -0.008 * math.sin(0.4 * t)
        imu.angular_velocity.z = 0.0
        imu.linear_acceleration.x = 0.0
        imu.linear_acceleration.y = 0.0
        imu.linear_acceleration.z = (
            9.80665 if bool(self.get_parameter('include_gravity_in_accel').value) else 0.0
        )
        imu.angular_velocity_covariance[0] = 0.01
        imu.angular_velocity_covariance[4] = 0.01
        imu.angular_velocity_covariance[8] = 0.01
        imu.linear_acceleration_covariance[0] = 0.1
        imu.linear_acceleration_covariance[4] = 0.1
        imu.linear_acceleration_covariance[8] = 0.1
        self.imu_pub.publish(imu)

        if bool(self.get_parameter('publish_gps').value):
            gps = NavSatFix()
            gps.header.stamp = now
            gps.header.frame_id = 'gps_link'
            gps.status.status = NavSatStatus.STATUS_FIX
            gps.status.service = NavSatStatus.SERVICE_GPS
            gps.latitude = float(self.get_parameter('latitude').value)
            gps.longitude = float(self.get_parameter('longitude').value)
            gps.altitude = float(self.get_parameter('altitude').value)
            gps.position_covariance = [
                1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 2.0,
            ]
            gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.gps_pub.publish(gps)

        if bool(self.get_parameter('publish_px4_odom').value):
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.orientation = attitude.pose.pose.orientation
            self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = FakePx4Sensors()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
