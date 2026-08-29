from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, NavSatFix

from .converters import (
    sensor_gps_to_navsatfix,
    vehicle_attitude_to_pose,
    vehicle_imu_to_ros,
    vehicle_odometry_to_ros,
)


class Px4DataBridge:
    def __init__(self, node, px4_available, vehicle_odometry_type, vehicle_imu_type,
                 sensor_gps_type, vehicle_attitude_type):
        self.node = node
        self.px4_available = px4_available
        self.vehicle_odometry_type = vehicle_odometry_type
        self.vehicle_imu_type = vehicle_imu_type
        self.sensor_gps_type = sensor_gps_type
        self.vehicle_attitude_type = vehicle_attitude_type

        self.odom_pub = self.node.create_publisher(
            Odometry,
            self.node.get_parameter('pub_odometry').value,
            10,
        )
        self.imu_pub = self.node.create_publisher(
            Imu,
            self.node.get_parameter('pub_imu').value,
            10,
        )
        self.gps_pub = self.node.create_publisher(
            NavSatFix,
            self.node.get_parameter('pub_gps').value,
            10,
        )
        self.attitude_pub = self.node.create_publisher(
            PoseWithCovarianceStamped,
            self.node.get_parameter('pub_attitude').value,
            10,
        )

    def start(self):
        if not self.px4_available:
            self.node.get_logger().warn('px4_msgs not available; PX4->ROS subscriptions disabled')
            return

        self.node.create_subscription(
            self.vehicle_odometry_type,
            self.node.get_parameter('px4_odometry_topic').value,
            self.px4_odometry_cb,
            10,
        )
        self.node.create_subscription(
            self.vehicle_imu_type,
            self.node.get_parameter('px4_imu_topic').value,
            self.px4_imu_cb,
            10,
        )
        self.node.create_subscription(
            self.sensor_gps_type,
            self.node.get_parameter('px4_gps_topic').value,
            self.px4_gps_cb,
            10,
        )
        self.node.create_subscription(
            self.vehicle_attitude_type,
            self.node.get_parameter('px4_attitude_topic').value,
            self.px4_attitude_cb,
            10,
        )
        self.node.get_logger().info('PX4 data bridge enabled (odom + imu + gps + attitude)')

    def px4_odometry_cb(self, msg):
        self.odom_pub.publish(vehicle_odometry_to_ros(msg))

    def px4_imu_cb(self, msg):
        self.imu_pub.publish(vehicle_imu_to_ros(msg))

    def px4_gps_cb(self, msg):
        self.gps_pub.publish(sensor_gps_to_navsatfix(msg))

    def px4_attitude_cb(self, msg):
        self.attitude_pub.publish(vehicle_attitude_to_pose(msg))
