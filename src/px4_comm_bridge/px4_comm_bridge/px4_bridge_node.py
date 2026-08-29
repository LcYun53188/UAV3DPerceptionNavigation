import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from .control_bridge import Px4ControlBridge
from .data_bridge import Px4DataBridge

try:
    from px4_msgs.msg import (
        OffboardControlMode,
        SensorGps,
        TrajectorySetpoint,
        VehicleAttitude,
        VehicleCommand,
        VehicleImu,
        VehicleOdometry,
        VehicleStatus,
    )
    PX4_MSGS = True
except Exception:
    OffboardControlMode = None
    SensorGps = None
    TrajectorySetpoint = None
    VehicleAttitude = None
    VehicleCommand = None
    VehicleOdometry = None
    VehicleImu = None
    VehicleStatus = None
    PX4_MSGS = False


class Px4CommBridge(Node):
    def __init__(self):
        super().__init__('px4_comm_bridge')

        # Feature switches
        self.declare_parameter('enable_data_bridge', True)
        self.declare_parameter('enable_control_bridge', True)

        # Data bridge parameters
        self.declare_parameter('px4_odometry_topic', '/px4/vehicle_odometry')
        self.declare_parameter('px4_imu_topic', '/px4/vehicle_imu')
        self.declare_parameter('px4_gps_topic', '/fmu/out/sensor_gps')
        self.declare_parameter('px4_attitude_topic', '/fmu/out/vehicle_attitude')
        self.declare_parameter('pub_odometry', '/px4/odom')
        self.declare_parameter('pub_imu', '/px4/imu')
        self.declare_parameter('pub_gps', '/gps/fix')
        self.declare_parameter('pub_attitude', '/px4/attitude')

        # Control bridge parameters
        self.declare_parameter('planner_cmd_topic', '/nav/cmd_vel')
        self.declare_parameter('planner_pose_topic', '/nav/cmd_pose')
        self.declare_parameter('planner_emergency_topic', '/nav/emergency')
        self.declare_parameter('planner_safety_topic', '/nav/safety_status')

        self.declare_parameter('fmu_offboard_mode_topic', '/fmu/in/offboard_control_mode')
        self.declare_parameter('fmu_trajectory_topic', '/fmu/in/trajectory_setpoint')
        self.declare_parameter('fmu_command_topic', '/fmu/in/vehicle_command')
        self.declare_parameter('px4_vehicle_status_topic', '/fmu/out/vehicle_status')

        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('input_velocity_frame', 'enu')
        self.declare_parameter('auto_arm', False)
        self.declare_parameter('emergency_action', 'land')
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('target_system', 1)
        self.declare_parameter('target_component', 1)

        self.data_bridge = Px4DataBridge(
            self,
            PX4_MSGS,
            VehicleOdometry,
            VehicleImu,
            SensorGps,
            VehicleAttitude,
        )
        self.control_bridge = Px4ControlBridge(
            self,
            PX4_MSGS,
            OffboardControlMode,
            TrajectorySetpoint,
            VehicleCommand,
        )

        if bool(self.get_parameter('enable_data_bridge').value):
            self.data_bridge.start()
        else:
            self.get_logger().info('PX4 data bridge disabled by parameter')

        if bool(self.get_parameter('enable_control_bridge').value):
            self.control_bridge.start()
        else:
            self.get_logger().info('PX4 control bridge disabled by parameter')

        self.get_logger().info('px4_comm_bridge started (data/control decoupled)')


def main(args=None):
    rclpy.init(args=args)
    node = Px4CommBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
