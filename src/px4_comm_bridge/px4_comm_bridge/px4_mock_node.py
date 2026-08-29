"""Mock PX4 node for validation without real firmware.

This node emulates the minimum PX4 topics needed to validate the ROS-side
navigation stack:
- subscribes to /fmu/in/vehicle_command
- publishes /fmu/out/vehicle_status
- optionally listens to offboard heartbeat / setpoint topics for logging

It does not simulate flight dynamics. It only provides feedback that lets the
PX4 control bridge and state machine advance through ARM, OFFBOARD, FLYING,
and emergency flows.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
)


class Px4MockNode(Node):
    def __init__(self):
        super().__init__('px4_mock_node')

        self.declare_parameter('status_rate_hz', 10.0)
        self.declare_parameter('land_disarm_delay_sec', 2.0)
        self.declare_parameter('verbose_logging', True)

        self.status_pub = self.create_publisher(VehicleStatus, '/fmu/out/vehicle_status', 10)
        self.command_sub = self.create_subscription(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            self.vehicle_command_cb,
            10,
        )
        self.offboard_sub = self.create_subscription(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            self.offboard_cb,
            10,
        )
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.setpoint_cb,
            10,
        )

        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
        self.nav_state_user_intention = self.nav_state
        self.last_offboard_time = self.get_clock().now()
        self.last_setpoint_time = self.get_clock().now()
        self.land_request_time = None
        self.latest_command = None

        rate = float(self.get_parameter('status_rate_hz').value)
        self.timer = self.create_timer(max(0.05, 1.0 / rate), self.publish_status)
        self.get_logger().info('PX4 mock node started (no firmware required)')

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def vehicle_command_cb(self, msg: VehicleCommand):
        self.latest_command = msg

        if int(msg.command) == int(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM):
            if float(msg.param1) >= 0.5:
                self.arming_state = VehicleStatus.ARMING_STATE_ARMED
                self.get_logger().info('Mock PX4: ARM accepted')
            else:
                self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
                self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
                self.get_logger().info('Mock PX4: DISARM accepted')
            return

        if int(msg.command) == int(VehicleCommand.VEHICLE_CMD_DO_SET_MODE):
            if int(msg.param2) == 6:
                if self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
                    self.nav_state = VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    self.nav_state_user_intention = self.nav_state
                    self.get_logger().info('Mock PX4: OFFBOARD accepted')
                else:
                    self.get_logger().warn('Mock PX4: OFFBOARD rejected because vehicle is not armed')
            return

        if int(msg.command) == int(VehicleCommand.VEHICLE_CMD_NAV_LAND):
            self.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_LAND
            self.land_request_time = self.get_clock().now()
            self.get_logger().info('Mock PX4: LAND accepted')
            return

        if int(msg.command) == int(VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH):
            self.nav_state = VehicleStatus.NAVIGATION_STATE_AUTO_RTL
            self.nav_state_user_intention = self.nav_state
            self.get_logger().info('Mock PX4: RTL accepted')
            return

    def offboard_cb(self, _msg: OffboardControlMode):
        self.last_offboard_time = self.get_clock().now()

    def setpoint_cb(self, _msg: TrajectorySetpoint):
        self.last_setpoint_time = self.get_clock().now()

    def _auto_complete_landing(self):
        if self.nav_state != VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            return
        if self.land_request_time is None:
            return

        delay_sec = float(self.get_parameter('land_disarm_delay_sec').value)
        elapsed = (self.get_clock().now() - self.land_request_time).nanoseconds / 1e9
        if elapsed >= delay_sec:
            self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
            self.nav_state = VehicleStatus.NAVIGATION_STATE_MANUAL
            self.nav_state_user_intention = self.nav_state
            self.land_request_time = None
            self.get_logger().info('Mock PX4: landing completed, disarmed')

    def publish_status(self):
        self._auto_complete_landing()

        msg = VehicleStatus()
        msg.timestamp = self.now_us()
        msg.armed_time = self.now_us() if self.arming_state == VehicleStatus.ARMING_STATE_ARMED else 0
        msg.takeoff_time = 0
        msg.arming_state = self.arming_state
        msg.latest_arming_reason = VehicleStatus.ARM_DISARM_REASON_COMMAND_EXTERNAL
        msg.latest_disarming_reason = VehicleStatus.ARM_DISARM_REASON_COMMAND_EXTERNAL
        msg.nav_state_timestamp = self.now_us()
        msg.nav_state_user_intention = self.nav_state_user_intention
        msg.nav_state = self.nav_state
        msg.executor_in_charge = 0
        msg.valid_nav_states_mask = 0xFFFFFFFF
        msg.can_set_nav_states_mask = 0xFFFFFFFF
        msg.failure_detector_status = VehicleStatus.FAILURE_NONE
        msg.hil_state = VehicleStatus.HIL_STATE_OFF
        msg.vehicle_type = VehicleStatus.VEHICLE_TYPE_ROTARY_WING
        msg.failsafe = self.nav_state in (
            VehicleStatus.NAVIGATION_STATE_AUTO_RTL,
            VehicleStatus.NAVIGATION_STATE_AUTO_LAND,
        )
        msg.failsafe_and_user_took_over = False
        msg.failsafe_defer_state = VehicleStatus.FAILSAFE_DEFER_STATE_DISABLED
        msg.gcs_connection_lost = False
        msg.gcs_connection_lost_counter = 0
        msg.high_latency_data_link_lost = False
        msg.is_vtol = False
        msg.is_vtol_tailsitter = False
        msg.in_transition_mode = False
        msg.in_transition_to_fw = False
        msg.system_type = 2
        msg.system_id = 1
        msg.component_id = 1
        msg.safety_button_available = False
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Px4MockNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
