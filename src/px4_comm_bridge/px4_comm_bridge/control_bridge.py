from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, Int8

from px4_msgs.msg import VehicleStatus, VehicleCommandAck
from rclpy.qos import qos_profile_sensor_data

from .converters import (
    fill_offboard_control_mode,
    fill_trajectory_setpoint,
    planner_twist_to_ned_velocity_and_yawspeed,
)
from .px4_state_machine import PX4StateMachine, PX4State


class Px4ControlBridge:
    def __init__(
        self,
        node,
        px4_available,
        offboard_control_mode_type,
        trajectory_setpoint_type,
        vehicle_command_type,
    ):
        self.node = node
        self.px4_available = px4_available
        self.offboard_control_mode_type = offboard_control_mode_type
        self.trajectory_setpoint_type = trajectory_setpoint_type
        self.vehicle_command_type = vehicle_command_type

        self.latest_cmd = None
        self.last_cmd_time = self.node.get_clock().now()
        self.emergency_active = False
        self.safety_level = 0

        self.offboard_mode_pub = None
        self.setpoint_pub = None
        self.vehicle_command_pub = None
        self.vehicle_status_sub = None

        self.last_px4_nav_state = 0
        self.last_px4_armed = False
        self.last_px4_offboard_active = False

        # ─────────────────────────────────────────────────────
        # W2-D8: 自动武装状态机初始化
        # ─────────────────────────────────────────────────────
        self.state_machine = PX4StateMachine(
            node, px4_available, vehicle_command_type
        )

    def start(self):
        if not self.px4_available:
            self.node.get_logger().warn('px4_msgs not available; ROS->PX4 control disabled')
            return

        self.offboard_mode_pub = self.node.create_publisher(
            self.offboard_control_mode_type,
            self.node.get_parameter('fmu_offboard_mode_topic').value,
            10,
        )
        self.setpoint_pub = self.node.create_publisher(
            self.trajectory_setpoint_type,
            self.node.get_parameter('fmu_trajectory_topic').value,
            10,
        )
        self.vehicle_command_pub = self.node.create_publisher(
            self.vehicle_command_type,
            self.node.get_parameter('fmu_command_topic').value,
            10,
        )

        self.node.create_subscription(
            TwistStamped,
            self.node.get_parameter('planner_cmd_topic').value,
            self.cmd_cb,
            10,
        )
        self.node.create_subscription(
            PoseStamped,
            self.node.get_parameter('planner_pose_topic').value,
            self.pose_cb,
            10,
        )
        self.node.create_subscription(
            Bool,
            self.node.get_parameter('planner_emergency_topic').value,
            self.em_cb,
            10,
        )
        self.node.create_subscription(
            Int8,
            self.node.get_parameter('planner_safety_topic').value,
            self.safety_cb,
            10,
        )
        self.vehicle_status_sub = self.node.create_subscription(
            VehicleStatus,
            self.node.get_parameter('px4_vehicle_status_topic').value,
            self.vehicle_status_cb,
            qos_profile_sensor_data,
        )

        self.command_ack_sub = self.node.create_subscription(
            VehicleCommandAck,
            self.node.get_parameter('px4_command_ack_topic').value,
            self.command_ack_cb, qos_profile_sensor_data,
        )

        rate = float(self.node.get_parameter('control_rate_hz').value)
        self.node.create_timer(max(0.01, 1.0 / rate), self.publish_control)
        
        # W2-D8: 设置状态机的命令发送回调
        self.state_machine.send_vehicle_command = self.send_vehicle_command
        
        self.node.get_logger().info('PX4 control bridge enabled')

    def now_us(self):
        return int(self.node.get_clock().now().nanoseconds / 1000)

    def cmd_cb(self, msg: TwistStamped):
        self.latest_cmd = msg
        self.last_cmd_time = self.node.get_clock().now()
        
        # W2-D8: 通知状态机接收到导航命令
        self.state_machine.on_navigation_command_received()
        
    def pose_cb(self, _msg: PoseStamped):
        # Reserved for future pose-based command mapping.
        return

    def em_cb(self, msg: Bool):
        if msg.data and not self.emergency_active:
            self.node.get_logger().warn('Emergency signal (Bool) received')
        self.emergency_active = bool(msg.data)
        
        # W2-D8: 通知状态机应急信号
        self.state_machine.on_emergency_signal(self.emergency_active)
        

    def safety_cb(self, msg: Int8):
        self.safety_level = int(msg.data)
        
        # W2-D8: 通知状态机安全状态
        self.state_machine.on_safety_status(self.safety_level)
        
        if self.safety_level >= 2:
            self.emergency_active = True

    def vehicle_status_cb(self, msg: VehicleStatus):
        self.last_px4_nav_state = int(msg.nav_state)
        self.last_px4_armed = int(msg.arming_state) == int(VehicleStatus.ARMING_STATE_ARMED)
        self.last_px4_offboard_active = (
            int(msg.nav_state) == int(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        )

        self.state_machine.on_px4_status_update(
            armed=self.last_px4_armed,
            offboard_active=self.last_px4_offboard_active,
            nav_state=self.last_px4_nav_state,
        )

    def command_ack_cb(self, msg: VehicleCommandAck):
        # Vehicle commands from this bridge use source system/component 1/1.
        if int(msg.target_system) != 1 or int(msg.target_component) != 1:
            return
        self.state_machine.on_command_ack(int(msg.command), int(msg.result))

    def publish_control(self):
        self.state_machine.update()
        state = self.state_machine.current_state
        if state in (PX4State.PRESTREAM, PX4State.ARM, PX4State.OFFBOARD):
            self.publish_offboard_mode()
            self.publish_halt_setpoint()
        elif state == PX4State.FLYING and self.latest_cmd is not None:
            self.publish_offboard_mode()
            self.publish_setpoint(self.latest_cmd)
        # Emergency delegates to the configured PX4 action. Terminal states
        # stop streaming so this bridge cannot reclaim manual control.

    def publish_offboard_mode(self):
        msg = self.offboard_control_mode_type()
        fill_offboard_control_mode(msg, self.now_us())
        self.offboard_mode_pub.publish(msg)

    def publish_setpoint(self, cmd_msg: TwistStamped):
        vx, vy, vz, yawspeed = planner_twist_to_ned_velocity_and_yawspeed(
            cmd_msg,
            str(self.node.get_parameter('input_velocity_frame').value),
        )
        msg = self.trajectory_setpoint_type()
        fill_trajectory_setpoint(msg, self.now_us(), vx, vy, vz, yawspeed)
        self.setpoint_pub.publish(msg)

    def publish_halt_setpoint(self):
        msg = self.trajectory_setpoint_type()
        fill_trajectory_setpoint(msg, self.now_us(), 0.0, 0.0, 0.0, 0.0)
        self.setpoint_pub.publish(msg)

    def send_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0,
        param3=0.0,
        param4=0.0,
        param5=0.0,
        param6=0.0,
        param7=0.0,
    ):
        msg = self.vehicle_command_type()
        msg.timestamp = self.now_us()
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.param3 = float(param3)
        msg.param4 = float(param4)
        msg.param5 = float(param5)
        msg.param6 = float(param6)
        msg.param7 = float(param7)
        msg.command = int(command)
        msg.target_system = int(self.node.get_parameter('target_system').value)
        msg.target_component = int(self.node.get_parameter('target_component').value)
        msg.source_system = 1
        msg.source_component = 1
        msg.confirmation = 0
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)
