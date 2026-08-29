from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool, Int8

from px4_msgs.msg import VehicleStatus

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
        self.offboard_engaged = False
        self.armed = False
        self.emergency_sent = False

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
            10,
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
        
        if not self.offboard_engaged:
            self.node.get_logger().info('Received first nav command, enabling offboard streaming')
            self.offboard_engaged = True

        if bool(self.node.get_parameter('auto_arm').value) and not self.armed:
            self.send_vehicle_command(self.vehicle_command_type.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.armed = True

    def pose_cb(self, _msg: PoseStamped):
        # Reserved for future pose-based command mapping.
        return

    def em_cb(self, msg: Bool):
        if msg.data and not self.emergency_active:
            self.node.get_logger().warn('Emergency signal (Bool) received')
        self.emergency_active = bool(msg.data)
        
        # W2-D8: 通知状态机应急信号
        self.state_machine.on_emergency_signal(self.emergency_active)
        
        self._check_and_trigger_emergency()

    def safety_cb(self, msg: Int8):
        self.safety_level = int(msg.data)
        
        # W2-D8: 通知状态机安全状态
        self.state_machine.on_safety_status(self.safety_level)
        
        if self.safety_level >= 2:
            self.emergency_active = True
        self._check_and_trigger_emergency()

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

    def _check_and_trigger_emergency(self):
        if self.emergency_active and not self.emergency_sent:
            self.node.get_logger().error('Safety action triggered!')
            self._send_emergency_action()
            self.emergency_sent = True
        elif not self.emergency_active:
            self.emergency_sent = False

    def publish_control(self):
        """
        W2-D8: 根据状态机驱动的流程发送控制信号
        
        旧流程 (简单): IDLE → 收到 cmd → 发送 offboard_control + setpoint
        新流程 (状态机): IDLE → ARM → OFFBOARD → FLYING → [EMERGENCY/LANDED]
        """
        
        # ─────────────────────────────────────────────────────
        # 1. 执行状态机 (状态转移 + 动作处理)
        # ─────────────────────────────────────────────────────
        self.state_machine.update()
        
        # ─────────────────────────────────────────────────────
        # 2. 根据状态机状态发送响应的控制信号
        # ─────────────────────────────────────────────────────
        sm_state = self.state_machine.current_state
        
        if sm_state == PX4State.IDLE or sm_state == PX4State.ARM or sm_state == PX4State.OFFBOARD:
            # 这些状态不发送控制信号（状态机自己发送 VEHICLE_COMMAND）
            return
        
        elif sm_state == PX4State.FLYING:
            # 飞行状态：发送心跳和设置点
            if self.latest_cmd is None:
                return
            
            # 心跳流送：持续发送 offboard_control_mode（50Hz）
            self.publish_offboard_mode()
            
            # 发送速度设置点
            self.publish_setpoint(self.latest_cmd)
        
        elif sm_state == PX4State.EMERGENCY:
            # 应急状态：心跳 + 停止速度
            self.publish_offboard_mode()
            self.publish_halt_setpoint()
        
        elif sm_state == PX4State.LANDED:
            # 已着陆：逐步退出 offboard 和解除武装
            # 这些由状态机通过 send_vehicle_command 处理
            pass

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

    def _send_emergency_action(self):
        action = str(self.node.get_parameter('emergency_action').value).lower()
        if action == 'rtl':
            self.node.get_logger().info('Sending RTL command')
            self.send_vehicle_command(self.vehicle_command_type.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)
        elif action == 'disarm':
            self.node.get_logger().info('Sending DISARM command')
            self.send_vehicle_command(self.vehicle_command_type.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        else:
            self.node.get_logger().info('Sending LAND command')
            self.send_vehicle_command(self.vehicle_command_type.VEHICLE_CMD_NAV_LAND)

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
