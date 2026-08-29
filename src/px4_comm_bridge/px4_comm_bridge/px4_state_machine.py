"""
PX4 自动武装状态机 (W2-D8-D9)

状态流：
  IDLE → ARM → OFFBOARD → FLYING
                    ↓        ↓
                  EMERGENCY  (命令超时/安全信号)
                    ↓
                  [着陆/返航/解除]
                    ↓
                  LANDED

心跳机制:
  - 持续流送 offboard_control_mode (50Hz) 保持模式
  - 10s无命令 → EMERGENCY
  - 任何CRITICAL安全信号 → EMERGENCY

安全状态机集成:
  - NavSafety.LEVEL_CRITICAL → 立即着陆
  - 多源故障检测 (TF/EKF/PointCloud/PX4)
"""

from enum import Enum, auto
import time


class PX4State(Enum):
    """车辆状态定义"""
    IDLE = auto()          # 未初始化
    ARM = auto()           # 武装中
    OFFBOARD = auto()      # 进入offboard模式中
    FLYING = auto()        # 正常飞行
    EMERGENCY = auto()     # 应急处理中
    LANDED = auto()        # 已着陆/解除


class PX4StateMachine:
    """
    PX4 自动武装与应急处理状态机
    
    与 ControlBridge 的关键关联:
    - 订阅: /nav/cmd_vel (导航命令)
    - 订阅: /nav/emergency (立即应急)
    - 订阅: /nav/safety_status (多源安全评分)
    - 发布: /fmu/in/offboard_control_mode (心跳)
    - 发布: /fmu/in/trajectory_setpoint (速度指令)
    - 发布: /fmu/in/vehicle_command (武装/模式/着陆命令)
    """

    def __init__(self, node, px4_available, vehicle_command_type):
        """初始化状态机
        
        Args:
            node: ROS2 Node 对象
            px4_available: px4_msgs 是否可用
            vehicle_command_type: VehicleCommand 消息类型
        """
        self.node = node
        self.px4_available = px4_available
        self.vehicle_command_type = vehicle_command_type

        # ─────────────────────────────────────────────────────
        # 状态变量
        # ─────────────────────────────────────────────────────
        self.current_state = PX4State.IDLE
        self.previous_state = None
        self.state_enter_time = self.node.get_clock().now()

        # 外部输入信号
        self.received_navigation_cmd = False
        self.emergency_signal_active = False
        self.safety_level = 0  # 0=OK, 1=WARN, 2=CRITICAL
        self.last_cmd_time = self.node.get_clock().now()

        # PX4 反馈状态 (从 VehicleStatus 订阅)
        self.px4_armed = False
        self.px4_offboard_active = False
        self.px4_nav_state = 0  # PX4 internal state

        # ─────────────────────────────────────────────────────
        # 参数声明
        # ─────────────────────────────────────────────────────
        self.node.declare_parameter('sm_auto_arm', True)  # 自动武装开关
        self.node.declare_parameter('sm_cmd_timeout_sec', 10.0)  # 命令超时，触发应急
        self.node.declare_parameter('sm_emergency_action', 'land')  # 应急动作: land/rtl/disarm
        self.node.declare_parameter('sm_offboard_heartbeat_hz', 50.0)  # offboard模式心跳频率
        self.node.declare_parameter('sm_manual_takeover_timeout_sec', 5.0)  # 手动接管超时

        # ─────────────────────────────────────────────────────
        # 类型定义 (PX4 VehicleCommand::VEHICLE_CMD_*)
        # ─────────────────────────────────────────────────────
        self.CMD_ARM = 400  # VEHICLE_CMD_COMPONENT_ARM_DISARM
        self.CMD_LAND = 21  # VEHICLE_CMD_NAV_LAND
        self.CMD_RTL = 20   # VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        self.CMD_SET_MODE = 176  # VEHICLE_CMD_DO_SET_MODE

        # 每个状态的调用计数 (用于日志和诊断)
        self.state_entry_count = {}
        for state in PX4State:
            self.state_entry_count[state] = 0

    def now_us(self):
        """获取当前时间（微秒）"""
        return int(self.node.get_clock().now().nanoseconds / 1000)

    def send_vehicle_command(self, command, param1=0.0, param2=0.0, param3=0.0, 
                            param4=0.0, param5=0.0, param6=0.0, param7=0.0):
        """发送 PX4 车辆命令（通过子类实现）"""
        # 此方法应在 ControlBridge 中调用
        raise NotImplementedError('Subclass must implement send_vehicle_command')

    # ─────────────────────────────────────────────────────
    # ✓ 外部输入接口
    # ─────────────────────────────────────────────────────

    def on_navigation_command_received(self):
        """导航规划器发来命令"""
        self.received_navigation_cmd = True
        self.last_cmd_time = self.node.get_clock().now()

    def on_emergency_signal(self, active: bool):
        """应急信号 (硬件E-stop或NavSafety.CRITICAL)"""
        if active and not self.emergency_signal_active:
            self.node.get_logger().error(f'[SM] EMERGENCY SIGNAL ACTIVATED')
        self.emergency_signal_active = bool(active)

    def on_safety_status(self, level: int):
        """多源安全评分 (0=OK, 1=WARN, 2=CRITICAL)"""
        self.safety_level = int(level)
        if level >= 2:
            self.on_emergency_signal(True)

    def on_px4_status_update(self, armed: bool, offboard_active: bool, nav_state: int):
        """从 VehicleStatus 消息更新 PX4 反馈状态"""
        self.px4_armed = bool(armed)
        self.px4_offboard_active = bool(offboard_active)
        self.px4_nav_state = int(nav_state)

    # ─────────────────────────────────────────────────────
    # ✓ 状态转移逻辑
    # ─────────────────────────────────────────────────────

    def _transition_to(self, new_state: PX4State):
        """状态转移处理"""
        if new_state == self.current_state:
            return

        self.previous_state = self.current_state
        self.current_state = new_state
        self.state_enter_time = self.node.get_clock().now()
        self.state_entry_count[new_state] = self.state_entry_count.get(new_state, 0) + 1

        self.node.get_logger().info(
            f'[SM] State transition: {self.previous_state.name} → {new_state.name} '
            f'(entry #{self.state_entry_count[new_state]})'
        )

    def _check_state_transitions(self):
        """主状态转移检查器"""

        # ─ IDLE 状态 ─
        if self.current_state == PX4State.IDLE:
            if self.received_navigation_cmd and not self.emergency_signal_active:
                auto_arm = bool(self.node.get_parameter('sm_auto_arm').value)
                if auto_arm:
                    self._transition_to(PX4State.ARM)
                else:
                    self.node.get_logger().warn('[SM] Navigation cmd received but sm_auto_arm=false')

        # ─ ARM 状态 ─
        elif self.current_state == PX4State.ARM:
            if self.px4_armed:
                self._transition_to(PX4State.OFFBOARD)
            elif self._state_timeout(timeout_sec=5.0):
                self.node.get_logger().error('[SM] ARM timeout (>5s), aborting')
                self._transition_to(PX4State.EMERGENCY)

        # ─ OFFBOARD 状态 ─
        elif self.current_state == PX4State.OFFBOARD:
            if self.px4_offboard_active:
                self._transition_to(PX4State.FLYING)
            elif self._state_timeout(timeout_sec=5.0):
                self.node.get_logger().error('[SM] OFFBOARD mode entry timeout (>5s), aborting')
                self._transition_to(PX4State.EMERGENCY)

        # ─ FLYING 状态 ─
        elif self.current_state == PX4State.FLYING:
            # 检测命令超时
            cmd_timeout_sec = float(self.node.get_parameter('sm_cmd_timeout_sec').value)
            now = self.node.get_clock().now()
            time_since_cmd = (now - self.last_cmd_time).nanoseconds / 1e9

            if time_since_cmd > cmd_timeout_sec:
                self.node.get_logger().error(
                    f'[SM] Command timeout (>{cmd_timeout_sec}s), triggering EMERGENCY'
                )
                self._transition_to(PX4State.EMERGENCY)

            # 检测安全信号
            if self.emergency_signal_active:
                self._transition_to(PX4State.EMERGENCY)

        # ─ EMERGENCY 状态 ─
        elif self.current_state == PX4State.EMERGENCY:
            # 等待着陆完成（通过高度检测或时间判断）
            # 当前简化：5秒后假设已着陆
            if self._state_timeout(timeout_sec=10.0):
                self._transition_to(PX4State.LANDED)

        # ─ LANDED 状态 ─
        elif self.current_state == PX4State.LANDED:
            # 只有按下新的导航目标才能重新起飞
            if self.received_navigation_cmd:
                self.received_navigation_cmd = False
                self._transition_to(PX4State.IDLE)

    def _state_timeout(self, timeout_sec: float) -> bool:
        """检查状态是否超时"""
        now = self.node.get_clock().now()
        elapsed = (now - self.state_enter_time).nanoseconds / 1e9
        return elapsed > timeout_sec

    # ─────────────────────────────────────────────────────
    # ✓ 状态动作处理
    # ─────────────────────────────────────────────────────

    def _execute_state_actions(self):
        """根据当前状态执行相应动作"""

        if self.current_state == PX4State.IDLE:
            # 空闲：不发送任何指令
            pass

        elif self.current_state == PX4State.ARM:
            # 武装动作：发送 ARM 命令
            if self._state_timeout(timeout_sec=0.5):
                self.node.get_logger().info('[SM] Sending ARM command')
                # Command: VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0 (ARM)
                self.send_vehicle_command(self.CMD_ARM, param1=1.0)
                # 重置超时计时器
                self.state_enter_time = self.node.get_clock().now()

        elif self.current_state == PX4State.OFFBOARD:
            # Offboard 入场：发送 Offboard 模式命令
            if self._state_timeout(timeout_sec=0.5):
                self.node.get_logger().info('[SM] Entering Offboard mode')
                # Command: VEHICLE_CMD_DO_SET_MODE, param1=1.0 (custom mode), param2=6.0 (OFFBOARD)
                self.send_vehicle_command(self.CMD_SET_MODE, param1=1.0, param2=6.0)
                # 重置超时计时器
                self.state_enter_time = self.node.get_clock().now()

        elif self.current_state == PX4State.FLYING:
            # 飞行状态：心跳streaming（通过 ControlBridge.publish_control）
            # 这里main loop会自动流送 offboard_control_mode + trajectory_setpoint
            pass

        elif self.current_state == PX4State.EMERGENCY:
            # 应急动作
            if self._state_timeout(timeout_sec=0.5):
                action = str(self.node.get_parameter('sm_emergency_action').value).lower()
                if action == 'rtl':
                    self.node.get_logger().error('[SM] EMERGENCY: Send RTL command')
                    self.send_vehicle_command(self.CMD_RTL)
                elif action == 'disarm':
                    self.node.get_logger().error('[SM] EMERGENCY: Send DISARM command')
                    self.send_vehicle_command(self.CMD_ARM, param1=0.0)
                else:
                    self.node.get_logger().error('[SM] EMERGENCY: Send LAND command')
                    self.send_vehicle_command(self.CMD_LAND)
                # 重置超时计时器
                self.state_enter_time = self.node.get_clock().now()

        elif self.current_state == PX4State.LANDED:
            # 已着陆：解除 Offboard 模式、解除武装准备
            if self._state_timeout(timeout_sec=1.0):
                if self.px4_offboard_active:
                    self.node.get_logger().info('[SM] Exiting Offboard mode')
                    self.send_vehicle_command(self.CMD_SET_MODE, param1=1.0, param2=0.0)
                if self.px4_armed:
                    self.node.get_logger().info('[SM] Sending DISARM command')
                    self.send_vehicle_command(self.CMD_ARM, param1=0.0)

    # ─────────────────────────────────────────────────────
    # ✓ 主更新循环
    # ─────────────────────────────────────────────────────

    def update(self):
        """每个控制周期调用一次（20Hz）"""
        # 1. 检查状态转移条件
        self._check_state_transitions()

        # 2. 执行当前状态的动作
        self._execute_state_actions()

    # ─────────────────────────────────────────────────────
    # ✓ 诊断接口
    # ─────────────────────────────────────────────────────

    def get_state_info(self) -> dict:
        """返回当前状态诊断信息"""
        return {
            'current_state': self.current_state.name,
            'safety_level': self.safety_level,
            'emergency_active': self.emergency_signal_active,
            'px4_armed': self.px4_armed,
            'px4_offboard_active': self.px4_offboard_active,
            'received_nav_cmd': self.received_navigation_cmd,
            'time_in_state_sec': (self.node.get_clock().now() - self.state_enter_time).nanoseconds / 1e9,
        }
