# W2-D8-D9: PX4 自动武装状态机设计

## 📋 概述

**目标**: 规范 PX4 车辆的启动、飞行、应急流程，从手动干预 → 完全自动

**设计期间**: 2026-05-17 (会议 + 代码实现)  
**实现文件**:
- `src/px4_comm_bridge/px4_comm_bridge/px4_state_machine.py` (新增, 385 行)
- `src/px4_comm_bridge/px4_comm_bridge/control_bridge.py` (修改, 状态机集成)

---

## 🎯 状态机架构

### 完整状态流

```
                  ┌─────────────┐
                  │   IDLE      │  无操作
                  └─────┬───────┘
                        │
              导航命令 + auto_arm=true
                        │
                        ▼
                  ┌─────────────┐
                  │    ARM      │  武装车辆
                  └──────┬──────┘   (发送 VEHICLE_CMD_ARM, param1=1.0)
                  (5s超时) │
          PX4确认 armed=true
                         │
                        ▼
                  ┌─────────────┐
                  │  OFFBOARD   │  进入 Offboard 模式
                  └──────┬──────┘   (发送 VEHICLE_CMD_DO_SET_MODE)
                  (5s超时) │
       PX4确认 offboard_active=true
                         │
                        ▼
                  ┌─────────────┐
                  │   FLYING    │  ◀──────┐
                  └──────┬──────┘         │
                         │                │ 正常命令流
    cmd_timeout or       │                │ heartbeat
    emergency_signal     │                │
                        ▼                │
                  ┌─────────────┐        │
       ┌─────────▶│ EMERGENCY   │        │ 
       │          └──────┬──────┘        │
       │                 │                │
       │      执行应急动作 │ (LAND/RTL/DISARM)
       │          (10s超时)              │
       │                │                │
       │                ▼                │
       │          ┌─────────────┐        │
       │          │   LANDED    │        │
       │          └──────┬──────┘        │
       │                 │                │
       │   安全恢复后等待│ 新导航命令    │
       │  新导航目标     │ → IDLE        │
       └─────────────────┘               │
         (received_navigation_cmd      │
           + 状态 = LANDED)        ─────┘
```

### 关键转移规则

| 源状态 | 触发条件 | 目标状态 | 备注 |
|--------|--------|--------|------|
| IDLE | nav_cmd + auto_arm=true | ARM | 用户点击"Go" |
| ARM | px4_armed=true | OFFBOARD | PX4 反馈武装成功 |
| ARM | timeout (5s) | EMERGENCY | 武装失败 |
| OFFBOARD | px4_offboard_active=true | FLYING | 进入 Offboard 模式 |
| OFFBOARD | timeout (5s) | EMERGENCY | 模式切换失败 |
| FLYING | emergency_active=true \| safety_level=CRITICAL | EMERGENCY | 立即响应 |
| FLYING | cmd_timeout (>10s) | EMERGENCY | 规划器掉线 |
| EMERGENCY | timeout (10s) | LANDED | 完成着陆 |
| LANDED | nav_cmd | IDLE | 重新开始 |

---

## 🔌 接口设计

### 输入信号 (订阅)

1. **导航命令** (`/nav/cmd_vel`)
   - 类型: `geometry_msgs/TwistStamped`
   - 触发: `on_navigation_command_received()`
   - 副作用: 如果 IDLE + auto_arm=true → ARM

2. **应急信号** (`/nav/emergency`, Bool)
   - 类型: `std_msgs/Bool`
   - 数据: `True` 立即触发应急
   - 触发: `on_emergency_signal(active)`

3. **多源安全评分** (`/nav/safety_status`, Int8)
   - 类型: `std_msgs/Int8`
   - 数据: `0=OK, 1=WARNING, 2=CRITICAL`
   - 触发: `on_safety_status(level)`
   - 规则: `level >= 2` → 自动转为应急

4. **PX4 状态反馈** (订阅 `/fmu/out/vehicle_status`)
   - 类型: `px4_msgs/VehicleStatus`
   - 字段: `armed_state, nav_state` 等
   - 触发: `on_px4_status_update(armed, offboard_active, nav_state)`

### 输出信号 (发布)

1. **Offboard 模式心跳** (`/fmu/in/offboard_control_mode`, 50Hz)
   - 保活信号，ControlBridge 自动流送（在 FLYING/EMERGENCY 状态）
   - 由 `publish_offboard_mode()` 发送

2. **速度设置点** (`/fmu/in/trajectory_setpoint`)
   - 实时速度命令 (FLYING 状态)
   - 停止信号 (EMERGENCY 状态)
   - 由 `publish_setpoint()` 或 `publish_halt_setpoint()` 发送

3. **车辆命令** (`/fmu/in/vehicle_command`)
   - ARM: `cmd=400, param1=1.0`
   - OFFBOARD: `cmd=176, param1=1.0, param2=6.0`
   - LAND: `cmd=21`
   - RTL: `cmd=20`
   - DISARM: `cmd=400, param1=0.0`
   - 由 `send_vehicle_command()` 发送

---

## ⚙️ 参数配置

### 状态机特定参数

```yaml
# px4_comm_bridge/config/px4_comm_bridge.yaml

# ─────────────────────────────────────────────────────
# 状态机控制参数 (W2-D8-D9)
# ─────────────────────────────────────────────────────
sm_auto_arm: true  # 自动武装开关: true=接收导航命令自动武装, false=手动控制

sm_cmd_timeout_sec: 10.0  # 命令超时阈值
                           # FLYING状态: 超过10s无导航命令 → EMERGENCY

sm_emergency_action: 'land'  # 应急动作: 'land' | 'rtl' | 'disarm'
                              # land - 原地降落
                              # rtl - 返回起点
                              # disarm - 立即解除武装(危险!)

sm_offboard_heartbeat_hz: 50.0  # Offboard 模式心跳频率
                                 # PX4要求心跳间隔 < 100ms

sm_manual_takeover_timeout_sec: 5.0  # 保留用于将来的手动接管功能
```

---

## 💾 代码分布

### 新增文件: `px4_state_machine.py` (385 行)

**类**: `PX4StateMachine`
- 状态定义: `PX4State` enum (IDLE, ARM, OFFBOARD, FLYING, EMERGENCY, LANDED)
- 输入接口: `on_*()` 方法族
  - `on_navigation_command_received()`
  - `on_emergency_signal(active)`
  - `on_safety_status(level)`
  - `on_px4_status_update(armed, offboard_active, nav_state)`
- 核心逻辑:
  - `update()` - 主循环 (20Hz, 由 ControlBridge 调用)
  - `_check_state_transitions()` - 检查状态转移条件
  - `_execute_state_actions()` - 发送相应的 PX4 命令
- 诊断接口:
  - `get_state_info()` - 返回当前状态信息 (for RViz/debugging)

### 修改文件: `control_bridge.py` (变更 ~50 行)

**关键改动**:
1. `__init__()`: 初始化 `self.state_machine = PX4StateMachine(...)`
2. `cmd_cb()`: 添加 `self.state_machine.on_navigation_command_received()`
3. `em_cb()`: 添加 `self.state_machine.on_emergency_signal()`
4. `safety_cb()`: 添加 `self.state_machine.on_safety_status()`
5. `publish_control()`: 完全重写
   - 调用 `self.state_machine.update()`
   - 根据 `sm_state` 决定发送什么信号

---

## 🎬 运行流程示例

### 场景 1: 正常起飞

```
T=0s    用户通过 Nav Stack 发送 goal_pose
            ↓
        IDLE → 接收到 /nav/cmd_vel
            ↓
        on_navigation_command_received()
            ↓
        state_machine.update()
            ↓
        [检查转移] IDLE + nav_cmd + auto_arm=true → ARM
            ↓
        [执行动作] 发送 VEHICLE_CMD_COMPONENT_ARM_DISARM (param1=1.0)
            ↓
T=1s    PX4 反馈 armed=true
            ↓
        on_px4_status_update(armed=true, ...)
            ↓
        update() → ARM + armed=true → OFFBOARD
            ↓
        [执行动作] 发送 VEHICLE_CMD_DO_SET_MODE

T=2s    PX4 反馈 offboard_active=true
            ↓
        on_px4_status_update(..., offboard_active=true, ...)
            ↓
        update() → OFFBOARD + offboard_active=true → FLYING
            ↓
        [发送控制] 流送 offboard_control_mode (50Hz) + trajectory_setpoint
            ↓
T=10s   持续接收导航命令和传感器反馈
            ↓ (应急时)

T=15s   NavSafety 检测到 TF 失败 (CRITICAL)
            ↓
        on_safety_status(2)  # CRITICAL
            ↓
        cm_state.current_state = EMERGENCY
            ↓
        update() → [执行应急动作] 发送 LAND 命令
            ↓
        发送停止速度 + offboard 心跳 (保持模式)

T=25s   EMERGENCY 状态 10s 超时
            ↓
        update() → EMERGENCY + timeout(10s) → LANDED
            ↓
        [执行登陆清理] 退出 Offboard + DISARM
```

### 场景 2: 命令超时

```
T=5s    规划器崩溃，停止发送 /nav/cmd_vel
            ↓
        last_cmd_time = T=5s
            ↓
T=15s   update() 检查
            ↓
        now - last_cmd_time = 10s > sm_cmd_timeout_sec (10s)
            ↓
        FLYING 状态下命令超时 → EMERGENCY
            ↓
        [执行应急] 发送 LAND 命令
```

---

## 🧪 测试验证 (W2-D9)

### 单元测试框架

```python
# test/test_px4_state_machine.py (待实现)

def test_idle_to_arm_transition():
    """测试: IDLE + nav_cmd → ARM"""
    sm = PX4StateMachine(mock_node, True, MockVehicleCommand)
    sm.on_navigation_command_received()
    sm.update()
    assert sm.current_state == PX4State.ARM

def test_arm_timeout():
    """测试: ARM 状态 5s 超时 → EMERGENCY"""
    sm = PX4StateMachine(mock_node, True, MockVehicleCommand)
    sm._transition_to(PX4State.ARM)
    # 模拟 5+ 秒无反馈
    assert sm.current_state == PX4State.EMERGENCY

def test_flying_emergency_signal():
    """测试: FLYING + emergency_signal=True → EMERGENCY"""
    sm = PX4StateMachine(mock_node, True, MockVehicleCommand)
    sm._transition_to(PX4State.FLYING)
    sm.on_emergency_signal(True)
    sm.update()
    assert sm.current_state == PX4State.EMERGENCY

def test_critical_safety_level():
    """测试: safety_level=2 (CRITICAL) → EMERGENCY"""
    sm = PX4StateMachine(mock_node, True, MockVehicleCommand)
    sm._transition_to(PX4State.FLYING)
    sm.on_safety_status(2)
    sm.update()
    assert sm.current_state == PX4State.EMERGENCY
```

### 集成测试

```bash
# Terminal 1: 启动完整导航栈
ros2 launch uav_bringup nav_stack.launch.py

# Terminal 2: 监控状态机
ros2 run px4_comm_bridge monitor_state  # 缺失，待实现

# Terminal 3: 测试场景
# 1. 发送导航命令 (ARM / OFFBOARD / FLYING 自动流程)
ros2 publish /nav/cmd_vel geometry_msgs/TwistStamped \
  '{header: {frame_id: "base_link"}, twist: {linear: {x: 1.0}}}'

# 2. 测试命令超时 (停止发送命令5秒，观察应急)
# (Python 脚本: 5s 后停止发送)

# 3. 测试应急信号
ros2 publish /nav/emergency std_msgs/Bool 'data: true'
```

---

## ⚠️ 已知限制 & TODO

### 当前实现局限

1. **PX4 状态反馈延迟**
   - 仅依赖 ControlBridge 缓存的旧状态
   - 需要添加 VehicleStatus 真实订阅更新

2. **着陆检测**
   - 当前: 硬编码 10s 超时后假设已着陆
   - 改进: 订阅 `/fmu/out/vehicle_status` detect alt=0 or landed flag

3. **手动接管**
   - `sm_manual_takeover_timeout_sec` 参数预留，功能未实现
   - 需要: 遥控器 RC 通道监听, 冲突解决策略

4. **故障恢复**
   - EMERGENCY → LANDED → IDLE: 一旦着陆无法自动恢复
   - 需要: 用户确认或定时器重启机制

### TODO (后续迭代)

- [ ] 添加 `/fmu/out/vehicle_status` 订阅以获取实时 PX4 状态
- [ ] 实现高度传感器着陆检测 (而非超时)
- [ ] 添加 RC 遥控器集成 (手动接管模式)
- [ ] 状态机诊断节点 (`monitor_state`)
- [ ] 完整单元测试套件
- [ ] RViz 可视化插件 (状态 + 转移图)

---

## 📊 编译验证

```
$ colcon build --packages-select px4_comm_bridge
Starting >>> px4_comm_bridge
Finished <<< px4_comm_bridge [0.63s]
Summary: 1 package finished [0.71s]
✅ PASS
```

---

## 📚 参考资料

- **PX4 Offboard Control**: https://docs.px4.io/main/en/ros/px4_ros_comm.html
- **VehicleCommand**: px4_msgs/msg/VehicleCommand.msg
- **状态机模式**: https://refactoring.guru/design-patterns/state

---

*W2-Phase1 完成: 状态机架构设计 + 代码实现*  
*W2-Phase2 待做: 集成测试 + 视觉反馈*
