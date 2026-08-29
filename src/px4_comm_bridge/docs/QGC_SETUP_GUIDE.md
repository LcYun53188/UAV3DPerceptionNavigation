# QGroundControl 地面站配置指南

本文档说明如何在 QGroundControl (QGC) 中配置 PX4 飞控参数，使其与 `px4_comm_bridge` 正常协作。

---

## 目录

1. [整体架构](#1-整体架构)
2. [前置条件](#2-前置条件)
3. [Step 1：配置 uXRCE-DDS 通讯链路](#3-step-1配置-uxrce-dds-通讯链路)
4. [Step 2：配置 Offboard 模式参数](#4-step-2配置-offboard-模式参数)
5. [Step 3：配置安全与失效保护](#5-step-3配置安全与失效保护)
6. [Step 4：（可选）RC 遥控器映射](#6-step-4可选rc-遥控器映射)
7. [伴飞计算机端启动流程](#7-伴飞计算机端启动流程)
8. [端到端验证清单](#8-端到端验证清单)
9. [消息流速率与参数对照表](#9-消息流速率与参数对照表)
10. [常见问题排查](#10-常见问题排查)

---

## 1. 整体架构

```
┌──────────────────┐   Serial/UDP    ┌──────────────────────┐   DDS    ┌──────────────────┐
│   PX4 Autopilot  │ ◄────────────► │  MicroXRCEAgent      │ ◄─────► │   ROS 2 Network  │
│   (Flight Ctrl)  │   uXRCE-DDS    │  (Companion PC)      │         │                  │
└──────────────────┘                └──────────────────────┘         └────────┬─────────┘
                                                                             │
                                                                    ┌────────┴─────────┐
                                                                    │ px4_comm_bridge   │
                                                                    │                  │
                                                                    │ ┌──────────────┐ │
                                                                    │ │ data_bridge   │ │  PX4 → ROS
                                                                    │ │ (Odom + IMU)  │ │
                                                                    │ └──────────────┘ │
                                                                    │ ┌──────────────┐ │
                                                                    │ │control_bridge │ │  ROS → PX4
                                                                    │ │ (Offboard)    │ │
                                                                    │ └──────────────┘ │
                                                                    └──────────────────┘
```

**通讯链路**：PX4 飞控 ↔ uXRCE-DDS Agent（伴飞计算机）↔ ROS 2 DDS 网络 ↔ `px4_comm_bridge` 节点。

---

## 2. 前置条件

| 条件 | 要求 |
|---|---|
| PX4 固件版本 | **v1.14+**（需要 uXRCE-DDS 客户端支持） |
| QGroundControl | v4.2+ |
| ROS 2 版本 | Jazzy |
| `px4_msgs` 包 | 与固件版本匹配的分支 |
| 物理连接 | 飞控 TELEM2 → 伴飞计算机串口 或 以太网 |

---

## 3. Step 1：配置 uXRCE-DDS 通讯链路

打开 QGC → **车辆设置（齿轮图标）** → **参数**，搜索并设置以下参数：

### 3.1 串口连接（推荐 TELEM2）

| 参数 | 设置值 | 说明 |
|---|---|---|
| `MAV_1_CONFIG` | `0`（Disabled） | 禁用 TELEM2 上的 MAVLink，释放给 DDS |
| `UXRCE_DDS_CFG` | `TELEM2` | 指定 uXRCE-DDS 使用的端口 |
| `SER_TEL2_BAUD` | `921600` | 波特率，建议不低于 921600 |

### 3.2 以太网连接（如飞控支持）

| 参数 | 设置值 | 说明 |
|---|---|---|
| `UXRCE_DDS_CFG` | `Ethernet` | 使用以太网端口 |
| `UXRCE_DDS_PRT` | `8888`（默认） | Agent 监听的 UDP 端口 |
| `UXRCE_DDS_AG_IP` | 伴飞计算机 IP（int32 格式） | Agent 所在 IP 地址 |

> **⚠️ 修改以上参数后必须重启飞控才能生效。**

### 3.3 验证 DDS 链路

在 QGC 的 **MAVLink 控制台** 中执行：

```bash
uxrce_dds_client status
```

看到 `connected` 状态即表示 PX4 端的 DDS 客户端已正常运行。

---

## 4. Step 2：配置 Offboard 模式参数

`px4_comm_bridge` 的控制桥以 **速度控制模式** 工作，需确保 PX4 端参数与之匹配。

| 参数 | 推荐值 | 说明 |
|---|---|---|
| `COM_ARM_WO_GPS` | `1`（Allow） | 允许无 GPS 情况下解锁（室内飞行必需） |
| `EKF2_HGT_REF` | 按实际选择 | 高度参考源（室内可选 Vision/Range） |
| `EKF2_EV_CTRL` | 按实际位掩码 | 是否使用外部视觉作为 EKF 输入 |

### Offboard 心跳要求

PX4 要求在 Offboard 模式下持续接收 `OffboardControlMode` 消息，频率 **> 2 Hz**。`px4_comm_bridge` 的控制桥默认以 `control_rate_hz = 20.0` Hz 发布，满足此要求。

如果心跳中断，PX4 将退出 Offboard 模式并触发失效保护。

---

## 5. Step 3：配置安全与失效保护

这些参数决定了通讯异常时飞行器的行为，**务必认真配置**。

### 5.1 RC 丢失保护

| 参数 | 推荐值 | 说明 |
|---|---|---|
| `COM_RCL_EXCEPT` | `4`（Offboard 位） | 在 Offboard 模式下忽略 RC 丢失 |
| `NAV_RCL_ACT` | `2`（Return）或 `3`（Land） | RC 丢失时的动作（非 Offboard 模式） |
| `COM_RC_LOSS_T` | `0.5` | RC 丢失超时（秒） |

> **说明**：如果你的无人机 **不使用传统遥控器**，`COM_RCL_EXCEPT` 设为 `4` 可避免在 Offboard 模式下误触发 RC 丢失失效保护。如果同时还使用 Hold 模式，可设为 `6`（= 4 + 2）。

### 5.2 Offboard 链路丢失保护

| 参数 | 推荐值 | 说明 |
|---|---|---|
| `COM_OF_LOSS_T` | `1.0` | Offboard 链路丢失超时（秒） |
| `COM_OBL_RC_ACT` | `0`（Position）或 `4`（Land） | Offboard 丢失且有 RC 时的动作 |
| `COM_OBL_ACT` | `3`（Land） | Offboard 丢失且无 RC 时的动作 |

### 5.3 与 px4_comm_bridge 应急机制的配合

`px4_comm_bridge` 内置了自己的应急处理逻辑（见 `control_bridge.py`）：

| px4_comm_bridge 参数 | 默认值 | 说明 |
|---|---|---|
| `cmd_timeout_sec` | `0.5` | 控制命令超时阈值 |
| `emergency_action` | `land` | 应急动作（`land` / `rtl` / `disarm`） |

**设计意图**：

```
速度指令超时(0.5s)                PX4 Offboard 丢失(1.0s)
       │                                   │
       ▼                                   ▼
 px4_comm_bridge                      PX4 内部
 发送 VehicleCommand                  触发 COM_OBL_RC_ACT
 (LAND/RTL/DISARM)                   或 COM_OBL_ACT
```

- `cmd_timeout_sec`（0.5s）< `COM_OF_LOSS_T`（1.0s），确保 **先由 px4_comm_bridge 处理应急**
- 如果 px4_comm_bridge 自身也失联，PX4 的内部失效保护作为最后一道防线

---

## 6. Step 4：（可选）RC 遥控器映射

如果使用 RC 遥控器，建议配置以下参数实现手动夺回控制：

| 参数 | 说明 |
|---|---|
| `RC_MAP_OFFB_SW` | 映射一个通道用于切换 Offboard 模式开/关 |
| `COM_RC_OVERRIDE` | `1` — 允许移动遥杆时自动从 Offboard 切回 Position 模式 |
| `COM_RC_STICK_OV` | 遥杆覆盖阈值（默认 30%） |

> **⚠️ 强烈建议**：即使在全自主飞行场景下，也保留一个 RC 遥控器作为安全备份。

---

## 7. 伴飞计算机端启动流程

完成 QGC 参数配置并重启飞控后，在伴飞计算机端按以下顺序启动：

### 7.1 启动 Micro XRCE-DDS Agent

**串口连接：**

```bash
MicroXRCEAgent serial --dev /dev/ttyTHS1 -b 921600
# 根据实际硬件调整串口设备路径，常见设备：
# Jetson:  /dev/ttyTHS1 或 /dev/ttyTHS2
# RPi:     /dev/ttyAMA0
# USB转串: /dev/ttyUSB0
```

**UDP 连接：**

```bash
MicroXRCEAgent udp4 -p 8888
```

### 7.2 启动 px4_comm_bridge

```bash
# 编译（首次）
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_comm_bridge --symlink-install
source install/setup.bash

# 独立运行
PARAM_FILE=$(ros2 pkg prefix px4_comm_bridge)/share/px4_comm_bridge/config/px4_comm_bridge.yaml
ros2 run px4_comm_bridge px4_bridge_node --ros-args --params-file "$PARAM_FILE"

# 或通过 nav_stack launch 文件运行（推荐）
ros2 launch uav_bringup nav_stack.launch.py
```

---

## 8. 端到端验证清单

按顺序执行以下验证步骤：

### ✅ Step 1：检查 DDS 链路

```bash
# 在伴飞计算机上确认 Agent 已连接
# Agent 终端应显示 "Session established" 或类似日志

# 在 QGC MAVLink 控制台中
uxrce_dds_client status
# 输出应为 "running" / "connected"
```

### ✅ Step 2：检查 ROS 2 话题

```bash
# 确认 PX4 话题已出现
ros2 topic list | grep -E "^/px4/|^/fmu/"

# 期望看到（至少）：
# /px4/vehicle_odometry
# /px4/vehicle_imu
```

### ✅ Step 3：检查 px4_comm_bridge 节点

```bash
# 确认节点在运行
ros2 node list | grep px4_comm_bridge

# 确认数据桥输出
ros2 topic hz /px4/odom      # 应有数据输出
ros2 topic hz /px4/imu            # 应有数据输出
ros2 topic echo /px4/odom --once
```

### ✅ Step 4：检查控制桥（地面测试）

```bash
# 确认 Offboard 模式话题正在发布
ros2 topic hz /fmu/in/offboard_control_mode

# 发送测试速度指令（零速度悬停，注意安全！）
ros2 topic pub /nav/cmd_vel geometry_msgs/msg/TwistStamped \
  "{header: {stamp: {sec: 0}, frame_id: 'base_link'}, \
    twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" \
  --rate 10
```

### ✅ Step 5：QGC 中确认状态

- 在 QGC 主界面确认飞行模式显示为 **Offboard**
- 检查 **Vehicle Setup → Safety** 面板中失效保护设置无警告
- 在 **Analyze → MAVLink Inspector** 中确认消息流正常

---

## 9. 消息流速率与参数对照表

### 数据桥（PX4 → ROS）

| 方向 | PX4 话题 | ROS 话题 | 消息类型 | 备注 |
|---|---|---|---|---|
| 订阅 | `/px4/vehicle_odometry` | — | `px4_msgs/VehicleOdometry` | PX4 发布频率 ~50 Hz |
| 订阅 | `/px4/vehicle_imu` | — | `px4_msgs/VehicleImu` | PX4 发布频率 ~200 Hz |
| 发布 | — | `/px4/odom` | `nav_msgs/Odometry` | frame: `map` → `base_link` |
| 发布 | — | `/px4/imu` | `sensor_msgs/Imu` | frame: `imu_link` |

### 控制桥（ROS → PX4）

| 方向 | ROS 话题 | PX4 话题 | 消息类型 | 备注 |
|---|---|---|---|---|
| 订阅 | `/nav/cmd_vel` | — | `geometry_msgs/TwistStamped` | 主控制通道，`angular.z` 映射为 yaw rate |
| 订阅 | `/nav/cmd_pose` | — | `geometry_msgs/PoseStamped` | 预留，尚未实现 |
| 订阅 | `/nav/emergency` | — | `std_msgs/Bool` | `true` 触发应急 |
| 订阅 | `/nav/safety_status` | — | `std_msgs/Int8` | ≥ 2 触发应急 |
| 发布 | — | `/fmu/in/offboard_control_mode` | `px4_msgs/OffboardControlMode` | 20 Hz 心跳 |
| 发布 | — | `/fmu/in/trajectory_setpoint` | `px4_msgs/TrajectorySetpoint` | 速度 + yawspeed 模式，NED |
| 发布 | — | `/fmu/in/vehicle_command` | `px4_msgs/VehicleCommand` | ARM/LAND/RTL |

---

## 10. 常见问题排查

### Q1：QGC 中找不到 `UXRCE_DDS_CFG` 参数

- PX4 固件版本需 ≥ v1.14
- 某些 flash 受限的飞控板（如 FMUv2）可能未编译 `uxrce_dds_client` 模块，需自行编译固件并启用

### Q2：Agent 已启动但 ROS 2 看不到 PX4 话题

- 检查波特率是否匹配（飞控端 `SER_TEL2_BAUD` 与 Agent 命令行 `-b` 参数）
- 检查串口设备路径是否正确
- 确认 `MAV_1_CONFIG = 0`（该端口的 MAVLink 已禁用）
- 确认 `px4_msgs` 版本与 PX4 固件版本匹配

### Q3：数据桥有输出但控制桥发不出命令

- 确认 `enable_control_bridge: true`
- 确认有节点在向 `/nav/cmd_vel` 发布速度指令
- 查看 px4_comm_bridge 日志是否出现 `"Received first nav command"` 信息

### Q4：Offboard 模式立即退出

- 确认 OffboardControlMode 流频率 > 2 Hz（`ros2 topic hz /fmu/in/offboard_control_mode`）
- 检查 `COM_RCL_EXCEPT` 是否包含 Offboard 位（值 `4`）
- 检查 `COM_OF_LOSS_T` 是否设置合理（建议 ≥ 1.0 秒）

### Q5：飞行器在 Offboard 模式下突然着陆

- 检查 `cmd_timeout_sec`（默认 0.5s），如果速度指令发布频率不稳定可适当放宽
- 检查 px4_comm_bridge 日志是否出现 `"Command timeout detected"` 错误
- 检查是否有 `/nav/emergency` 或 `/nav/safety_status` 话题意外触发

### Q6：速度方向反了

- 检查 `input_velocity_frame` 参数，`enu` 模式下 px4_comm_bridge 会自动做 ENU→NED 转换
- 如果你的规划器直接输出 NED 速度，将参数设为 `ned`

---

## 附：QGC 参数设置速查表

以下为一次性配置清单，在 QGC 参数页面中逐一搜索并设置：

```
# ===== uXRCE-DDS 通讯（串口方式）=====
MAV_1_CONFIG      = 0           # 禁用 TELEM2 MAVLink
UXRCE_DDS_CFG     = TELEM2      # 启用 DDS 到 TELEM2
SER_TEL2_BAUD     = 921600      # 波特率

# ===== Offboard 模式 =====
COM_ARM_WO_GPS    = 1           # 允许无 GPS 解锁（室内）

# ===== 安全与失效保护 =====
COM_RCL_EXCEPT    = 4           # Offboard 模式忽略 RC 丢失
NAV_RCL_ACT       = 2           # RC 丢失动作：Return（非 Offboard 时）
COM_OF_LOSS_T     = 1.0         # Offboard 丢失超时
COM_OBL_RC_ACT    = 0           # Offboard 丢失（有 RC）：切 Position
COM_OBL_ACT       = 3           # Offboard 丢失（无 RC）：着陆

# ===== RC 安全覆盖（可选）=====
COM_RC_OVERRIDE   = 1           # 允许遥杆覆盖
COM_RC_STICK_OV   = 30          # 遥杆覆盖阈值（%）
```

> **完成设置后务必重启飞控！**
