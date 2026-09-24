# PX4 + Micro XRCE-DDS + ROS 2 串口通信接入指南

本文档说明如何在本项目中使用 Micro XRCE-DDS 的串口方式接入 PX4，并配合 `px4_comm_bridge` 运行。

## 1. 目标架构

```text
PX4 Autopilot (TELEM2)
      │ serial (TX/RX/GND)
      ▼
USB-TTL Serial Adapter
      │ USB
      ▼
Companion Computer (Jetson / x86 / Raspberry Pi)
      │
      ▼
Micro XRCE-DDS Agent
      │ DDS
      ▼
ROS 2 DDS Network
      │
      ▼
px4_comm_bridge
```

## 2. 硬件连接方式

推荐方式：`PX4 TELEM2 -> USB-TTL 串口模块 -> 伴飞计算机 USB`

```text
PX4 TELEM2 TX  ---->  USB-TTL RX
PX4 TELEM2 RX  ---->  USB-TTL TX
PX4 TELEM2 GND ---->  USB-TTL GND
```

### 说明

- TX/RX 必须交叉连接
- GND 必须连接
- 逻辑电平需要匹配（通常 3.3V / 5V 兼容模块）
- 伴飞计算机端通常会识别为：
  - `/dev/ttyUSB0`
  - `/dev/ttyACM0`
  - `/dev/ttyTHS1`

## 3. PX4 参数配置

建议在 QGroundControl 中设置：

```text
MAV_1_CONFIG  = 0
UXRCE_DDS_CFG = TELEM2
SER_TEL2_BAUD = 921600
```

注意：

- `MAV_1_CONFIG = 0` 表示关闭 TELEM2 上的 MAVLink 复用
- `UXRCE_DDS_CFG = TELEM2` 表示用 TELEM2 作为 uXRCE-DDS 通道
- 修改后必须重启飞控才能生效

## 4. 伴飞计算机端启动 Micro XRCE-DDS Agent

推荐方式：

```bash
MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600
```

本项目提供了启动脚本：

```bash
./scripts/run_px4_microxrce.sh
```

也可以覆盖端口和波特率：

```bash
PX4_SERIAL_PORT=/dev/ttyTHS1 ./scripts/run_px4_microxrce.sh
./scripts/run_px4_microxrce.sh --port /dev/ttyUSB0 --baud 921600
```

## 5. 启动 ROS 2 侧桥接

在 ROS 2 环境中启动：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_comm_bridge --symlink-install
source install_uav/setup.bash
PARAM_FILE=$(ros2 pkg prefix px4_comm_bridge)/share/px4_comm_bridge/config/px4_comm_bridge.yaml
ros2 run px4_comm_bridge px4_bridge_node --ros-args --params-file "$PARAM_FILE"
```

也可以通过项目启动入口：

```bash
ros2 launch uav_bringup nav_stack.launch.py
```

## 6. 验证方法

### 6.1 检查 Agent 是否已连接

```bash
ros2 topic list | grep -E '^/px4/|^/fmu/'
```

如果正常，应看到类似：

- `/px4/vehicle_odometry`
- `/px4/vehicle_imu`
- `/fmu/in/offboard_control_mode`

### 6.2 检查桥接节点

```bash
ros2 node list | grep px4_comm_bridge
ros2 topic hz /px4/odom
ros2 topic hz /px4/imu
```

### 6.3 检查控制桥

```bash
ros2 topic hz /fmu/in/offboard_control_mode
```

控制桥工作正常时，Offboard 控制模式消息应持续发布。

## 7. 本项目中已接入的关键点

- PX4 桥接包：`src/px4_comm_bridge`
- 配置文件：`src/px4_comm_bridge/config/px4_comm_bridge.yaml`
- 启动入口：`src/uav_bringup/launch/nav_stack.launch.py`
- 运行脚本：`./scripts/run_px4_microxrce.sh`

## 8. 常见问题

### Q1: /dev/ttyUSB0 不存在

- 检查 USB-TTL 模块是否插好
- 检查 `ls /dev/ttyUSB* /dev/ttyACM*`

### Q2: Agent 启动成功但 ROS 2 无 PX4 话题

- 检查 PX4 参数是否已写入并重启
- 检查 TELEM2 是否被禁用为 MAVLink
- 检查端口路径和波特率是否一致

### Q3: Offboard 模式立即退出

- 确认 `/fmu/in/offboard_control_mode` 发布频率 > 2 Hz
- 检查 `COM_RCL_EXCEPT` 与 `COM_OF_LOSS_T`

## 9. 参考文档

- `src/px4_comm_bridge/docs/QGC_SETUP_GUIDE.md`
- `src/px4_comm_bridge/README.md`
