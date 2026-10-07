# px4_comm_bridge

PX4 与 ROS 2 之间的统一桥接包，负责 PX4 数据/控制双向桥接。

## 1. 架构（职责分离）

- `px4_bridge_node.py`：主装配层，仅声明参数并按开关启动子模块
- `data_bridge.py`：仅负责 `PX4 -> ROS` 数据桥接
- `control_bridge.py`：仅负责 `ROS -> PX4` Offboard 控制与应急逻辑
- `converters.py`：纯转换函数，不依赖节点状态

通过参数实现功能解耦：

- `enable_data_bridge`：开启/关闭数据桥
- `enable_control_bridge`：开启/关闭控制桥

## 2. 消息流

### 2.1 数据桥（PX4 -> ROS）

订阅：

- `px4_msgs/msg/VehicleOdometry`（默认 `/px4/vehicle_odometry`）
- `px4_msgs/msg/VehicleImu`（默认 `/px4/vehicle_imu`）

发布：

- `nav_msgs/msg/Odometry`（默认 `/px4/odom`）
- `sensor_msgs/msg/Imu`（默认 `/px4/imu`）

### 2.2 控制桥（ROS -> PX4）

订阅：

- `geometry_msgs/msg/TwistStamped`（默认 `/nav/cmd_vel`）
- `geometry_msgs/msg/PoseStamped`（默认 `/nav/cmd_pose`，当前预留）
- `std_msgs/msg/Bool`（默认 `/nav/emergency`）
- `std_msgs/msg/Int8`（默认 `/nav/safety_status`）

发布到 PX4 FMU：

- `px4_msgs/msg/OffboardControlMode`（默认 `/fmu/in/offboard_control_mode`）
- `px4_msgs/msg/TrajectorySetpoint`（默认 `/fmu/in/trajectory_setpoint`）
- `px4_msgs/msg/VehicleCommand`（默认 `/fmu/in/vehicle_command`）

控制桥能力：

- 持续 Offboard 流发布
- 命令超时保护（`cmd_timeout_sec`）
- `auto_arm`
- 应急动作 `land|rtl|disarm`
- ENU/NED 速度与 yaw rate 输入转换

## 3. 关键参数

| 参数名 | 默认值 | 说明 |
|---|---|---|
| `enable_data_bridge` | `true` | 是否启用 PX4->ROS 数据桥 |
| `enable_control_bridge` | `true` | 是否启用 ROS->PX4 控制桥 |
| `px4_odometry_topic` | `/px4/vehicle_odometry` | PX4 里程计输入 |
| `px4_imu_topic` | `/px4/vehicle_imu` | PX4 IMU 输入 |
| `pub_odometry` | `/px4/odom` | Odometry 输出 |
| `pub_imu` | `/px4/imu` | Imu 输出 |
| `planner_cmd_topic` | `/nav/cmd_vel` | 规划速度输入；`angular.z` 作为 yaw rate |
| `planner_pose_topic` | `/nav/cmd_pose` | 规划位姿输入（预留） |
| `planner_emergency_topic` | `/nav/emergency` | 应急输入 |
| `planner_safety_topic` | `/nav/safety_status` | 安全等级输入 |
| `fmu_offboard_mode_topic` | `/fmu/in/offboard_control_mode` | Offboard 模式输出 |
| `fmu_trajectory_topic` | `/fmu/in/trajectory_setpoint` | 轨迹输出 |
| `fmu_command_topic` | `/fmu/in/vehicle_command` | 命令输出 |
| `control_rate_hz` | `20.0` | 控制循环频率 |
| `input_velocity_frame` | `enu` | 输入速度/yaw rate 坐标系（`enu`/`ned`） |
| `auto_arm` | `false` | 自动解锁及模式进入的唯一授权 |
| `emergency_action` | `land` | 应急动作（`land`/`rtl`/`disarm`） |
| `cmd_timeout_sec` | `0.5` | 控制命令超时阈值 |
| `target_system` | `1` | PX4 target_system |
| `target_component` | `1` | PX4 target_component |

## 4. 构建与运行

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_comm_bridge --symlink-install
source install/setup.bash
PARAM_FILE=$(ros2 pkg prefix px4_comm_bridge)/share/px4_comm_bridge/config/px4_comm_bridge.yaml
ros2 run px4_comm_bridge px4_bridge_node --ros-args --params-file "$PARAM_FILE"
```

## 5. 快速验证

```bash
ros2 node list | grep px4_comm_bridge
ros2 topic list | grep -E "^/px4/odom$|^/px4/imu$|^/fmu/in/offboard_control_mode$|^/fmu/in/trajectory_setpoint$|^/fmu/in/vehicle_command$"
```

## 6. 与 uav_bringup 的关系

`uav_bringup/launch/nav_stack.launch.py` 已切换为启动 `px4_comm_bridge`，由本包统一承担 PX4 通讯能力。

## M0 状态机修复（2026-09-24）

唯一授权为 `auto_arm`，默认 false；旧 `sm_auto_arm` 不再读取。状态机统一使用
`cmd_timeout_sec` 和 `emergency_action`，不再存在回调直接解锁或独立应急命令路径。

流程为 `IDLE -> PRESTREAM -> ARM -> OFFBOARD -> FLYING`。启动要求收到新鲜
VehicleStatus 和导航命令，预发送零速度参考与心跳 1.5 秒；只有 VehicleStatus 确认
armed/Offboard 后才推进状态，ACK accepted 不作为状态确认。状态与 ACK 订阅使用
best-effort QoS。拒绝 ACK（临时拒绝除外）进入 FAULT；ACK 仅按命令和接收者过滤，
协议无请求 ID，不能保证区分其他发布者或之前同类命令的延迟 ACK。

重发间隔与状态期限分开，使用单调时钟：`sm_transition_timeout_sec=5.0`，
`sm_status_timeout_sec=1.0`，`sm_prestream_sec=1.5`。反馈丢失进入 FAULT 并停止
流送；命令过期或安全异常在已解锁时请求配置的应急动作，同时停止流送。
应急结束依赖解除武装反馈，LANDED 在这里仅表示已解除武装，并非触地检测。

飞行中解除武装或离开 Offboard 会锁定 MANUAL，停止心跳、设置点和模式命令。
MANUAL/FAULT/LANDED 不因新导航命令恢复，当前恢复方式为操作者确认后显式重启桥节点。
`auto_arm=false` 当前也不会主动进入 Offboard 控制已人工解锁的飞控。

此实现仍是速度桥。零速度预发送不是经过验证的制动轨迹；模式超时或进程失联后的
实际动作仍依赖 PX4 配置。固件版本、预发送持续性、DDS 联调、状态来源时间验证、
坐标原点对齐和 SITL 故障注入尚待验收，不代表 M0 或实机验收完成。

回归测试：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH=src/px4_comm_bridge ./scripts/with_venv.sh python -m pytest -q src/px4_comm_bridge/test
```

VehicleOdometry 回读现在要求明确 NED pose 和 NED/body-FRD velocity，输出 odom/ENU pose 与 base_link/FLU twist；包含姿态和协方差转换，非法帧/非有限 pose 拒绝发布。真实 W0 任务使用独立唯一网关，见 [本机飞行入口](../../simulation/px4/README.md#w0-真实飞行任务)；旧桥接的自动控制入口不由该 profile 启动。
