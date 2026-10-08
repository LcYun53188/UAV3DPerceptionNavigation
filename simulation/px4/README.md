# 本机 PX4 SITL 基础环境

当前实现：PX4 v1.16.2、Agent v2.4.3、对应 px4_msgs 的独立构建，以及未解锁 x500 的 DDS/clock/QGC 冒烟验证。全部运行于本机，Jetson 延期。下方 W0 入口已扩展真实任务、Offboard 控制、坐标转换与已知区域；VIO/感知避障未接入。

```bash
# 首次下载锁定源码、SITL 所需子模块并构建；不构建 CUDA 导航栈
./scripts/build_px4_sim.sh --jobs 4
# 仅准备源码并核对消息定义
./scripts/build_px4_sim.sh --prepare-only
# 校验已有源码与 Agent 已解析依赖是否发生漂移
./scripts/with_venv.sh python scripts/prepare_px4_sim.py --check --check-external
# 编译后的独立 ROS/Agent 运行环境
./scripts/with_px4_sim.sh python scripts/run_px4_sitl_smoke.py --duration 30
```

源码/构建/安装均在 `.deps/`。PX4 构建用隔离的 Python 3.11 环境及 `requirements/px4-sim.txt`；ROS Jazzy 消息生成和运行用工作区已有 Python 3.12 环境。构建时清除继承的 PYTHONPATH，避免 Python 扩展 ABI 混用。Agent 安装到本工作区，不需要 sudo 或全局 ldconfig。

现有 `src/px4_msgs` 和 `install_uav` 保留；SITL 用 `.deps/px4_msgs` 与 `.deps/px4-msgs-install`，必须使用 `with_px4_sim.sh` 选择该 overlay。该包的 v1.16.2 tag 对应 commit `392e831c1f659429ca83902e66820d7094591410`。`check_px4_interfaces.py` 按固件 dds_topics.yaml 检查导出消息及其嵌套类型的声明顺序/字段/常量；注释和空白不参与比较。

构建脚本拒绝不符合版本锁或有 tracked 修改的根 checkout。PX4 仅初始化当前 SITL 所需子模块，NuttX、Gazebo Classic 等不初始化。Agent superbuild 的底层 Fast-CDR/Fast-DDS/spdlog 会解析上游分支；已记录本次实际 commit 和构建生成的 tracked diff hash，并在构建末尾及运行前核对，漂移会拒绝使用。全新构建若解析到更新的依赖，将失败并要求显式重验版本锁，不自动认可新依赖；这仍有首次下载时的上游分支可用性依赖。

冒烟运行器使用 instance 7、system_id 8、ROS domain 78、XRCE UDP 8898、GCS UDP 14550、PX4 GCS 本地端口 18577，Gazebo partition 每次为唯一 UUID。DDS namespace 是 `/px4_7`，VehicleStatus topic 是 `/px4_7/fmu/out/vehicle_status_v1`。运行前检查端口与 PX4 实例锁，并通过工作区互斥锁防止本工具重复启动。已有占用时退出，不终止其他会话。

一个 supervisor 分别管理 Gazebo server、standalone PX4、Agent、clock bridge 和 QGC，每个组件属于本次新建进程组。Gazebo 只由 supervisor 启动，PX4 使用 `PX4_GZ_STANDALONE=1`。结束或失败只清理这些组。QGC 使用独立 XDG 配置/缓存、关闭串口自动连接，offscreen 模式连接模拟飞机；不会改变用户现有 QGC 配置，也没有人工 GUI 验收结论。

PX4 采用 `UXRCE_DDS_SYNCT=0`；ROS `/clock` 从同一 Gazebo 单向桥接。检查状态/里程计/位置/着陆样本、接收年龄、源时间与 clock 差、唯一发布者、未解锁与已着陆，并从 QGC 日志核对 system_id 8 的识别事件。所有等待期限用单调时钟。该短测试没有实现 CLOCK_FAULT latch、暂停恢复或低 RTF 正式验收，也没有验证 ENU/NED 转换。

当前机器缺少 GStreamer development 依赖，默认的摄像头串流插件未构建。项目的 `server_control.config` 从锁定 PX4 的 Gazebo server 配置派生，仅删除 GstCameraSystem 加载项；其他物理与传感器系统保留。它仅用于 x500 基础状态验证，摄像头/深度/VIO 能力不计通过；后续 S5 应使用完整且单独验证的感知配置。

日志、manifest（命令/PID/版本/场景配置 hash）、rootfs 参数、ULog、观测和清理结果保存到 `.cache/simulation/sitl/<run_id>/`。回归包括源码 pin 拒绝覆盖、消息字段重排与嵌套类型缺失检测。基础链路报告见 [本机 SITL 验证](../../docs/validation/simulation/2026-10-07-px4-sitl-build/REPORT.md)。

## W0 真实飞行任务

已提供 `known_region_control` 后端：真实重力/旋翼动力学、PX4 EKF 回读与唯一
Offboard 网关，支持起飞、三维定点导航、定时悬停、返回记录起点、原生降落。
默认任务还会确认实际停稳、触地和解除武装。此后端使用显式已知空旷区域，
感知避障、VIO 和 BT 集成属于后续工作。

```bash
# PX4/Agent 已构建后，构建任务接口、桥接和执行后端
./scripts/build_px4_flight.sh
# 默认：2 m 起飞 -> 两航点 -> 30 s 悬停 -> 返航 -> 降落
./scripts/sim.sh px4-flight
# 运动中暂停、停稳、恢复新子任务，再完成整条任务
./scripts/sim.sh px4-flight --flight-scenario pause-resume
# 运动中取消，确认停稳交接；保持最多 30 s 后转原生降落
./scripts/sim.sh px4-flight --flight-scenario cancel
# 暂停/恢复 Gazebo 时钟；旧任务故障锁存，PX4 丢失 Offboard 后降落
./scripts/sim.sh px4-flight --flight-scenario clock-fault
# 自定义任务（仍接受相同 W0、身份、定位、期限和落地门槛）
./scripts/sim.sh px4-flight --mission-file simulation/missions/W0_flight_sequence.json
```

任务文件的 `NAVIGATE.offset_enu` 表示相对本次起点的东、北、上偏移（米）；
客户端将它经已记录的非 identity 对齐变成 map 目标。也可直接给 `target_map`。
RETURN 在当前飞行高度返回起点 XY，采用区域内轨迹；此处不使用 PX4 默认
可能升至高空的 RTL 模式。LAND 采用 PX4 原生 AUTO_LAND。任务必须从地面开始，
首个步骤为 TAKEOFF，最后为 LAND；高度 0.8–3 m、悬停 2–60 s、最多 20 步，
客户端总预算 210 s（服务端允许 20–240 s）。超预算不会无限续飞。

配置及边界见 [W0 安全区域](../safe_regions/W0.json)，默认任务见
[任务文件](../missions/W0_flight_sequence.json)。区域包含场景/机型 hash、已知支持面、
机体半径 0.5 m、跟踪余量 0.3 m、制动余量 1.2 m；整个路径/停止目标必须位于
膨胀后允许体积。路径使用停止端点的五次曲线，参考速度/加速度/jerk 上限分别
为 0.6 m/s、0.5 m/s²、0.6 m/s³。这是冻结的本机模型配置，不是硬件参数。

`/uav/px4/execute_mission` 使用 ExecuteMission（backend=`PX4_KNOWN_REGION`，
mission_type=`FLIGHT_SEQUENCE`）；暂停/恢复在 `/uav/px4/pause`、`/uav/px4/resume`。
TaskStatus / ControlStatus / 回读里程计分别位于 `/uav/px4/task_status`、
`/uav/px4/control_status`、`/uav/px4/odometry`。执行器发布 map→odom 静态对齐及
odom→base_link；PX4 世界 NED 速度转换为 ROS child-frame FLU twist。root UUID、
子任务 UUID、世代与递增事件序号分层记录；恢复重新生成参考，不恢复旧时间轴。

运行器分配本次授权 nonce，并要求固定 domain 78、实例 7/system 8、唯一分区与
唯一控制/clock 发布者。仅收到规划速度不触发该入口解锁，旧 bridge 不在此启动链
中运行。源码/场景不匹配、缺状态、越界或缺授权均拒绝任务。ACK 不代替实际
armed/Offboard/landed 状态。降落提交后拒绝取消，不重新请求 Offboard。

飞行配置显式使用 `EKF2_MAG_TYPE=6`（初始化磁航向，之后依靠惯性/GNSS），
避免默认 Automatic 在离地约 1.5 m 后的磁航向重新对齐使固定坐标会话失效。
航向有效性、误差、reset 计数门槛继续生效；出现 reset 时撤销旧任务，不自动续飞。
该估计器配置只用于本次模型基线，不代表已实现外部 VIO 定位。

本配置位置接收/源年龄上限为 0.1 s；VehicleStatus 约 2 Hz，采用 0.75 s 上限；
land_detected 约 1 Hz，采用 1.2 s 上限。它们与 S1 只读观察器的 0.5 s 候选不同，
在 W0 profile 中独立声明。clock 无进展超过 0.5 s 锁存故障；暂停时位置新鲜度可先
触发任务 ABORTED，随后 clock fault 仍锁存。恢复 clock 不重启旧输出。
`COM_OF_LOSS_T=0.5`、`COM_OBL_RC_ACT=4`（Land）、`COM_RC_IN_MODE=4` 和
`COM_DISARM_LAND=2` 仅覆盖本次 SITL 的 rootfs 参数，并写入 manifest。

运行器检查真实 ROS Action 结果，保存飞控状态、控制命令、独立原生 Gazebo
Pose_V 真值、参考/速度和故障诊断；真值只用于验收，不注入控制定位。QGC 默认
offscreen，终端输出阶段；结束后只清理本次进程组。详情及已测限制见
[真实飞行验证报告](../../docs/validation/simulation/2026-10-07-px4-flight/REPORT.md)。

## BT 执行与进展租约

构建入口现包含 `uav_bt`。运行 `./scripts/sim.sh px4-flight --bt` 经 BT.CPP XML
及真实 ROS Action 调用同一飞行后端；可加 `--flight-scenario pause-resume` 或 `cancel`。
`--flight-scenario runner-exit` / `runner-stall` 仅可配合 `--bt`，分别注入 Runner
强制退出和 tick 停滞。握手阶段最多 5 s 且禁止解锁；首个有效 tick 后冻结 0.5 s
进展期限。失效后退役航线、制动确认、保持最多 30 s，再原生降落。
原生降落不被 Runner 消失打断；细节见 [BT 包说明](../../src/uav_bt/README.md)。

BT 首批 XML 包装整条 ExecuteMission，尚未将各飞行步骤拆成独立 BT Action；
EGO/algorithm 任务树仍待实现。验证见 [BT/PX4 报告](../../docs/validation/simulation/2026-10-08-bt-px4/REPORT.md)。

逐步骤 BT 与可视化运行：`./scripts/sim.sh px4-flight --bt --ui`；协议与范围见 [逐步骤 BT](../../docs/PX4_STEP_BT.md)。

## 深度相机参考链路

目标硬件是 **OAK-D Pro W**，暂在本机开发。PX4 v1.16.2 自带的
`x500_depth` 实际包含 `OakD-Lite` 模型：本入口只验证参考传感器数据链路，
不宣称 Pro W 的广角视场、双目基线、内参、IMU 或安装外参已经匹配。
实机标定必须从对应 Pro W 设备读取，不能沿用这里的数值。

```bash
./scripts/sim.sh px4-depth --ui --duration 45
```

该模式使用独立 domain 78、随机 Gazebo partition 和 instance 7；若 QGC 已占用
14550，先保存当前状态并关闭已有 QGC。结束后自动清理本次进程，可重新打开实机 QGC。
隔离 QGC 配置禁用 USB 自动连接；本模式没有 FlightServer、解锁或 setpoint 发布者。
`--flight`、`--bt`、自定义任务和飞行故障场景不能与此模式组合。

Gazebo 实际深度话题 `/depth_camera` 和标定话题 `/camera_info` 分别桥接到
`/px4_depth/image`、`/px4_depth/camera_info`。深度为 640×480、32FC1，
参考模型水平 FOV 1.274 rad、clip 0.2–19.1 m。审计检查布局/大小端、有限深度、
标定矩阵、非空且一致的 frame、时间戳递增、数据与仿真时钟新鲜度、唯一发布者；
同时要求 DDS/clock/QGC、定位有效、全程 DISARMED 且着地。
证据写入 `.cache/simulation/sitl/<run_id>/depth-camera.json`、`observation.json`
和含模型/依赖 hash 的 `manifest.json`。没有 CameraInfo 即失败，桥接节点存在不算通过。

本阶段不发布相机 TF，不融合 nvblox，不建立地图对齐，也不运行 VIO。
下一步先明确 Pro W 的仿真参考配置与光学外参，再接入隔离的 nvblox 地图会话和
已有 PlanningContext；这些完成前不授权 EGO 曲线控制 PX4。

本批实测与失败证据见 [深度相机验证报告](../../docs/validation/simulation/2026-10-08-px4-depth/REPORT.md)。
