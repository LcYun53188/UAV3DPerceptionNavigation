# 无人机导航工作区（uav_nav_ws）

ROS 2 无人机导航实验项目，包含 OAK-D Pro W / MID360 感知、VIO / LIO、三维 EKF、EGO + nvblox 三维导航和 PX4 通信接口。
当前已实现 **Gazebo 中的深度建图、定向探索、地图保存与加载、三维轨迹规划、速度模型执行和 RViz 选点导航**。
算法仿真使用 Gazebo 真值定位和简化速度模型；另有独立 PX4 v1.16.2 SITL 已知区域飞行后端，支持起飞、航点、悬停、返航和降落。感知导航与 PX4 的融合、VIO 飞行及实机飞行仍待验证。

当前优先 **完整 BT 任务 → PX4 导航／避障集成**，使用官方 SITL GNSS／惯性 EKF 定位进行功能开发；VIO 稳定性单独优化与验收。相机默认固定下偏 **5°**。本轮无界面完整 BT 飞行回归通过，见 [基线回归](docs/validation/simulation/2026-10-09-task-first-baseline/REPORT.md)。完整任务实现不代表 VIO 或避障已验收，见 [集成路线](docs/PX4_INTEGRATION_ROADMAP.md)。

新增显式 EGO 导航／返航执行分支：网关复检已绑定曲线并转换到 PX4 local，地图／会话／控制权变化撤销旧曲线。115 项回归、真实规划输出准入及缺少规划源时 SITL 解锁前拒绝已通过；同机深度地图与实际曲线飞行尚未验收，见 [执行前置报告](docs/validation/simulation/2026-10-09-ego-px4-execution/REPORT.md)。

新增室内仓库场景（墙、门洞、立柱、货架及纹理），可通过 `px4-vio-sensors --scene warehouse` 验证实际双目／IMU VIO；仓库 0.8 m 初始 VIO 飞行闭环已通过，完整高度任务与避障尚待验收。入口和门槛见 [仓库场景说明](docs/WAREHOUSE_SIMULATION.md)。

仓库 VIO 已通过同低负载配置 120 秒运动验证（RMSE 2.60 cm，最大 3.78 cm），同机 120 秒融合在显式 0.8×／NVIDIA 无界面／图像深度 1／EKF 延迟缓冲 160 ms 配置下连续两轮通过（READY 约 114.02 s）。默认／带 UI 配置仍有失败，仓库 0.8 m BT VIO 起飞／悬停／降落连续两轮通过，1.5 m 悬停仍失败，导航尚未执行。运动证据见 [最新运动报告](docs/validation/simulation/2026-10-09-warehouse-low-load-motion/REPORT.md)，融合证据见 [最新融合报告](docs/validation/simulation/2026-10-09-warehouse-queue-latency/REPORT.md)。

新增无周期网格地面纹理与 30° 下俯对照：两轮 120 秒运动测试通过，30° 相对 15° 的位置 RMSE 从 3.10 cm 降至 1.29 cm；但 30° 同机地面长时融合失败，SDK 原始协方差持续接近 1.0 导致源撤销。新配置尚不允许飞行，见 [纹理与下俯对照](docs/validation/simulation/2026-10-09-vio-pitch-texture/REPORT.md)。

新增四窗口 VIO 飞行诊断（Gazebo、QGC、RViz、只读双目／任务监视）：两轮 1.5 m UI 任务复现源新鲜度失败并自动降落、锁定。飞行画面存在分布广的候选角点，但尚未取得 SDK 跟踪内点证据，不能认定画面不足或问题已解决。运行与证据见 [UI 画面诊断报告](docs/validation/simulation/2026-10-09-vio-flight-ui/REPORT.md)。

新增 480×300 双目负载优化、协方差坏样本拒绝及 FlightServer 仅位姿 VIO 准入。最终 0.8× 的 120 秒未解锁融合一轮通过，同配置仍有失败；真实悬停尚未验收。见 [最新优化报告](docs/validation/simulation/2026-10-09-vio-optimization/REPORT.md)。

本机 PX4/BT 入口：首次执行 `./scripts/build_px4_sim.sh --jobs 4` 和 `./scripts/build_px4_flight.sh`，随后运行 `./scripts/sim.sh px4-flight --bt --ui`，显示 Gazebo/QGC 并执行逐步骤飞行 BT。任务完成后确认着陆解除武装并自动清理本次会话。详见 [PX4 运行说明](simulation/px4/README.md) 与 [BT 生命周期](src/uav_bt/README.md)。

真实 VIO 传感器前置验证：构建 `./scripts/build_vio_node.sh` 后运行 `./scripts/sim.sh px4-vio-sensors --ui --duration 35`，检查 Gazebo 双目／IMU 经实际 cuVSLAM 的静止跟踪。全程未解锁，不发布外部视觉或飞行控制；参考模型尚非 OAK-D Pro W 标定模型，也尚未完成 VIO 悬停。见 [VIO 验证](docs/VIO_HOVER.md)。

运行 `./scripts/sim.sh px4-vio-sensors --normalize --reset-source --ui --duration 40` 可审计 SDK 位姿协方差归一化与实际 reset 后旧源撤销。短时验证已通过，但 90 秒检查中 SDK 协方差超限，源已锁存停止输出；尚未达到 VIO 飞行准入条件。见 [位姿与 reset 记录](docs/validation/simulation/2026-10-08-vio-pose/REPORT.md)。

新增独立载台运动验证：`./scripts/build_vio_motion.sh` 后运行 `./scripts/sim.sh px4-vio-sensors --normalize --motion --scene layered --duration 120`。实际双目/IMU VIO 已在多深度场景完成三轴移动与转向检查，真值只供误差审计，PX4 保持未解锁；尚未完成 VIO 飞行。对照结果见 [运动报告](docs/validation/simulation/2026-10-09-vio-motion/REPORT.md)。

显式仅位姿融合已增加固定初始化对齐和独立审计：`./scripts/sim.sh px4-vision-audit --vision-fusion-profile aligned_pose_v1 --duration 35`。合成位姿已通过实际 PX4 位置/高度/航向融合与停更检查，速度为 NaN；默认四类融合门控保留；实际 SDK 源的受管入口见下文。见 [位姿融合报告](docs/validation/simulation/2026-10-09-vio-pose-fusion/REPORT.md)。

实际 VIO→PX4 未解锁融合：`./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene layered --real-time-factor .8 --duration 120`。同一架 x500 的双目/IMU 经 cuVSLAM、固定对齐进入实际 EKF，2326 个样本、三类融合及停止源后的失效检查通过。此结果使用 0.8 倍目标仿真速度；实时长时试验仍出现超时，不能视为实时飞行或 VIO 悬停通过。见 [实际融合报告](docs/validation/simulation/2026-10-09-real-vio-fusion/REPORT.md)。 最新新鲜度交接修复、2329 样本回归及实时延迟对照见 [时序报告](docs/validation/simulation/2026-10-09-vio-timing/REPORT.md)。 后续 [SDK 分段诊断](docs/validation/simulation/2026-10-09-vio-sdk-timing/REPORT.md) 复现同步器单位问题；本轮四个整体检查仍失败，不代表稳定性通过。 最新 [单位与时钟修复](docs/validation/simulation/2026-10-09-sync-clock-fix/REPORT.md) 已通过 reset、运动和短时融合，120 秒融合仍未通过。

同机地图与 EGO 规划源入口：`./scripts/sim.sh px4-depth-plan --duration 40`。已验证实际深度地图绑定定位会话／六个重置计数，并受管启动 EGO shadow 图；地面飞行包络未通过，尚未执行同机轨迹飞行。见 [规划源绑定报告](docs/validation/simulation/2026-10-10-depth-planning-sources/REPORT.md)。

同机 5° 深度建图入口：`./scripts/sim.sh px4-depth-map --duration 40`。该入口受管启动官方 x500＋理想 RGBD 参考相机、只读 PX4 EKF TF、nvblox 和地图会话，全程未解锁；它验证实际深度形成 ESDF，尚未授权 EGO 飞行，也不是 OAK-D Pro W 标定或 VIO 验收。见 [同机深度建图报告](docs/validation/simulation/2026-10-09-px4-depth-mapping/REPORT.md)。

相机实机型号为 **OAK-D Pro W**。`./scripts/sim.sh px4-depth --ui` 单独验收 PX4 深度传感器到 ROS 的数据链路，使用上游 OakD-Lite 参考模型，不能作为 Pro W 的视场、标定或 VIO 验证。它全程保持未解锁，不与 W0 飞行模式混用。详见 [相机仿真说明](simulation/px4/README.md#深度相机参考链路)。

OAK-D Pro W 室内悬停已开始实现：新增 VIO 输入适配、源连续性检查及 PX4 实际融合健康门控，缺失定位/融合证据时拒绝起飞。独立 PX4 遥测构建已通过未解锁的合成外部视觉实际 EKF 融合与停更审计。当前尚未完成真实 VIO 悬停闭环；监视、审计入口与剩余条件见 [VIO 悬停说明](docs/VIO_HOVER.md)。

已新增带定位/地图会话绑定的 EGO 影子规划：非 identity 坐标转换、真实 EGO 绕障规划与独立曲线复检已接入，输出尚不控制 PX4。详见 [规划上下文](docs/PLANNING_CONTEXT.md)。

PX4 里程计新鲜度、定位重置锁存及故障回归见 [定位输入门控](docs/PX4_LOCALIZATION_GATES.md)。

DM-FC01 USB 台架只接收诊断与无遥控器供电条件说明见 [USB 台架检查](docs/PX4_USB_BENCH.md)。

算法导航提供带反馈和确认停稳取消的 `/uav/algorithm/navigate` Action，并接入父 MissionServer、两航点 BT、暂停恢复与进展租约。构建、运行和验证边界见 [任务说明](docs/ALGORITHM_MISSIONS.md) 与 [导航 Action 说明](docs/NAVIGATION_ACTION.md)。

## 选择运行方式

| 使用方式 | 入口 / 参数 | 是否更新地图 | 启动后做什么 |
| --- | --- | --- | --- |
| 仅查看场景与传感器 | `run_uav_obstacle_course.sh` | 不启动 nvblox | 查看深度、点云和里程计 |
| 在线建图并朝目标探索 | `uav_ego_nvblox.launch.py mode:=mapping` | 持续积分深度 | 局部初始化后，在 RViz 发送目标 |
| 离线扫描并保存地图 | 同上，`mode:=mapping` | 扫描时积分深度 | 运行完整扫描脚本，生成地图包 |
| 加载已有地图导航 | `uav_ego_nvblox.launch.py mode:=localization` | 不积分新深度 | 加载兼容地图，等待有效后发送目标 |

导航 launch 的 `mode` 只接受 `mapping` 和 `localization`；在线探索与离线扫描使用同一个建图模式。下方各流程择一运行，不要同时启动多套仿真。`uav_obstacle_course.sdf` 与导航用的 `uav_ego_expanded.sdf` 是不同场景，不能混用地图和扫描布局。

## 运行前准备

按 [安装指南](docs/INSTALLATION.md) 准备 `.venv`、ROS 2 Jazzy、Gazebo Harmonic / ros_gz 和 NVIDIA CUDA 环境。首次使用或更新算法接口后，在项目根目录构建：

```bash
./scripts/build_algorithm_sim.sh
```

脚本默认 CUDA 13.2、GPU 架构 89（本机 RTX 4070）；其他环境见 [构建说明](docs/EGO_NVBLOX_GAZEBO.md#构建)。地图不随 Git 分发，只有加载地图流程需要事先保存的地图包。

## 推荐：使用仿真控制脚本

统一入口自动记录并复用 ROS 域、Gazebo 分区和场景布局，无需在每个终端手动设置环境。

```bash
# 终端 A：扩展场景 + 在线建图 + Gazebo/RViz；Ctrl+C 退出
./scripts/sim.sh start

# 终端 B：等待服务、初始化起点，成功后在 RViz 使用 2D Goal Pose
./scripts/sim.sh init
./scripts/sim.sh status
```

| 操作 | 命令 |
| --- | --- |
| 小场景 / 无界面后台 | `./scripts/sim.sh start --layout lab --view none --background` |
| 查看实时日志 | `./scripts/sim.sh logs --follow` |
| 取消目标，继续建图 | `./scripts/sim.sh cancel` |
| 保存当前地图 | `./scripts/sim.sh save .cache/maps/run1` |
| 完整离线扫描并保存 | `./scripts/sim.sh survey .cache/maps/survey1` |
| 退出整套仿真 | `./scripts/sim.sh stop` |

加载地图时，先 `stop`，再 `start --mode localization`，然后在另一个终端执行 `./scripts/sim.sh load .cache/maps/run1`。场景需与保存时一致，已有目录不会被覆盖。脚本不会自动保存地图。

详细的启动选项、逐步操作、命令行选点、状态判断和故障排查见 **[仿真启动与控制脚本手册](docs/SIMULATION_CONTROL.md)**。当前扩展场景已验证分段移动，但仍可能以 `EXPLORATION_BUDGET` 结束；不能将启动或发布目标成功理解为可靠到达。

## 原始 launch 与 ROS 命令

下面保留手动操作流程，供排查和调整底层参数。与控制脚本二选一，不要重复启动。**手动命令的每个终端**都先进入项目根目录，再设置相同环境变量：

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
```

### 方式一：仅查看场景与传感器

```bash
./simulation/scripts/run_uav_obstacle_course.sh
```

需要 MID360 ROS 点云桥接时，改为：

```bash
./simulation/scripts/run_uav_obstacle_course.sh launch_mid360:=true
```

该入口启动障碍场地、传感器桥、TF 和里程计速度估计，不启动 nvblox、EGO 或目标导航。操作见 [障碍场地说明](docs/UAV_OBSTACLE_COURSE.md)。

### 方式二：在线建图并朝目标探索

**终端 A：** 启动扩展场景、在线建图和 RViz。

```bash
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py \
  mode:=mapping gui:=true map_extent:=10.5 goal_height:=1.2 \
  world:="$PWD/src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf"
```

**终端 B：** 等待节点启动，初始化起飞区域。

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
./scripts/with_venv.sh python scripts/survey_gazebo_map.py --layout expanded --local-only
```

若提示 `Gazebo pose bridge unavailable`，先确认终端 B 的 `ROS_DOMAIN_ID` 与终端 A 相同；新终端不会继承另一个终端的 `export`。初始化失败时先不要发送目标：机体附近未知空间可能使无人机只旋转观察，最后报告 `NO_REACHABLE_FRONTIER`。修正环境并完成初始化后重新发送目标。

看到 `Local launch area observed` 后，在 RViz 顶部选择 **2D Goal Pose**（G），按下并拖动方向，松开即发送目标。默认目标 Z=1.2 m，中间轨迹可升降。目标可以在未知区域，系统会朝该方向选择安全观测位置，分段飞行并继续建图。

局部初始化使用 Gazebo `set_pose` 放置相机，补足前向相机的机体近距盲区，**不是自主起飞或探索飞行**。它不要求扫描整张地图，也不保存地图；运行期间不要发送导航目标。起点安全体积仍未观测时不会盲飞。

```bash
./scripts/with_venv.sh ros2 topic echo /uav/navigation/state \
  --qos-durability transient_local
```

`EXPLORING` 表示前往临时观测点，`NAVIGATING` 表示前往最终目标，`REACHED` 表示到达。目标已知被占据、无安全候选点或探索预算耗尽时进入 `BLOCKED`；地图、定位等异常进入 `STOPPED`。终止后不会因地图更新自动恢复，需要重新发送目标。详见 [在线探索说明](docs/EGO_NVBLOX_GAZEBO.md#在线建图与定向探索)。

### 方式三：离线扫描并保存地图

**终端 A：** 使用建图模式启动扩展场景。若方式二的同一套建图仿真仍在运行，可直接使用它，先取消导航目标；不要再启动一套。

```bash
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py \
  mode:=mapping gui:=true map_extent:=10.5 \
  world:="$PWD/src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf"
```

**终端 B：** 完整扫描并保存到一个尚不存在的目录。

```bash
./scripts/with_venv.sh python scripts/survey_gazebo_map.py \
  --layout expanded --output .cache/maps/uav_ego_expanded
```

完整扫描无需先执行 `--local-only`。等待 `Saved map bundle ...`；输出包含 `static_map.nvblx` 和 `manifest.json`。这是离线相机扫描，扫描期间不要发送导航目标。

在线探索得到的地图也可保存。先取消任务并确认执行器为 `HOLD`，再另存到新目录：

```bash
./scripts/with_venv.sh ros2 service call /uav/cancel std_srvs/srv/Trigger '{}'
./scripts/with_venv.sh ros2 topic echo /uav/executor/state --once
./scripts/with_venv.sh ros2 run uav_nav_sim map_bundle save .cache/maps/uav_ego_explored
```

已有目录不会被覆盖。详细流程见 [首次建图](docs/EGO_NVBLOX_GAZEBO.md#首次建图扩展场景)。

### 方式四：加载已有地图导航

**先退出原来的建图 launch，等待子进程结束。** `mode` 通过重新启动切换，不能在建图模式直接调用地图加载来代替模式切换。

**终端 A：** 使用与保存地图时相同的场景、查询范围和兼容配置启动。

```bash
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py \
  mode:=localization gui:=true map_extent:=10.5 goal_height:=1.2 \
  world:="$PWD/src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf"
```

**终端 B：** 加载方式三保存的地图，确认快照有效。

```bash
./scripts/with_venv.sh ros2 run uav_nav_sim map_bundle load .cache/maps/uav_ego_expanded
./scripts/with_venv.sh ros2 topic echo /uav/map/snapshot --once --field valid \
  --qos-durability transient_local
```

若首次输出 `false`，稍后重新检查；看到 `true` 后再使用 RViz **2D Goal Pose**。本机另有 `.cache/maps/uav_ego_expanded_20260926`，可用 `ls .cache/maps` 核对并替换加载路径，其他机器须使用自己的地图。

此模式不更新地图、不探索未知区域；使用 Gazebo 真值定位复用同场景静态地图，不是实机重定位。详见 [地图加载说明](docs/EGO_NVBLOX_GAZEBO.md#加载地图并导航扩展场景)。

### 小场景与显示方式

导航示例默认使用扩展场景；若要复现已验证的小场景探索，可使用下列完整入口，并在终端 B 将扫描布局设为 `lab`：

```bash
# 终端 A：默认 uav_ego_lab.sdf，map_extent 默认 5.0
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py \
  mode:=mapping gui:=true goal_height:=1.2
# 终端 B：局部初始化；完整扫描则改用 --output .cache/maps/uav_ego_lab
./scripts/with_venv.sh python scripts/survey_gazebo_map.py --layout lab --local-only
```

| 场景 | `world` | `map_extent` | 扫描 `--layout` |
| --- | --- | --- | --- |
| 默认小场景 | `uav_ego_lab.sdf`（省略 `world` 即可） | `5.0` | `lab` |
| 扩展场景 | 显式传入 `uav_ego_expanded.sdf` 路径 | `10.5` | `expanded` |

以下参数可用于上述两种导航模式，替换启动命令中的显示参数即可：

| 显示方式 | 参数 |
| --- | --- |
| Gazebo + RViz | `gui:=true` |
| 仅 RViz | `gui:=false launch_rviz:=true` |
| 无界面 | `gui:=false launch_rviz:=false` |

无界面仍需要 GPU 渲染深度。若以 `launch_rviz:=false` 启动，可在同域终端单独打开 RViz；同一域只运行一个选点转换节点：

```bash
./scripts/with_venv.sh ros2 launch uav_bringup rviz_navigation.launch.py goal_height:=1.2
```

地图默认按高度着色，颜色不代表可通行性。目标高度、显示参数和状态排查见 [运行指引](docs/EGO_NVBLOX_GAZEBO.md)。

### 停止与退出

```bash
./scripts/with_venv.sh ros2 service call /uav/cancel std_srvs/srv/Trigger '{}'
```

上述命令只取消导航目标，Gazebo、建图、规划器和 RViz 仍在运行。退出整套仿真时，在启动 launch 的终端 A 按 `Ctrl+C`，等待子进程退出；单独启动的 RViz launch 也在其终端按 `Ctrl+C`。

如果原终端已关闭，在新终端查找启动进程：

```bash
pgrep -af '[r]os2 launch uav_bringup (uav_ego_nvblox|gazebo_harmonic_nav|rviz_navigation)\.launch\.py'
```

确认命令属于本次仿真后，用实际 PID 替换下面的 `12345`，先查看，再发送正常退出信号：

```bash
SIM_PID=12345  # 替换为上一步查到的 launch PID
ps -p "$SIM_PID" -o pid,ppid,stat,args
```

确认无误后执行：

```bash
kill -INT -- "$SIM_PID"
```

等待 launch 清理子进程，再重复查询确认退出。残留 Gazebo、RViz 或节点的查找，以及进程无响应时的处理，见 [停止正在运行的进程](docs/EGO_NVBLOX_GAZEBO.md#停止正在运行的进程)。

## 其他构建与验证入口

硬件感知、定位和接口组件使用独立构建目录：

```bash
./scripts/build_uav_stack.sh
```

`with_venv.sh` 自动加载 `.venv`、系统 ROS 2 和 `install_uav`。请从未加载旧工作区的新终端运行；旧 `build/`、`install/` 不再由脚本加载。
可选 NVIDIA 定位/建图依赖使用 `./scripts/build_nvidia_3d_nav_deps.sh`。

OAK-D + cuVSLAM + 三维 EKF + RViz（硬件定位验证）：

```bash
./scripts/with_venv.sh ros2 launch uav_bringup oakd_visual_slam_rviz.launch.py
```

EKF 模拟输入验证：

```bash
./scripts/with_venv.sh ros2 launch uav_bringup ekf_mock_validation.launch.py
```

## 定位选项与实现范围

`nav_stack.launch.py` 保留传感器、三维 EKF、安全监测和 PX4 接口；EGO + nvblox 当前通过独立的 `uav_ego_nvblox.launch.py` 进行 Gazebo 算法验证，尚未接入硬件飞行链路。

- Isaac ROS Visual SLAM：使用 `oakd_visual_slam_rviz.launch.py` 独立验证。
- VINS-Fusion：保留源码、标定和启动入口，可通过 `nav_stack.launch.py enable_vins:=true odometry_source:=vio` 使用；`enable_vins:=false` 不会自动切换至 cuVSLAM。
- FAST-LIO：保留 `odometry_source:=lio` 选项。

硬件配置与限制见 [项目评估](docs/UAV_PROJECT_ASSESSMENT.md)。模块接口和后续验收标准见 [开发手册](docs/EGO_NVBLOX_DEVELOPMENT_GUIDE.md)，当前仿真运行方法以 [运行指引](docs/EGO_NVBLOX_GAZEBO.md) 为准。
完整保留已有 Isaac ROS 系列源码、工具和示例，见 [Isaac 内容说明](docs/ISAAC_ROS_COMPONENTS.md)。地面底盘串口桥和 Nav2 地面规划入口已移除。

## 主要代码

| 包 | 用途 |
| --- | --- |
| `uav_bringup` | 启动配置、三维 EKF、硬件定位验证、Gazebo 场景和 RViz 配置 |
| `uav_nav_sim` | 地图会话与地图包、Gazebo 执行器、RViz 目标高度转换 |
| `uav_ego_adapter` / `ego_planner_vendor` | 三维地图查询、EGO 轨迹规划与验收 |
| `isaac_ros_nvblox` | TSDF / ESDF 建图与网格显示 |
| `oakd_perception` / `imu_fusion` | OAK-D 图像、深度、点云和 IMU |
| `nav_mapping` | Livox 消息转换与三维点云合并 |
| `nav_safety` / `nav_guard` | 输入健康监测和里程计跳变保护 |
| `px4_comm_bridge` / `px4_msgs` | PX4 数据与控制接口 |
| `VINS-Fusion-ros2` | 可选 VINS 定位 |
| `nav_px4_bridge` | 已禁用构建的历史 PX4 兼容层 |

完整资料见 [文档索引](docs/INDEX.md)。

## Git 与第三方依赖维护

依赖初始化、补丁校验、LFS 资源及配置归属见
[Vendor 维护说明](patches/vendor/README.md)。构建前可执行：

```bash
./scripts/apply_vendor_patches.sh --check
./scripts/install_vendor_lfs_assets.sh --check
python3 scripts/vendor_patches.py --ego --check  # prepare_ego_vendor.sh 执行后
```

子模块应用补丁后显示 dirty 是预期状态，以精确校验结果为准。
Python 依赖通过 `uv pip install --python .venv/bin/python -r requirements/algorithm-sim.txt`
安装，不再从 `.deps/` 加载复制的 Python 包。
大型地图、模型网格和测试 bag 使用 Git LFS；正常 push 需包含对应 LFS 对象。

仓库初始飞行入口为 `px4-vio-sensors --flight hover-low`，完整参数及原始失败／成功
证据见 [BT VIO 飞行报告](docs/validation/simulation/2026-10-09-warehouse-bt-flight/REPORT.md)。
