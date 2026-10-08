# 无人机导航工作区（uav_nav_ws）

ROS 2 无人机导航实验项目，包含 OAK-D / MID360 感知、VIO / LIO、三维 EKF、EGO + nvblox 三维导航和 PX4 通信接口。
当前已实现 **Gazebo 中的深度建图、定向探索、地图保存与加载、三维轨迹规划、速度模型执行和 RViz 选点导航**。
算法仿真使用 Gazebo 真值定位和简化速度模型；另有独立 PX4 v1.16.2 SITL 已知区域飞行后端，支持起飞、航点、悬停、返航和降落。感知导航与 PX4 的融合、外部 VIO 及实机飞行仍待验证。

本机 PX4/BT 入口：首次执行 `./scripts/build_px4_sim.sh --jobs 4` 和 `./scripts/build_px4_flight.sh`，随后运行 `./scripts/sim.sh px4-flight --bt --ui`，显示 Gazebo/QGC 并执行逐步骤飞行 BT。任务完成后确认着陆解除武装并自动清理本次会话。详见 [PX4 运行说明](simulation/px4/README.md) 与 [BT 生命周期](src/uav_bt/README.md)。

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
