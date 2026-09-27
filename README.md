# 无人机导航工作区（uav_nav_ws）

ROS 2 无人机导航实验项目，包含 OAK-D / MID360 感知、VIO / LIO、三维 EKF、EGO + nvblox 三维导航和 PX4 通信接口。
当前已实现 **Gazebo 中的深度建图、地图保存与加载、三维轨迹规划、速度模型执行和 RViz 选点导航**。
仿真使用 Gazebo 真值定位和简化速度模型；真实定位、PX4 飞控接入与实机自主飞行尚未完成闭环验证。

## 选择仿真入口

在项目根目录执行命令，同一时间只运行一套仿真。

| 目的 | 入口 | 启动内容 |
| --- | --- | --- |
| 查看障碍场地、检查传感器 | `./simulation/scripts/run_uav_obstacle_course.sh` | Gazebo、传感器桥、TF、里程计速度估计；不启动建图和导航 |
| 建图、地图复用、RViz 点击目标避障 | `uav_ego_nvblox.launch.py`，见下方命令 | Gazebo、nvblox、EGO、仿真执行器，可同时启动 RViz |

`uav_obstacle_course.sdf` 与导航示例的 `uav_ego_expanded.sdf` 是不同场景，不能混用地图或扫描布局。
完整操作步骤见 [EGO + nvblox 仿真运行指引](docs/EGO_NVBLOX_GAZEBO.md)。

## 快速运行导航仿真

### 首次准备

按 [安装指南](docs/INSTALLATION.md) 准备 `.venv`、ROS 2 Jazzy、Gazebo Harmonic / ros_gz 和 NVIDIA CUDA 环境，再构建算法链路：

```bash
./scripts/build_algorithm_sim.sh
```

当前构建脚本默认 CUDA 13.2、GPU 架构 89（本机 RTX 4070）；其他环境的调整说明见 [运行指引](docs/EGO_NVBLOX_GAZEBO.md#构建)。
没有地图时，先按 [首次建图](docs/EGO_NVBLOX_GAZEBO.md#首次建图扩展场景) 扫描并保存 `.cache/maps/uav_ego_expanded`，再执行下方加载流程。地图不随 Git 分发。

### 终端 A：启动扩展场景与 RViz

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py \
  mode:=localization gui:=true map_extent:=10.5 goal_height:=1.2 \
  world:="$PWD/src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf"
```

### 终端 B：加载地图

进入同一项目根目录，使用与终端 A 相同的环境变量：

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
# 本机已有扩展地图；其他机器请替换为自己扫描保存的目录
./scripts/with_venv.sh ros2 run uav_nav_sim map_bundle load .cache/maps/uav_ego_expanded_20260926
./scripts/with_venv.sh ros2 topic echo /uav/map/snapshot --once --field valid \
  --qos-durability transient_local
```

本机已有目录为 `.cache/maps/uav_ego_expanded_20260926`；按首次建图指引新建的目录为 `.cache/maps/uav_ego_expanded`，两者不要混淆。可先运行 `ls .cache/maps` 核对目录，再填写加载路径。等待 `valid` 输出 `true` 后，在 RViz 顶部选择 **2D Goal Pose**（G），在已观测的空闲区域按下并拖动方向，松开即发送目标。默认目标高度为地图坐标系 Z=1.2 m，中间轨迹仍可升降。

地图默认按高度着色：`3D nvblox Map → Mesh Color → Height`；可改为 `Normals` 区分表面朝向。
修改目标高度、查看规划状态、重开 RViz 和常见问题见 [操作说明](docs/EGO_NVBLOX_GAZEBO.md#rviz-定高选点)。

### Gazebo 中的建图模式

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py \
  mode:=mapping gui:=true map_extent:=10.5 goal_height:=1.2 \
  world:="$PWD/src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf"
```

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
