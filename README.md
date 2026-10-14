# 无人机导航工作区（uav_nav_ws）

基于 ROS 2 的无人机感知、定位、三维建图、路径规划与任务执行工作区。项目使用 OAK-D Pro W／MID360、VIO／LIO、nvblox、EGO Planner 和 PX4，以本机 Gazebo 仿真开发为主。

[文档总目录](docs/README.md) · [脚本总目录](scripts/README.md) · [开发日志](docs/history/DEVELOPMENT_LOG.md)

## 功能与代码

| 功能 | 主要模块 |
| --- | --- |
| 相机、深度、点云与 IMU | `oakd_perception`、`imu_fusion`、`nav_mapping` |
| 定位与输入健康监测 | cuVSLAM、VINS-Fusion、FAST-LIO、三维 EKF、`nav_guard`、`nav_safety` |
| 三维地图、地图包与轨迹规划 | nvblox、`uav_nav_sim`、`uav_ego_adapter`、EGO Planner |
| 任务编排与飞控执行 | `uav_mission`、`uav_bt`、`px4_comm_bridge` |
| 场景、启动与显示 | `uav_bringup`、Gazebo Harmonic、RViz、QGroundControl |

模块接口和包级说明见 [模块文档](docs/README.md#模块文档)。

## 当前验证范围

算法仿真支持深度建图、探索、地图保存加载、三维规划和 RViz 选点，使用 Gazebo 真值定位与简化速度执行器。PX4 SITL 提供独立动力学任务链路，并包含 VIO／光流／测距融合及实验导航入口。

仿真、硬件定位与实机飞行按各自的验收范围解释；实验导航的准入条件见 [VIO 说明](docs/flight/VIO_HOVER.md)。阶段结果、误差指标和待验收项集中在 [开发日志](docs/history/DEVELOPMENT_LOG.md) 与 [验证记录](docs/README.md#验证记录)。

## 运行前准备

所有命令从项目根目录执行。按 [安装与构建指南](docs/setup/INSTALLATION.md) 准备 `.venv`、ROS 2 Jazzy、Gazebo Harmonic／ros_gz 和 CUDA，再构建所需链路。

算法仿真构建：

```bash
./scripts/setup/apply_vendor_patches.sh --check
./scripts/setup/install_vendor_lfs_assets.sh --check
./scripts/build/build_algorithm_sim.sh
```

`with_venv.sh` 加载 Python 虚拟环境、系统 ROS 2 与 `install_uav`；PX4 使用 `with_px4_sim.sh` 的隔离环境。其他构建和环境要求见 [脚本目录](scripts/README.md)。

## 选择运行方式

| 用途 | 运行入口 | 详细说明 |
| --- | --- | --- |
| 在线建图、探索与 RViz 导航 | `./scripts/sim.sh start` | [仿真控制](docs/simulation/SIMULATION_CONTROL.md) |
| 扫描、保存与加载地图 | `./scripts/sim.sh survey`、`save`、`load` | [地图操作](docs/simulation/EGO_NVBLOX_GAZEBO.md) |
| 查看场景与传感器 | `./simulation/scripts/run_uav_obstacle_course.sh` | [Gazebo 场景](docs/simulation/GAZEBO_HARMONIC_SIMULATION.md) |
| PX4／BT 逐步骤飞行仿真 | `./scripts/sim.sh px4-flight --bt --ui` | [构建与运行](simulation/px4/README.md)、[步骤授权](docs/flight/PX4_STEP_BT.md) |
| VIO／光流／测距未解锁融合审计 | `./scripts/sim.sh px4-vio-mtf01` | [联合融合](docs/flight/MTF01_SIMULATION.md) |
| 独立纯光流飞行仿真 | `./scripts/sim.sh px4-flow-local --duration 35 --native-manual` | [MTF-01 独立控制](docs/flight/MTF01_STANDALONE_FLIGHT.md) |
| Pocket USB 操作与定位源选择 | `scripts/usb_joystick.py`、`scripts/select_localization.py` | [USB 操作](docs/flight/USB_POCKET_SIMULATION.md) |
| OAK-D 硬件静止与移动验证 | `scripts/audit_oakd_vio_hardware.py`、`scripts/oakd_vio_motion_ui.py` | [硬件定位](docs/hardware/OAKD_VISUAL_SLAM_RVIZ.md) |

各入口先按对应手册构建、配置和检查。算法导航的 `mapping` 更新地图，`localization` 加载静态地图；场景须与地图包一致。选择一条链路运行。

## 快速运行导航仿真

完成算法构建后，打开两个终端：

```bash
# 终端 A：扩展场景、在线建图与 Gazebo/RViz；Ctrl+C 退出
./scripts/sim.sh start

# 终端 B：初始化起点、查看状态，然后在 RViz 选点
./scripts/sim.sh init
./scripts/sim.sh status
```

常用控制：

```bash
# 取消导航目标，保持仿真运行
./scripts/sim.sh cancel
# 保存地图，输出目录须尚不存在
./scripts/sim.sh save .cache/maps/run1
# 退出整套受管算法仿真
./scripts/sim.sh stop
```

探索、地图加载、日志和手动 launch 的退出方法见 [仿真控制手册](docs/simulation/SIMULATION_CONTROL.md)；PX4 的取消、降落和会话清理见 [PX4 运行说明](simulation/px4/README.md)。

## 文档与维护

| 资料 | 内容 |
| --- | --- |
| [文档总目录](docs/README.md) | 安装、仿真、导航、飞行、硬件与模块资料 |
| [脚本总目录](scripts/README.md) | 工具用途、环境与调用方式 |
| [开发日志](docs/history/DEVELOPMENT_LOG.md) | 按日期记录开发进展、实验指标与遗留问题 |
| [全链路开发计划](docs/development/SIMULATION_DEVELOPMENT_PLAN.md) | 阶段任务、依赖与验收标准 |
| [第三方依赖维护](patches/vendor/README.md) | 子模块补丁、版本约束与 Git LFS |
| [文档维护约定](docs/MAINTENANCE.md) | 目录归属、引用规则与迁移映射 |

更新文档或脚本后执行：

```bash
python3 scripts/maintain_catalogs.py --write
python3 scripts/maintain_catalogs.py --check
```

大型地图、模型与 bag 使用 Git LFS。`docs/validation/` 与 `scripts/test_*` 是 Git 忽略的本地证据和独立回归脚本，新克隆可能没有这些文件；包内测试和运行工具继续跟踪。
