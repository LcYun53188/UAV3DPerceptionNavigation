# 无人机导航工作区（uav_nav_ws）

基于 ROS 2 的无人机感知、定位、三维建图、路径规划与任务执行工作区。项目使用 OAK-D Pro W／MID360、VIO／LIO、nvblox、EGO Planner 和 PX4，以本机 Gazebo 仿真开发为主。

[文档总目录](docs/README.md) · [脚本总目录](scripts/README.md) · [开发日志](docs/history/DEVELOPMENT_LOG.md)

## 功能与代码

所有受管仿真统一使用**无 GNSS 环境**。两个程序默认使用实体 HID 遥控器；模拟遥控器需显式选择。旧 GNSS 任务已停用，详见[启动策略](docs/simulation/LAUNCHERS.md#全局无-gnss-策略)。

| 功能 | 主要模块 |
| --- | --- |
| 相机、深度、点云与 IMU | `oakd_perception`、`imu_fusion`、`nav_mapping` |
| 定位与输入健康监测 | cuVSLAM、VINS-Fusion、FAST-LIO、三维 EKF、`nav_guard`、`nav_safety` |
| 三维地图、地图包与轨迹规划 | nvblox、`uav_nav_sim`、`uav_ego_adapter`、EGO Planner |
| 任务编排与飞控执行 | `uav_mission`、`uav_bt`、`px4_comm_bridge` |
| 场景、启动与显示 | `uav_bringup`、Gazebo Harmonic、RViz、QGroundControl |

模块接口和包级说明见 [模块文档](docs/README.md#模块文档)。

## 当前验证范围

日常仿真只保留两种程序：无 VIO 的光流纯手动飞行，以及 VIO／光流／测距融合、nvblox 建图、EGO 规划和行为树任务的完整项目。两者均使用 PX4 SITL 动力学。专项回归工具保留在分类目录中，不再作为总入口的启动类型。

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

运行 `./scripts/sim.sh` 选择程序，或直接使用以下命令。默认识别实体 RadioMaster Pocket HID（8 轴／24 按钮），未连接时明确报错；启动后在 QGC 校准并启用摇杆，在 Fly 页面发送操纵。程序不会自动解锁或起飞。

| 程序 | 命令 | 内容 |
| --- | --- | --- |
| 纯手动飞行 | `./scripts/sim.sh run manual` | Gazebo、PX4、QGC，光流／测距定位，无 VIO、建图和自动任务 |
| 完整项目 | `./scripts/sim.sh run full` | Gazebo、PX4、QGC、RViz，VIO、光流／测距、nvblox、EGO 与遥控器任务控制 |

使用模拟遥控器时，程序自动复用或打开电脑面板，无需另开面板命令：

```bash
./scripts/sim.sh run full --input-source virtual
# 纯手动程序同样支持 --input-source virtual
```

`--device /dev/input/...` 可指定设备；校准完成并退出后自动保存，下次同设备无需重复校准。`--qgc-config <文件>` 可显式导入，`--dry-run` 只预览。完整说明见[仿真启动规划](docs/simulation/LAUNCHERS.md)。

完整项目可用 `--people` 显式启用 RGB 人员检测、深度定位与地图标注，默认关闭；先按[人员感知模块说明](src/uav_target_perception/README.md)构建并准备模型。

## 快速运行导航仿真

完成[构建准备](docs/simulation/LAUNCHERS.md#构建准备)后，在两个终端运行：

```bash
# 终端 A：保持完整项目运行；默认实体 HID
./scripts/sim.sh run full
```

```bash
# 终端 B：自动连接当前工作区的运行会话，无需手工 export ROS_DOMAIN_ID
./scripts/sim.sh status
./scripts/sim.sh logs
# 飞机落地且上锁后保存地图包，目录须不存在
./scripts/sim.sh save .cache/maps/run1
./scripts/sim.sh export-mesh .cache/maps/run1.ply
# 请求正常关闭；飞行中会等待操作员降落上锁
./scripts/sim.sh stop
```

运行器异常退出时，用 `./scripts/sim.sh cleanup` 查看残留，确认清单后执行 `./scripts/sim.sh cleanup --apply`。

地图命令通过本轮完整项目的地图服务执行。纯手动程序没有地图；旧算法仿真、自动测试、场景查看与专项检查不再列为公共启动入口。当前完整项目的任务由遥控器触发，地图操作及关闭方法见[仿真控制手册](docs/simulation/SIMULATION_CONTROL.md)。

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
python3 scripts/tools/maintain_catalogs.py --write
python3 scripts/tools/maintain_catalogs.py --check
```

大型地图、模型与 bag 使用 Git LFS。`docs/validation/` 与 `scripts/tests/local/test_*` 是 Git 忽略的本地证据和独立回归脚本，新克隆可能没有这些文件；包内测试和运行工具继续跟踪。
