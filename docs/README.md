# 项目文档总目录

[项目首页](../README.md) · [脚本目录](../scripts/README.md) · [开发日志](history/DEVELOPMENT_LOG.md) · [维护与迁移约定](MAINTENANCE.md)

先按用途选择入口：日常算法仿真看 [仿真控制](simulation/SIMULATION_CONTROL.md)，PX4 动力学看 [本机 SITL](../simulation/px4/README.md)，硬件定位看 [OAK-D](hardware/OAKD_VISUAL_SLAM_RVIZ.md)。命令统一从项目根目录执行。

## 阅读顺序与文档职责

1. 安装和构建：`setup/`；先准备工作区，再选择算法或 PX4 的构建入口。
2. 运行与排错：`simulation/`、`flight/`、`hardware/`；使用具体链路的操作手册。
3. 导航和任务契约：`navigation/` 与模块文档；查询 Action、地图会话、坐标和控制权。
4. 开发设计与验收规划：`development/`；其中待开发条目不是实现或飞行证据。
5. 结果与历史：[开发日志](history/DEVELOPMENT_LOG.md) 汇集阶段进展，`validation/` 保存本地实测记录，`history/` 同时保留历史首页快照。

`SIMULATION_CONTROL` 负责日常命令，`EGO_NVBLOX_GAZEBO` 负责底层 launch 参数和故障排查，`SIMULATION_MANUAL` 负责系统架构与接口；首页和此目录只保留入口。原 `INDEX.md` 已合并到本页。

<!-- BEGIN DOCUMENT CATALOG -->

## 安装与构建

| 文档 | 主题 |
| --- | --- |
| [docs/setup/CUDA_TOOLKIT_13_2_INSTALLATION.md](setup/CUDA_TOOLKIT_13_2_INSTALLATION.md) | CUDA Toolkit 13.2 安装指南 |
| [docs/setup/INSTALLATION.md](setup/INSTALLATION.md) | 安装与构建指南 |

## 仿真运行与场景

| 文档 | 主题 |
| --- | --- |
| [docs/simulation/EGO_NVBLOX_GAZEBO.md](simulation/EGO_NVBLOX_GAZEBO.md) | EGO + nvblox 仿真运行指引 |
| [docs/simulation/GAZEBO_HARMONIC_SIMULATION.md](simulation/GAZEBO_HARMONIC_SIMULATION.md) | Gazebo Harmonic 无人机场景 |
| [docs/simulation/LAUNCHERS.md](simulation/LAUNCHERS.md) | 仿真启动规划 |
| [docs/simulation/OBSTACLE_COURSE.md](simulation/OBSTACLE_COURSE.md) | 四区无人机障碍赛道 |
| [docs/simulation/SIMULATION_CONTROL.md](simulation/SIMULATION_CONTROL.md) | 仿真启动与控制脚本 |
| [docs/simulation/SIMULATION_MANUAL.md](simulation/SIMULATION_MANUAL.md) | 无人机导航仿真使用手册 (UAV Simulation User Manual) |
| [docs/simulation/UAV_OBSTACLE_COURSE.md](simulation/UAV_OBSTACLE_COURSE.md) | UAV 障碍穿越场地 |
| [docs/simulation/VISION_MONITOR.md](simulation/VISION_MONITOR.md) | RGB、检测与 AR 路线监控界面 |
| [docs/simulation/WAREHOUSE_SIMULATION.md](simulation/WAREHOUSE_SIMULATION.md) | 仓库场景与分阶段 VIO 验证 |

## 导航与任务契约

| 文档 | 主题 |
| --- | --- |
| [docs/navigation/ALGORITHM_MISSIONS.md](navigation/ALGORITHM_MISSIONS.md) | algorithm 两航点 BT 与 MissionServer |
| [docs/navigation/EGO_NVBLOX_DEVELOPMENT_GUIDE.md](navigation/EGO_NVBLOX_DEVELOPMENT_GUIDE.md) | EGO + nvblox 无人机自主导航开发手册 |
| [docs/navigation/NAVIGATION_ACTION.md](navigation/NAVIGATION_ACTION.md) | EGO 导航 Action（algorithm） |
| [docs/navigation/PLANNING_CONTEXT.md](navigation/PLANNING_CONTEXT.md) | 带坐标与会话绑定的 EGO 影子规划 |

## PX4、BT 与定位模式

| 文档 | 主题 |
| --- | --- |
| [docs/flight/MTF01_SIMULATION.md](flight/MTF01_SIMULATION.md) | MTF-01 光流／测距仿真 |
| [docs/flight/MTF01_STANDALONE_FLIGHT.md](flight/MTF01_STANDALONE_FLIGHT.md) | MTF-01 直连飞控：不依赖 Jetson 的纯光流模式 |
| [docs/flight/PX4_LOCALIZATION_GATES.md](flight/PX4_LOCALIZATION_GATES.md) | PX4 定位输入门控（S3 首批） |
| [docs/flight/PX4_STEP_BT.md](flight/PX4_STEP_BT.md) | 本机 PX4 逐步骤 BT |
| [docs/flight/USB_POCKET_SIMULATION.md](flight/USB_POCKET_SIMULATION.md) | Pocket USB 与行为树决策仿真 |
| [docs/flight/VIO_HOVER.md](flight/VIO_HOVER.md) | OAK-D Pro W 辅助室内悬停 |
| [docs/flight/VIO_POCKET_AVOIDANCE.md](flight/VIO_POCKET_AVOIDANCE.md) | Pocket 无 GNSS VIO 与静态避障仿真 |
| [docs/flight/VIRTUAL_POCKET_PANEL.md](flight/VIRTUAL_POCKET_PANEL.md) | Pocket UAV 电脑面板 |

## 硬件定位与通信

| 文档 | 主题 |
| --- | --- |
| [docs/hardware/DAMIAO_IMU_USB.md](hardware/DAMIAO_IMU_USB.md) | 达妙 USB IMU 与视觉里程计融合 |
| [docs/hardware/OAKD_VISUAL_SLAM_RVIZ.md](hardware/OAKD_VISUAL_SLAM_RVIZ.md) | OAK-D Visual SLAM 与 RViz 验证 |
| [docs/hardware/PX4_MICRO_XRCE_DDS_SERIAL_SETUP.md](hardware/PX4_MICRO_XRCE_DDS_SERIAL_SETUP.md) | PX4 + Micro XRCE-DDS + ROS 2 串口通信接入指南 |
| [docs/hardware/PX4_USB_BENCH.md](hardware/PX4_USB_BENCH.md) | DM-FC01 USB 台架检查 |

## 开发设计与计划

| 文档 | 主题 |
| --- | --- |
| [docs/development/BT_PX4_DEVELOPMENT_PLAN.md](development/BT_PX4_DEVELOPMENT_PLAN.md) | BehaviorTree 与 PX4 协同开发计划 |
| [docs/development/ISAAC_ROS_COMPONENTS.md](development/ISAAC_ROS_COMPONENTS.md) | Isaac 系列保留范围 |
| [docs/development/MISSION_PERSON_FOLLOWING.md](development/MISSION_PERSON_FOLLOWING.md) | 任务内临时人物跟随与轨迹记录 |
| [docs/development/PERSON_FOLLOWING_PLAN.md](development/PERSON_FOLLOWING_PLAN.md) | 移动人体主动跟踪实现方案 |
| [docs/development/PX4_INTEGRATION_ROADMAP.md](development/PX4_INTEGRATION_ROADMAP.md) | PX4 官方链路三阶段集成 |
| [docs/development/PX4_NAVIGATION_STRATEGY.md](development/PX4_NAVIGATION_STRATEGY.md) | PX4 三维导航策略 |
| [docs/development/PX4_STATE_MACHINE_DESIGN.md](development/PX4_STATE_MACHINE_DESIGN.md) | W2-D8-D9: PX4 自动武装状态机设计 |
| [docs/development/SIMULATION_DEVELOPMENT_PLAN.md](development/SIMULATION_DEVELOPMENT_PLAN.md) | UAV 全链路仿真开发计划 |
| [docs/development/UAV_PROJECT_ASSESSMENT.md](development/UAV_PROJECT_ASSESSMENT.md) | 无人机导航项目评估 |

## 历史资料

| 文档 | 主题 |
| --- | --- |
| [docs/history/DEVELOPMENT_LOG.md](history/DEVELOPMENT_LOG.md) | 开发日志 |
| [docs/history/POCKET_SC_DESIGN_2026-10-13.md](history/POCKET_SC_DESIGN_2026-10-13.md) | Pocket SB／SC 早期方案（2026-10-13） |
| [docs/history/README_2026-10-13.md](history/README_2026-10-13.md) | 项目首页历史快照（2026-10-13 整理前） |

## 模块文档

| 模块资料 | 主题 |
| --- | --- |
| [src/uav_bringup/gazebo/models/THIRD_PARTY_NOTICES.md](../src/uav_bringup/gazebo/models/THIRD_PARTY_NOTICES.md) | Third-party Gazebo arena assets |
| [src/uav_ego_adapter/README.md](../src/uav_ego_adapter/README.md) | EGO nvblox adapter |
| [src/ego_planner_vendor/README.md](../src/ego_planner_vendor/README.md) | EGO library vendor |
| [src/uav_mission/PROTOCOL.md](../src/uav_mission/PROTOCOL.md) | S1 task identity and FlightSession mock |
| [src/uav_mission/README.md](../src/uav_mission/README.md) | S1 AircraftState observer |
| [src/uav_bt/README.md](../src/uav_bt/README.md) | uav_bt：PX4 与 algorithm 任务树 |
| [src/uav_nav_interfaces/README.md](../src/uav_nav_interfaces/README.md) | Algorithm simulation contracts |
| [src/nav_guard/README.md](../src/nav_guard/README.md) | nav_guard |
| [src/nav_safety/README.md](../src/nav_safety/README.md) | nav_safety |
| [src/nav_mapping/README.md](../src/nav_mapping/README.md) | nav_mapping |
| [src/nav_px4_bridge/README.md](../src/nav_px4_bridge/README.md) | nav_px4_bridge (Deprecated) |
| [src/imu_fusion/IMU_MULTI_CONFIG.md](../src/imu_fusion/IMU_MULTI_CONFIG.md) | IMU 融合节点多IMU配置指南 |
| [src/oakd_perception/CHANGELOG.md](../src/oakd_perception/CHANGELOG.md) | 已归档：深度模式修改总结 |
| [src/oakd_perception/DEPTH_MODE_CONFIG.md](../src/oakd_perception/DEPTH_MODE_CONFIG.md) | OAK-D 深度模式配置指南 |
| [src/oakd_perception/FOV_FILTER_QUICK_REF.md](../src/oakd_perception/FOV_FILTER_QUICK_REF.md) | FOV过滤规则 - 快速参考卡 |
| [src/oakd_perception/FOV_FILTER_RULES.md](../src/oakd_perception/FOV_FILTER_RULES.md) | FOV边界点云过滤规则详细说明 |
| [src/oakd_perception/IMU_QUICK_START.md](../src/oakd_perception/IMU_QUICK_START.md) | OAK-D IMU 使用指南 |
| [src/oakd_perception/QUICK_START.md](../src/oakd_perception/QUICK_START.md) | 快速参考（已整合） |
| [src/oakd_perception/README.md](../src/oakd_perception/README.md) | oakd_perception 文档中心 |
| [src/oakd_perception/archive/CHANGELOG.md](../src/oakd_perception/archive/CHANGELOG.md) | 已归档：深度模式修改总结 |
| [src/oakd_perception/scripts/README.md](../src/oakd_perception/scripts/README.md) | 脚本目录说明（已整合） |
| [src/px4_comm_bridge/README.md](../src/px4_comm_bridge/README.md) | px4_comm_bridge |
| [src/px4_comm_bridge/docs/QGC_SETUP_GUIDE.md](../src/px4_comm_bridge/docs/QGC_SETUP_GUIDE.md) | QGroundControl 地面站配置指南 |
| [src/uav_target_perception/README.md](../src/uav_target_perception/README.md) | RGB 人员检测与地图定位 |

## 仿真与依赖资源

| 入口 | 内容 |
| --- | --- |
| [simulation/README.md](../simulation/README.md) | 无人机仿真入口 |
| [simulation/px4/README.md](../simulation/px4/README.md) | 本机 PX4 SITL 基础环境 |
| [simulation/acceptance/README.md](../simulation/acceptance/README.md) | 仿真验收配置（S0 首批） |
| [simulation/assets/README.md](../simulation/assets/README.md) | Isaac Sim Assets |
| [patches/vendor/README.md](../patches/vendor/README.md) | Vendor patch manifest |

<!-- END DOCUMENT CATALOG -->

## 验证记录

`docs/validation/` 与 `scripts/tests/local/test_*` 不纳入 Git，本地记录链接在新克隆中可能不可用。报告中的“当前”“尚未”按该轮日期理解；仿真、静态检查、台架与实机证据分别解释。使用范围见 [项目首页](../README.md#当前验证范围)，阶段结果见 [开发日志](history/DEVELOPMENT_LOG.md)。

| 日期／主题 | 记录与适用范围 |
| --- | --- |
| 2026-10-13，20 cm 定位验收 | [完整名义导航](validation/simulation/2026-10-13-nav-20cm/REPORT.md)：最大定位误差和到点误差通过，动态跟踪及其他验收缺口另列 |
| 2026-10-13，完整 VIO 链路诊断 | [时钟、前馈与 EKF 噪声](validation/simulation/2026-10-13-vio-chain-diagnosis/REPORT.md) |
| 2026-10-13，SDK16 联合任务 | [起飞、建图、导航、悬停、返航、降落](validation/simulation/2026-10-13-vio-chain/REPORT.md)：协方差资格暂缓，主动绕障待验收 |
| 2026-10-13，临时返航与恢复 | [检查点与独立恢复意图](validation/simulation/2026-10-13-return-resume/REPORT.md) |
| 2026-10-12，SDK16 采用与 10 cm 诊断 | [动态融合、协方差导出和失败证据](validation/simulation/2026-10-12-sdk16-adoption/REPORT.md)：历史阶段的回退说明以当前入口为准 |
| 2026-10-12，无 GNSS 全链路 | [源失效处置与准入缺口](validation/simulation/2026-10-12-full-chain/REPORT.md)：后续进展见 10-13 记录 |
| 2026-10-10，OAK-D Pro W 硬件 | [静止审计](validation/hardware/2026-10-10-oakd-pro-w-vio/REPORT.md)、[移动 UI](validation/hardware/2026-10-10-oakd-pro-w-vio/motion-ui-run1/REPORT.md)、[导航输出优化](validation/hardware/2026-10-10-oakd-pro-w-vio/navigation-optimization/REPORT.md) |
| 2026-10-08，任务与接口 | [BT／PX4](validation/simulation/2026-10-08-bt-px4/REPORT.md)、[算法 Mission](validation/simulation/2026-10-08-algorithm-mission/REPORT.md)、[算法故障](validation/simulation/2026-10-08-algorithm-faults/REPORT.md)、[导航 Action](validation/simulation/2026-10-08-navigation-action/REPORT.md) |
| 2026-10-08，参考深度传感器 | [OakD-Lite 参考链路](validation/simulation/2026-10-08-px4-depth/REPORT.md)：不是 OAK-D Pro W 标定验收 |

完整阶段叙述保存在 [历史首页](history/README_2026-10-13.md)。可用以下只读命令列出本机全部报告，原始证据和校验清单保留在各运行目录：

```bash
rg --files docs/validation -g '*.md' | sort
```

- [Pocket VIO／深度／EGO 静态避障仿真](flight/VIO_POCKET_AVOIDANCE.md)
