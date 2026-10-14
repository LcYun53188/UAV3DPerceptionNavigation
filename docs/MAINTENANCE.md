# 文档与脚本维护约定

项目首页负责项目概况、使用范围与快速入口；[开发日志](history/DEVELOPMENT_LOG.md) 负责阶段进展和实验结果；[文档总目录](README.md) 负责全部资料导航；[脚本总目录](../scripts/README.md) 负责每个工具的用途、环境与分类。

## 目录归属

| 路径 | 内容 |
| --- | --- |
| `docs/setup/` | 安装、CUDA 与构建环境 |
| `docs/simulation/` | 算法仿真、场景、仓库和运行排错 |
| `docs/navigation/` | 地图、规划、Action 与算法任务契约 |
| `docs/flight/` | PX4／BT、VIO、光流、USB 操作与定位门控 |
| `docs/hardware/` | OAK-D、USB IMU、PX4 串口和台架操作 |
| `docs/development/` | 架构决策、开发计划、项目评估和依赖范围 |
| `docs/history/` | 开发日志与日期明确的历史快照 |
| `docs/validation/` | Git 忽略的本地报告、失败证据与原始数据 |
| `scripts/build/` | 构建入口 |
| `scripts/setup/` | 安装与第三方依赖准备 |
| `scripts/` | 统一运行入口、Python 运行器、分析工具和共享模块 |
| `simulation/scripts/` | 场景与 Isaac Sim 启动入口 |
| `src/<package>/` | 随模块维护的文档、测试与包内脚本 |

Python 运行器和共享模块仍使用根目录导入布局，避免修改运行、测试与证据冻结的模块依赖。所有工具按用途列入脚本目录；构建、安装脚本已实际迁移，旧 `.sh` 路径为指向同一实现的相对符号链接。运行、参数和退出码继续透传；规范路径与旧路径都按真实脚本位置解析工作区根目录。

## 合并与引用规则

- `docs/INDEX.md` 与旧 `docs/README.md` 的导航内容合并到 `docs/README.md`。
- 阶段进展、实验指标、测试计数和临时运行状态集中到 `docs/history/DEVELOPMENT_LOG.md` 或专项报告，按日期记录；首页保留项目介绍、使用范围与操作入口。
- 早期首页历史更新和重复操作流程保存在 `docs/history/README_2026-10-13.md`，作为历史快照保留。
- 三份算法仿真手册按日常命令、底层参数、系统架构分工，保留有用的专门内容与现有章节锚点。
- 新文档写入对应主题目录，模块细节随包维护；相对链接以文档所在目录为基准，代码块里的命令以项目根目录为基准。
- 原始验证记录不迁移、不改写；旧报告里的文档路径可按下表定位。文档迁移不等于重新运行或重新认可历史结论。
- 第三方源码与文档按 vendor／上游约定维护，不批量重写。新增构建脚本使用 `scripts/build/`，安装脚本使用 `scripts/setup/`。

## 目录更新与检查

```bash
python3 scripts/maintain_catalogs.py --write
python3 scripts/maintain_catalogs.py --check
```

工具使用 Python 标准库、Bash，并在 Git 工作区检查目录文件是否被忽略：生成主题文档、模块文档及脚本目录，检查覆盖率、项目文档的本地链接与 Markdown 章节锚点，以及工作区脚本的 Python／Bash 语法。它不会执行被扫描脚本、启动 Gazebo、访问设备或发送飞控命令。HTTP 链接不联网核验，RST／HTML／Mermaid 的引用不在 Markdown 链接检查范围。

本地 `docs/validation` 链接在目标缺失时统计为可选证据，不判为失败；历史报告不参与当前文档检查。`scripts/test_*` 只在本机存在时参与语法检查，不写入版本化目录，包内测试继续随代码管理。检查通过表示静态一致性，不代表 ROS 图、SITL 或实机验收。

## 文档旧路径迁移表

文档原路径已迁移，项目内现行引用同步更新；外部收藏和历史证据可按此表定位。`docs/INDEX.md` 继续提供目录入口。

| 原路径 | 新路径 |
| --- | --- |
| `docs/INSTALLATION.md` | [docs/setup/INSTALLATION.md](setup/INSTALLATION.md) |
| `docs/CUDA_TOOLKIT_13_2_INSTALLATION.md` | [docs/setup/CUDA_TOOLKIT_13_2_INSTALLATION.md](setup/CUDA_TOOLKIT_13_2_INSTALLATION.md) |
| `docs/SIMULATION_CONTROL.md` | [docs/simulation/SIMULATION_CONTROL.md](simulation/SIMULATION_CONTROL.md) |
| `docs/SIMULATION_MANUAL.md` | [docs/simulation/SIMULATION_MANUAL.md](simulation/SIMULATION_MANUAL.md) |
| `docs/EGO_NVBLOX_GAZEBO.md` | [docs/simulation/EGO_NVBLOX_GAZEBO.md](simulation/EGO_NVBLOX_GAZEBO.md) |
| `docs/GAZEBO_HARMONIC_SIMULATION.md` | [docs/simulation/GAZEBO_HARMONIC_SIMULATION.md](simulation/GAZEBO_HARMONIC_SIMULATION.md) |
| `docs/UAV_OBSTACLE_COURSE.md` | [docs/simulation/UAV_OBSTACLE_COURSE.md](simulation/UAV_OBSTACLE_COURSE.md) |
| `docs/WAREHOUSE_SIMULATION.md` | [docs/simulation/WAREHOUSE_SIMULATION.md](simulation/WAREHOUSE_SIMULATION.md) |
| `docs/EGO_NVBLOX_DEVELOPMENT_GUIDE.md` | [docs/navigation/EGO_NVBLOX_DEVELOPMENT_GUIDE.md](navigation/EGO_NVBLOX_DEVELOPMENT_GUIDE.md) |
| `docs/ALGORITHM_MISSIONS.md` | [docs/navigation/ALGORITHM_MISSIONS.md](navigation/ALGORITHM_MISSIONS.md) |
| `docs/NAVIGATION_ACTION.md` | [docs/navigation/NAVIGATION_ACTION.md](navigation/NAVIGATION_ACTION.md) |
| `docs/PLANNING_CONTEXT.md` | [docs/navigation/PLANNING_CONTEXT.md](navigation/PLANNING_CONTEXT.md) |
| `docs/PX4_STEP_BT.md` | [docs/flight/PX4_STEP_BT.md](flight/PX4_STEP_BT.md) |
| `docs/USB_POCKET_SIMULATION.md` | [docs/flight/USB_POCKET_SIMULATION.md](flight/USB_POCKET_SIMULATION.md) |
| `docs/MTF01_SIMULATION.md` | [docs/flight/MTF01_SIMULATION.md](flight/MTF01_SIMULATION.md) |
| `docs/MTF01_STANDALONE_FLIGHT.md` | [docs/flight/MTF01_STANDALONE_FLIGHT.md](flight/MTF01_STANDALONE_FLIGHT.md) |
| `docs/VIO_HOVER.md` | [docs/flight/VIO_HOVER.md](flight/VIO_HOVER.md) |
| `docs/PX4_LOCALIZATION_GATES.md` | [docs/flight/PX4_LOCALIZATION_GATES.md](flight/PX4_LOCALIZATION_GATES.md) |
| `docs/OAKD_VISUAL_SLAM_RVIZ.md` | [docs/hardware/OAKD_VISUAL_SLAM_RVIZ.md](hardware/OAKD_VISUAL_SLAM_RVIZ.md) |
| `docs/DAMIAO_IMU_USB.md` | [docs/hardware/DAMIAO_IMU_USB.md](hardware/DAMIAO_IMU_USB.md) |
| `docs/PX4_USB_BENCH.md` | [docs/hardware/PX4_USB_BENCH.md](hardware/PX4_USB_BENCH.md) |
| `docs/PX4_MICRO_XRCE_DDS_SERIAL_SETUP.md` | [docs/hardware/PX4_MICRO_XRCE_DDS_SERIAL_SETUP.md](hardware/PX4_MICRO_XRCE_DDS_SERIAL_SETUP.md) |
| `docs/BT_PX4_DEVELOPMENT_PLAN.md` | [docs/development/BT_PX4_DEVELOPMENT_PLAN.md](development/BT_PX4_DEVELOPMENT_PLAN.md) |
| `docs/SIMULATION_DEVELOPMENT_PLAN.md` | [docs/development/SIMULATION_DEVELOPMENT_PLAN.md](development/SIMULATION_DEVELOPMENT_PLAN.md) |
| `docs/PX4_INTEGRATION_ROADMAP.md` | [docs/development/PX4_INTEGRATION_ROADMAP.md](development/PX4_INTEGRATION_ROADMAP.md) |
| `docs/PX4_NAVIGATION_STRATEGY.md` | [docs/development/PX4_NAVIGATION_STRATEGY.md](development/PX4_NAVIGATION_STRATEGY.md) |
| `docs/PX4_STATE_MACHINE_DESIGN.md` | [docs/development/PX4_STATE_MACHINE_DESIGN.md](development/PX4_STATE_MACHINE_DESIGN.md) |
| `docs/UAV_PROJECT_ASSESSMENT.md` | [docs/development/UAV_PROJECT_ASSESSMENT.md](development/UAV_PROJECT_ASSESSMENT.md) |
| `docs/ISAAC_ROS_COMPONENTS.md` | [docs/development/ISAAC_ROS_COMPONENTS.md](development/ISAAC_ROS_COMPONENTS.md) |
| `docs/warehouse-layout.svg` | [docs/simulation/warehouse-layout.svg](simulation/warehouse-layout.svg) |

## 脚本兼容路径

旧脚本名保留为符号链接，完整映射见 [脚本目录](../scripts/README.md#兼容入口)。如复制脚本到其他机器，应保留仓库目录结构及符号链接。
