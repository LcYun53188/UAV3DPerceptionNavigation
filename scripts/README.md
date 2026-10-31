# 工作区脚本总目录

[项目首页](../README.md) · [文档总目录](../docs/README.md) · [维护约定](../docs/MAINTENANCE.md)

所有命令从项目根目录运行。日常使用 `./scripts/sim.sh`；根目录只保留该入口、`with_venv.sh`、`with_px4_sim.sh` 和本目录说明。工具实现、兼容链接、本地测试与默认配置分别维护，新增文件放入对应子目录。

## 目录布局

| 目录 | 内容 |
| --- | --- |
| [simulation/](simulation/README.md) | 算法、PX4、VIO、光流、Pocket 与仿真验收工具 |
| `build/` | Shell／Python 构建器 |
| `setup/` | 系统依赖安装、第三方补丁与 LFS 准备 |
| `hardware/oakd/` | OAK-D 采集、定位审计与监视 |
| `hardware/lidar/` | MID360／FAST-LIO 启动 |
| `hardware/px4/` | 飞控 USB 台架和串口 Agent |
| `diagnostics/` | 导航状态检查、ROS bag 记录和 RViz |
| `config/` | 导航脚本默认参数与本地覆盖 |
| `tools/` | 文档、脚本目录与链接维护 |
| `support/` | 分类工具的公共模块路径支持 |
| [tests/](tests/README.md) | 版本化入口检查与 Git 忽略的 `local/` 独立回归 |
| [compat/](compat/README.md) | 集中保存历史脚本名的兼容链接和迁移表 |

旧 `scripts/<工具>` 已从根目录移出。优先使用下方规范路径；需要按旧文件名调用时使用 `scripts/compat/<工具>`。例如旧构建命令改用 `./scripts/build/build_px4_flight.sh`，或通过 `./scripts/compat/build_px4_flight.sh` 调用同一实现。`simulation/scripts/` 的三个历史场景入口继续保留。

## 环境与使用方式

| 工具类型 | 调用方式 | 说明 |
| --- | --- | --- |
| 仿真总入口 | `./scripts/sim.sh` 或 `./scripts/sim.sh menu` | 只选择光流纯手动或完整项目 |
| 配置启动 | `./scripts/sim.sh run manual/full [选项]` | 与菜单共用默认值和校验；支持 `--config`、`--save`、`--dry-run` |
| 会话操作 | `./scripts/sim.sh status/save/export-mesh/logs/stop` | 新终端自动绑定当前会话；地图操作要求落地上锁 |
| 内部诊断 | `scripts/simulation/` 分类工具 | 供专项回归使用，不再注册为公共仿真程序 |
| ROS／硬件 Python | `./scripts/with_venv.sh python scripts/hardware/<分类>/<工具>.py ...` | .venv、ROS 2 与 install_uav |
| PX4 Python | `./scripts/with_px4_sim.sh python scripts/simulation/px4/<工具>.py ...` | 隔离 PX4 消息环境；额外任务环境按工具手册加载 |
| 离线纯 Python | `python3 scripts/simulation/vio/analysis/<工具>.py ...` | 是否需要 NumPy 等依赖，以源码与手册为准 |
| Python 构建器 | `./scripts/with_venv.sh python scripts/build/<工具>.py ...` | PX4 构建器使用 `with_px4_sim.sh`；旧路径继续可用 |
| 构建／安装 Shell | `./scripts/build/<工具>.sh`／`./scripts/setup/<工具>.sh` | 安装脚本可能修改系统依赖，参数和前置条件见对应手册 |
| C++／头文件 | 由相应 Python 构建器消费 | 不作为可直接执行的命令 |

目录描述说明默认用途，具体模式和参数以源码、对应手册及工具帮助为准。在线 `check_*`、`audit_*` 和 `run_*` 中部分工具会启动会话、发仿真目标或注入故障；离线分析工具只读取已完成运行的数据。`--help` 仅在工具支持时使用，不能用批量执行脚本来验证语法。

## 推荐入口

- 总启动菜单、配置与脚本分工：[仿真启动规划](../docs/simulation/LAUNCHERS.md)。
- 算法建图、探索、地图包：[仿真控制手册](../docs/simulation/SIMULATION_CONTROL.md)。
- PX4、BT 与取消／降落：[本机 PX4](../simulation/px4/README.md)、[任务树](../src/uav_bt/README.md)。
- 虚拟 Pocket 面板与模式：[控件及 QGC 接入](../docs/flight/VIRTUAL_POCKET_PANEL.md)。
- 无 GNSS VIO 避障操作：[VIO／深度／EGO](../docs/flight/VIO_POCKET_AVOIDANCE.md)。
- 无 GNSS VIO 与光流：[VIO 验证](../docs/flight/VIO_HOVER.md)、[MTF-01](../docs/flight/MTF01_SIMULATION.md)。
- OAK-D 硬件：[定位验证](../docs/hardware/OAKD_VISUAL_SLAM_RVIZ.md)。
- 依赖补丁与 LFS：[Vendor 维护](../patches/vendor/README.md)。

<!-- BEGIN SCRIPT CATALOG -->

## 统一入口与环境

| 文件 | 用途 |
| --- | --- |
| [scripts/sim.sh](sim.sh) | 两个仿真程序：光流纯手动、完整项目；支持新终端会话操作。 |
| [scripts/with_px4_sim.sh](with_px4_sim.sh) | 加载隔离的 PX4 消息与仿真环境。 |
| [scripts/with_venv.sh](with_venv.sh) | 进入工作区，加载 .venv、系统 ROS 2 与 install_uav。 |

## 仿真启动实现

| 文件 | 用途 |
| --- | --- |
| [scripts/simulation/__init__.py](simulation/__init__.py) | Workspace simulation launchers; importing this package never starts a simulation. |
| [scripts/simulation/backends.py](simulation/backends.py) | 唯一的运行器与环境映射；保留旧入口、参数顺序、信号和退出码。 |
| [scripts/simulation/cleanup.py](simulation/cleanup.py) | 查找当前工作区的残留仿真进程；默认预览，--apply 分阶段清理。 |
| [scripts/simulation/cli.py](simulation/cli.py) | 两个完整仿真程序及当前会话操作。 |
| [scripts/simulation/compat.sh](simulation/compat.sh) | 历史启动脚本共用的兼容转发器，不维护场景或任务参数。 |
| [scripts/simulation/control.py](simulation/control.py) | Operate the current public simulation from a second terminal. |
| [scripts/simulation/control_ros.py](simulation/control_ros.py) | Map operations through the full project's guarded map-session services. |
| [scripts/simulation/environment.sh](simulation/environment.sh) | 集中加载 PX4／Mission／VIO 环境，再执行所选仿真运行器。 |
| [scripts/simulation/launch_isaac.sh](simulation/launch_isaac.sh) | Isaac Sim 4.5 UI 启动实现，复用已有安装与环境变量。 |
| [scripts/simulation/launch_scene.sh](simulation/launch_scene.sh) | Gazebo 场景与传感器桥启动实现，保持原单实例锁。 |
| [scripts/simulation/manual.py](simulation/manual.py) | Shared HID/panel ownership for the manual flow and full project programs. |
| [scripts/simulation/map_visualization.py](simulation/map_visualization.py) | Startup-only map display choices shared by the CLI and VIO runner. |
| [scripts/simulation/menu.py](simulation/menu.py) | 交互仿真菜单；配置与启动策略由 profiles 和 backends 统一维护。 |
| [scripts/simulation/policy.py](simulation/policy.py) | Workspace simulation policy: no satellite measurements or GNSS aiding. |
| [scripts/simulation/profiles.py](simulation/profiles.py) | 仿真配置注册表：菜单、命令行和 JSON 配置共用的选项与参数规则。 |
| [scripts/simulation/session.py](simulation/session.py) | Bind terminal operations to one owned simulation and its process lifetime. |
| [scripts/simulation/vio/__init__.py](simulation/vio/__init__.py) | Simulation tools grouped by responsibility. |

## 算法仿真与导航

| 文件 | 用途 |
| --- | --- |
| [scripts/simulation/algorithm/__init__.py](simulation/algorithm/__init__.py) | Simulation tools grouped by responsibility. |
| [scripts/simulation/algorithm/analyze_observation_grid.py](simulation/algorithm/analyze_observation_grid.py) | Offline ESDF evidence and ideal camera visibility; never produces a flight map. |
| [scripts/simulation/algorithm/check_continuous_navigation.py](simulation/algorithm/check_continuous_navigation.py) | Deterministic ROS planner/executor check with an incrementally observed corridor. |
| [scripts/simulation/algorithm/check_gazebo_algorithm_mission.py](simulation/algorithm/check_gazebo_algorithm_mission.py) | Owned C++ BT/MissionServer regression in the managed algorithm lab. |
| [scripts/simulation/algorithm/check_gazebo_autonomous.py](simulation/algorithm/check_gazebo_autonomous.py) | Bounded autonomous mapping trial with independent Gazebo geometry checks. |
| [scripts/simulation/algorithm/check_gazebo_expanded_goal.py](simulation/algorithm/check_gazebo_expanded_goal.py) | Expanded-world RViz goal regression with independent SDF geometry checks. |
| [scripts/simulation/algorithm/check_gazebo_exploration.py](simulation/algorithm/check_gazebo_exploration.py) | Online exploration regression for lab/expanded (sends a simulation goal). |
| [scripts/simulation/algorithm/check_gazebo_failures.py](simulation/algorithm/check_gazebo_failures.py) | Negative goals for the fixed uav_ego_lab scene; run only while stopped. |
| [scripts/simulation/algorithm/check_gazebo_goal.py](simulation/algorithm/check_gazebo_goal.py) | One goal regression in uav_ego_lab; ground-truth cylinder/wall clearance. |
| [scripts/simulation/algorithm/check_gazebo_map_loss.py](simulation/algorithm/check_gazebo_map_loss.py) | Fault injection for the dedicated Gazebo algorithm test process only. |
| [scripts/simulation/algorithm/check_gazebo_navigation_action.py](simulation/algorithm/check_gazebo_navigation_action.py) | Ordinary Action-client regression for the algorithm velocity-model profile. |
| [scripts/simulation/algorithm/sim_control.py](simulation/algorithm/sim_control.py) | Manage one Gazebo EGO/nvblox session in this workspace (Linux). |
| [scripts/simulation/algorithm/sim_ros_control.py](simulation/algorithm/sim_ros_control.py) | Bounded ROS operations for sim.sh; run via with_venv.sh. |
| [scripts/simulation/algorithm/survey_gazebo_map.py](simulation/algorithm/survey_gazebo_map.py) | Offline Gazebo depth survey (teleported camera poses, NOT flight validation). |

## PX4 运行与任务

| 文件 | 用途 |
| --- | --- |
| [scripts/simulation/px4/__init__.py](simulation/px4/__init__.py) | Simulation tools grouped by responsibility. |
| [scripts/simulation/px4/assess_px4_bt_suite.py](simulation/px4/assess_px4_bt_suite.py) | Read-only acceptance of four owned BT/SITL runs; never starts flight or writes FMU. |
| [scripts/simulation/px4/bt_flight_client.py](simulation/px4/bt_flight_client.py) | Owned BT subprocess and read-only root-result observer for SITL acceptance. |
| [scripts/simulation/px4/check_px4_interfaces.py](simulation/px4/check_px4_interfaces.py) | Compare every DDS-exported message and its nested types with pinned px4_msgs. |
| [scripts/simulation/px4/localization_selection_session.py](simulation/px4/localization_selection_session.py) | Ground-only source selector attached to owned joint SITL; no flight publishers. |
| [scripts/simulation/px4/no_gnss_flight_cases.py](simulation/px4/no_gnss_flight_cases.py) | Actual QGC/uinput -> PX4 acceptance stimuli, never fabricated FMU telemetry. |
| [scripts/simulation/px4/prepare_px4_depth_assets.py](simulation/px4/prepare_px4_depth_assets.py) | Same-aircraft ideal RGBD reference; fixed 5 degrees down, not OAK-D calibration. |
| [scripts/simulation/px4/prepare_px4_sim.py](simulation/px4/prepare_px4_sim.py) | Prepare locked SITL dependencies without changing existing checkouts. |
| [scripts/simulation/px4/px4_camera_mount_audit.py](simulation/px4/px4_camera_mount_audit.py) | Read-only Gazebo Scene pose audit against the advertised body optical TF. |
| [scripts/simulation/px4/px4_depth_audit.py](simulation/px4/px4_depth_audit.py) | Read-only audit of the PX4 reference depth stream (OakD-Lite model, not OAK-D Pro W calibration). |
| [scripts/simulation/px4/px4_mapping_tf.py](simulation/px4/px4_mapping_tf.py) | Disarmed-only mapping TF observer. No PX4 input publishers or truth poses. |
| [scripts/simulation/px4/px4_planning_audit.py](simulation/px4/px4_planning_audit.py) | Read-only real-map planning graph audit; never submits a goal or FMU input. |
| [scripts/simulation/px4/px4_vision_audit.py](simulation/px4/px4_vision_audit.py) | Synthetic EV input -> actual PX4 EKF fusion audit, disarmed only. |
| [scripts/simulation/px4/run_px4_flight_tasks.py](simulation/px4/run_px4_flight_tasks.py) | Exercise real ExecuteMission against owned x500 SITL, with independent truth. |
| [scripts/simulation/px4/run_px4_sitl_smoke.py](simulation/px4/run_px4_sitl_smoke.py) | Owned, disarmed x500 smoke session. Run with scripts/with_px4_sim.sh. |
| [scripts/simulation/px4/select_localization.py](simulation/px4/select_localization.py) | One identity-bound ground source selection request; no Pocket dependency. |

## 光流与测距仿真

| 文件 | 用途 |
| --- | --- |
| [scripts/simulation/flow/__init__.py](simulation/flow/__init__.py) | Simulation tools grouped by responsibility. |
| [scripts/simulation/flow/mtf01_audit.py](simulation/flow/mtf01_audit.py) | Assess actual PX4 uORB receipts for the isolated flow/range reference. |
| [scripts/simulation/flow/mtf01_flow_bridge.py](simulation/flow/mtf01_flow_bridge.py) | Image-derived flow for a fixed +90 degree pitch camera, no truth input. |
| [scripts/simulation/flow/mtf01_range_bridge.py](simulation/flow/mtf01_range_bridge.py) | Single-ray Gazebo range -> PX4, fixed down-facing body installation only. |
| [scripts/simulation/flow/prepare_mtf01_assets.py](simulation/flow/prepare_mtf01_assets.py) | MTF-01 reference geometry: rendered down-facing camera and GPU single ray. |
| [scripts/simulation/flow/run_flow_manual_tasks.py](simulation/flow/run_flow_manual_tasks.py) | Owned no-EV/no-GNSS native Position test with a simulated RC receiver. |
| [scripts/simulation/flow/run_fusion_switch_sitl.py](simulation/flow/run_fusion_switch_sitl.py) | Disarmed synthetic EV + rendered flow/range + actual PX4 selection audit. |
| [scripts/simulation/flow/run_px4_mtf01.py](simulation/flow/run_px4_mtf01.py) | 默认未解锁光流／测距审计；显式 --flight 支持独立光流飞行仿真。 |

## Pocket 输入与操作会话

| 文件 | 用途 |
| --- | --- |
| [scripts/simulation/pocket/__init__.py](simulation/pocket/__init__.py) | Simulation tools grouped by responsibility. |
| [scripts/simulation/pocket/bt_operator_intent.py](simulation/pocket/bt_operator_intent.py) | Send one operator intent to the current isolated SITL BT, without a USB radio. |
| [scripts/simulation/pocket/pocket_input.py](simulation/pocket/pocket_input.py) | Read-only Pocket identity checks before starting owned simulation processes. |
| [scripts/simulation/pocket/pocket_operator_session.py](simulation/pocket/pocket_operator_session.py) | Owned Pocket SITL control with explicit legacy USB or UAV AETR profiles. |
| [scripts/simulation/pocket/pocket_panel.py](simulation/pocket/pocket_panel.py) | Desktop RadioMaster Pocket UAV panel: AETR channels and optional uinput output. |
| [scripts/simulation/pocket/pocket_panel_model.py](simulation/pocket/pocket_panel_model.py) | UAV Model 5 channel model; independent of Tk, ROS and Linux input. |
| [scripts/simulation/pocket/pocket_switches.py](simulation/pocket/pocket_switches.py) | Pocket SA/SB/SD stable edges, bound to the inspected USB axes. |
| [scripts/simulation/pocket/pocket_uav_switches.py](simulation/pocket/pocket_uav_switches.py) | SB/SC flight matrix shared by virtual AETR and physical Pocket CH5--8. |
| [scripts/simulation/pocket/pocket_validation_input.py](simulation/pocket/pocket_validation_input.py) | Owned uinput stimulus for explicit computer-only flight acceptance cases. |
| [scripts/simulation/pocket/pocket_virtual_joystick.py](simulation/pocket/pocket_virtual_joystick.py) | Linux uinput Pocket transport using only the standard library. |
| [scripts/simulation/pocket/qgc_calibration.py](simulation/pocket/qgc_calibration.py) | Preserve device-specific QGC calibration and repair signed Pocket throttle ranges. |
| [scripts/simulation/pocket/usb_bt_operator.py](simulation/pocket/usb_bt_operator.py) | Explicit USB switch mapping to the bound BT; sticks remain QGC's input. |
| [scripts/simulation/pocket/usb_joystick.py](simulation/pocket/usb_joystick.py) | Read Linux USB joystick events; inspection is the default, no flight output. |

## VIO 运行、传感器与建图

| 文件 | 用途 |
| --- | --- |
| [scripts/simulation/vio/runtime/__init__.py](simulation/vio/runtime/__init__.py) | Simulation tools grouped by responsibility. |
| [scripts/simulation/vio/runtime/mission_person_observation.py](simulation/vio/runtime/mission_person_observation.py) | Task-scoped automatic person episodes and bounded map-frame trajectory history. |
| [scripts/simulation/vio/runtime/mission_person_observer.py](simulation/vio/runtime/mission_person_observer.py) | ROS task observer and map trajectory display for automatic follow shadow episodes. |
| [scripts/simulation/vio/runtime/obstacle_course.py](simulation/vio/runtime/obstacle_course.py) | Deterministic four-zone course, shared geometry and bounded PX4 recipes. |
| [scripts/simulation/vio/runtime/person_follow_backend.py](simulation/vio/runtime/person_follow_backend.py) | Identity-bound RGB target selection and 3-D follow preview; no motion commands. |
| [scripts/simulation/vio/runtime/person_follow_client.py](simulation/vio/runtime/person_follow_client.py) | Nonblocking UI client for selection/preview; execution readiness comes from backend. |
| [scripts/simulation/vio/runtime/person_follow_core.py](simulation/vio/runtime/person_follow_core.py) | Selected-person preview lifecycle. No flight authority or navigation output. |
| [scripts/simulation/vio/runtime/prepare_vio_sensor_assets.py](simulation/vio/runtime/prepare_vio_sensor_assets.py) | Generate an explicit stereo/IMU reference rig and textured SITL test world. |
| [scripts/simulation/vio/runtime/px4_vio_flight_session.py](simulation/vio/runtime/px4_vio_flight_session.py) | Continuous EV writer for owned warehouse SITL; no command/setpoint publishers. |
| [scripts/simulation/vio/runtime/px4_vio_motion_audit.py](simulation/vio/runtime/px4_vio_motion_audit.py) | Independent timestamp-matched VIO error audit; truth is never fed to VIO. |
| [scripts/simulation/vio/runtime/px4_vio_pose_audit.py](simulation/vio/runtime/px4_vio_pose_audit.py) | Observe normalized SDK poses and the reset retirement boundary, without FMU writes. |
| [scripts/simulation/vio/runtime/px4_vio_real_fusion.py](simulation/vio/runtime/px4_vio_real_fusion.py) | Owned, disarmed same-aircraft cuVSLAM -> PX4 EV audit. No synthetic inputs. |
| [scripts/simulation/vio/runtime/px4_vio_sensor_audit.py](simulation/vio/runtime/px4_vio_sensor_audit.py) | Observe physical simulated stereo/IMU and actual cuVSLAM outputs, never inject pose. |
| [scripts/simulation/vio/runtime/record_obstacle_course_truth.py](simulation/vio/runtime/record_obstacle_course_truth.py) | Passive Gazebo truth recorder for course audits; publishes no control/TF data. |
| [scripts/simulation/vio/runtime/record_vio_motion_build.py](simulation/vio/runtime/record_vio_motion_build.py) | Bind a successfully built fixture to its source and binary, not an old cache. |
| [scripts/simulation/vio/runtime/record_vio_same_aircraft_build.py](simulation/vio/runtime/record_vio_same_aircraft_build.py) | 记录同机运动插件源码与二进制回执。 |
| [scripts/simulation/vio/runtime/run_cuvslam_pose_transport.py](simulation/vio/runtime/run_cuvslam_pose_transport.py) | Synthetic DDS tests of pose warmup and reset retirement, never real VIO/flight. |
| [scripts/simulation/vio/runtime/run_px4_vio_sensors.py](simulation/vio/runtime/run_px4_vio_sensors.py) | Owned official PX4/Gazebo/QGC VIO audits and explicit warehouse BT flight. |
| [scripts/simulation/vio/runtime/run_vio_node_smoke.py](simulation/vio/runtime/run_vio_node_smoke.py) | Check real cuVSLAM node/CUDA loading without camera or FMU publishers. |
| [scripts/simulation/vio/runtime/run_vio_transport_smoke.py](simulation/vio/runtime/run_vio_transport_smoke.py) | Isolated DDS contract test with synthetic odometry/status, never real VIO. |
| [scripts/simulation/vio/runtime/vio_chain_mapping.py](simulation/vio/runtime/vio_chain_mapping.py) | Attach actual depth to the VIO aircraft and start the owned mapping graph. |
| [scripts/simulation/vio/runtime/vio_component.launch.py](simulation/vio/runtime/vio_component.launch.py) | Owned single-thread SDK container, using the same parameters and source topics. |
| [scripts/simulation/vio/runtime/vio_covariance_consistency.py](simulation/vio/runtime/vio_covariance_consistency.py) | Truth-observer-only six-dimensional consistency rejection test. |
| [scripts/simulation/vio/runtime/vio_cpu_affinity.py](simulation/vio/runtime/vio_cpu_affinity.py) | Explicit process-local scheduling comparison; no system-wide CPU changes. |
| [scripts/simulation/vio/runtime/vio_evidence.py](simulation/vio/runtime/vio_evidence.py) | Preserve invalid SDK values in raw arrays and explicit nullable diagnostics. |
| [scripts/simulation/vio/runtime/vio_gc_pause_audit.py](simulation/vio/runtime/vio_gc_pause_audit.py) | Passive Python GC pause evidence; never changes thresholds or GC behavior. |
| [scripts/simulation/vio/runtime/vio_live_evidence.py](simulation/vio/runtime/vio_live_evidence.py) | Freeze completed mission evidence in a process independent of ROS callbacks. |
| [scripts/simulation/vio/runtime/vio_map_startup.py](simulation/vio/runtime/vio_map_startup.py) | Ground-only startup stability before enabling depth integration. |
| [scripts/simulation/vio/runtime/vio_mtf01_joint.py](simulation/vio/runtime/vio_mtf01_joint.py) | Disarmed joint observation only: existing VIO contract plus independent MTF aids. |
| [scripts/simulation/vio/runtime/vio_observer_gc.py](simulation/vio/runtime/vio_observer_gc.py) | Keep automatic cyclic GC outside a bounded owned observation/control window. |
| [scripts/simulation/vio/runtime/vio_pose_window.py](simulation/vio/runtime/vio_pose_window.py) | Source-rate evidence uses ROS/sample time; retirement uses monotonic time. |
| [scripts/simulation/vio/runtime/vio_render_device.py](simulation/vio/runtime/vio_render_device.py) | Explicit renderer selection for owned Gazebo only; never changes machine settings. |
| [scripts/simulation/vio/runtime/vio_same_aircraft_dynamic.py](simulation/vio/runtime/vio_same_aircraft_dynamic.py) | Explicit disarmed x500 physics fixture; this policy never admits flight. |
| [scripts/simulation/vio/runtime/vio_sensor_quality.py](simulation/vio/runtime/vio_sensor_quality.py) | Numerical acceptance of observed sensor timestamps; independent of ROS callbacks. |
| [scripts/simulation/vio/runtime/vio_sensor_transforms.py](simulation/vio/runtime/vio_sensor_transforms.py) | Sensor mount rotations shared by generated geometry and runtime TF. |
| [scripts/simulation/vio/runtime/vio_sim_clock.py](simulation/vio/runtime/vio_sim_clock.py) | Simulation-only clock from unchanged Gazebo IMU physics timestamps. |
| [scripts/simulation/vio/runtime/vio_stereo_ui.py](simulation/vio/runtime/vio_stereo_ui.py) | Camera monitor: RGB detections, AR planned route, stereo and diagnostics. |
| [scripts/simulation/vio/runtime/vio_ui_lifecycle.py](simulation/vio/runtime/vio_ui_lifecycle.py) | Viewer retention requires current independent PX4 ground telemetry. |
| [scripts/simulation/vio/runtime/vio_warehouse_scene.py](simulation/vio/runtime/vio_warehouse_scene.py) | Deterministic warehouse geometry and independent free-volume checks. |
| [scripts/simulation/vio/runtime/vision_monitor.py](simulation/vio/runtime/vision_monitor.py) | RGB detection / AR planning display with stereo previews; no flight publishers. |
| [scripts/simulation/vio/runtime/vision_overlay.py](simulation/vio/runtime/vision_overlay.py) | Pure geometry and freshness checks for the camera operator display. |

## VIO SDK 与原生辅助源码

| 文件 | 用途 |
| --- | --- |
| [scripts/simulation/vio/sdk/__init__.py](simulation/vio/sdk/__init__.py) | Simulation tools grouped by responsibility. |
| [scripts/simulation/vio/sdk/openvins_covariance_replay.cpp](simulation/vio/sdk/openvins_covariance_replay.cpp) | OpenVINS 历史候选原生协方差回放诊断。 |
| [scripts/simulation/vio/sdk/openvins_pose_covariance.hpp](simulation/vio/sdk/openvins_pose_covariance.hpp) | OpenVINS 历史候选位姿协方差辅助实现。 |
| [scripts/simulation/vio/sdk/openvins_ros_node.cpp](simulation/vio/sdk/openvins_ros_node.cpp) | OpenVINS 历史候选 ROS 接口源码；当前路线暂停。 |
| [scripts/simulation/vio/sdk/prepare_openvins_config.py](simulation/vio/sdk/prepare_openvins_config.py) | Generate OpenVINS calibration from physical mount TF and camera intrinsics. |
| [scripts/simulation/vio/sdk/prepare_vio_replay_calibration.py](simulation/vio/sdk/prepare_vio_replay_calibration.py) | Resolve SDK15 dump body-reference ambiguity using physical mount calibration. |
| [scripts/simulation/vio/sdk/vio_sdk16_acceptance.py](simulation/vio/sdk/vio_sdk16_acceptance.py) | Fail-closed VIO candidate acceptance receipt; publishes no ROS controls. |
| [scripts/simulation/vio/sdk/vio_sdk16_conditional_covariance.hpp](simulation/vio/sdk/vio_sdk16_conditional_covariance.hpp) | SDK16 条件位姿协方差导出辅助实现。 |
| [scripts/simulation/vio/sdk/vio_sdk16_covariance.hpp](simulation/vio/sdk/vio_sdk16_covariance.hpp) | SDK16 协方差导出共用实现。 |
| [scripts/simulation/vio/sdk/vio_sdk16_marginal_covariance.hpp](simulation/vio/sdk/vio_sdk16_marginal_covariance.hpp) | SDK16 边缘位姿协方差导出辅助实现。 |
| [scripts/simulation/vio/sdk/vio_sdk16_state_audit.py](simulation/vio/sdk/vio_sdk16_state_audit.py) | Observe exact SDK16 Track initialization state; no estimator/control writes. |
| [scripts/simulation/vio/sdk/vio_sdk16_world_covariance.hpp](simulation/vio/sdk/vio_sdk16_world_covariance.hpp) | SDK16 世界坐标系协方差转换辅助实现。 |
| [scripts/simulation/vio/sdk/vio_sdk_covariance_replay.cpp](simulation/vio/sdk/vio_sdk_covariance_replay.cpp) | 原生 SDK 位姿／协方差回放诊断。 |
| [scripts/simulation/vio/sdk/vio_sdk_parameters.py](simulation/vio/sdk/vio_sdk_parameters.py) | Parameters compatible with the reviewed cuVSLAM 15 stream sequencer. |
| [scripts/simulation/vio/sdk/vio_sdk_profile.py](simulation/vio/sdk/vio_sdk_profile.py) | Optional CUDA/NVTX trace; profiling is never source qualification. |
| [scripts/simulation/vio/sdk/vio_sdk_runtime.py](simulation/vio/sdk/vio_sdk_runtime.py) | Read-only SDK parameter receipt; never changes estimator parameters. |
| [scripts/simulation/vio/sdk/vio_sdk_selection.py](simulation/vio/sdk/vio_sdk_selection.py) | One explicit VIO default; missing or drifted SDK16 fails without fallback. |
| [scripts/simulation/vio/sdk/vio_sdk_target.py](simulation/vio/sdk/vio_sdk_target.py) | Give the profiled SDK its own session so SIGINT cannot cancel Nsight export. |

## VIO 离线分析、诊断与验收

| 文件 | 用途 |
| --- | --- |
| [scripts/simulation/vio/analysis/__init__.py](simulation/vio/analysis/__init__.py) | Simulation tools grouped by responsibility. |
| [scripts/simulation/vio/analysis/analyze_sdk_covariance.py](simulation/vio/analysis/analyze_sdk_covariance.py) | Offline covariance evidence; no change to source quality or flight admission. |
| [scripts/simulation/vio/analysis/analyze_vio_chain_flight.py](simulation/vio/analysis/analyze_vio_chain_flight.py) | Offline receipt-time trajectory screening; never a control/estimator input. |
| [scripts/simulation/vio/analysis/analyze_vio_chain_map.py](simulation/vio/analysis/analyze_vio_chain_map.py) | Read-only collision screening of an actual flight's frozen depth map. |
| [scripts/simulation/vio/analysis/analyze_vio_chain_native.py](simulation/vio/analysis/analyze_vio_chain_native.py) | Screen actual PX4 native fusion in one completed flight's ULog. |
| [scripts/simulation/vio/analysis/analyze_vio_final_rest.py](simulation/vio/analysis/analyze_vio_final_rest.py) | Separate pose marginal diagnostics in an already frozen final-rest window. |
| [scripts/simulation/vio/analysis/analyze_vio_flight_images.py](simulation/vio/analysis/analyze_vio_flight_images.py) | Offline image structure diagnostics. Candidate corners are NOT cuVSLAM inliers. |
| [scripts/simulation/vio/analysis/analyze_vio_physical_replay.py](simulation/vio/analysis/analyze_vio_physical_replay.py) | Recompute physical-reference replay diagnostics, never flight acceptance. |
| [scripts/simulation/vio/analysis/analyze_vio_sdk16_marginal.py](simulation/vio/analysis/analyze_vio_sdk16_marginal.py) | Validate repaired SDK exports and same-trajectory covariance counterfactuals. |
| [scripts/simulation/vio/analysis/analyze_vio_startup.py](simulation/vio/analysis/analyze_vio_startup.py) | Read-only stationary startup comparison; never grants flight qualification. |
| [scripts/simulation/vio/analysis/assess_obstacle_course.py](simulation/vio/analysis/assess_obstacle_course.py) | Check ordered waypoints, directed apertures and swept native Gazebo truth. |
| [scripts/simulation/vio/analysis/assess_px4_real_vio_fusion.py](simulation/vio/analysis/assess_px4_real_vio_fusion.py) | Independently inspect saved source/output/EKF chronology, without ROS or VIO code. |
| [scripts/simulation/vio/analysis/assess_vio_motion_evidence.py](simulation/vio/analysis/assess_vio_motion_evidence.py) | Replay archived reference-motion error independently, without ROS or live graph. |
| [scripts/simulation/vio/analysis/assess_vio_navigation.py](simulation/vio/analysis/assess_vio_navigation.py) | Accept a completed simulated mission against an explicit positioning limit. |
| [scripts/simulation/vio/analysis/assess_vio_sdk_candidate.py](simulation/vio/analysis/assess_vio_sdk_candidate.py) | Native candidate diagnostics, independent of ROS and flight admission. |
| [scripts/simulation/vio/analysis/assess_vio_sdk_profile.py](simulation/vio/analysis/assess_vio_sdk_profile.py) | Read Nsight's exported SQLite; overlap is evidence, not causal attribution. |
| [scripts/simulation/vio/analysis/assess_vio_static_avoidance.py](simulation/vio/analysis/assess_vio_static_avoidance.py) | Audit actual EGO/truth evidence; an empty-path mission cannot pass avoidance. |
| [scripts/simulation/vio/analysis/assess_vio_timing.py](simulation/vio/analysis/assess_vio_timing.py) | Compare original-sample timing across owned publishers; no control/admission effects. |
| [scripts/simulation/vio/analysis/assess_vio_warehouse.py](simulation/vio/analysis/assess_vio_warehouse.py) | Read-only warehouse scene/source qualification; never authorizes flight. |
| [scripts/simulation/vio/analysis/assess_warehouse_flight.py](simulation/vio/analysis/assess_warehouse_flight.py) | Read-only actual warehouse BT closure audit; low flight never qualifies 1.5 m. |
| [scripts/simulation/vio/analysis/audit_vio_sdk_dump.py](simulation/vio/analysis/audit_vio_sdk_dump.py) | Read actual SDK-consumed input dump; diagnostic receipt, never flight evidence. |
| [scripts/simulation/vio/analysis/audit_vio_truth_topics.py](simulation/vio/analysis/audit_vio_truth_topics.py) | Read-only Gazebo topic graph proof for an active owned dynamic audit. |
| [scripts/simulation/vio/analysis/diagnose_vio_chain.py](simulation/vio/analysis/diagnose_vio_chain.py) | Read-only VIO transport, EKF noise and control-lag diagnosis of a completed run. |
| [scripts/simulation/vio/analysis/diagnose_vio_deadline.py](simulation/vio/analysis/diagnose_vio_deadline.py) | Locate original-sample deadline handover; read-only, never reauthorize a source. |
| [scripts/simulation/vio/analysis/diagnose_vio_motion.py](simulation/vio/analysis/diagnose_vio_motion.py) | Read-only per-axis error and covariance chronology; single initial alignment. |
| [scripts/simulation/vio/analysis/diagnose_vio_sdk16_errors.py](simulation/vio/analysis/diagnose_vio_sdk16_errors.py) | SDK16 error diagnostics against observer-only truth, with no trajectory fit. |
| [scripts/simulation/vio/analysis/diagnose_vio_sdk16_information.py](simulation/vio/analysis/diagnose_vio_sdk16_information.py) | Separate SDK conditional export correctness from global covariance validity. |
| [scripts/simulation/vio/analysis/diagnose_vio_sdk16_replay.py](simulation/vio/analysis/diagnose_vio_sdk16_replay.py) | Same-pixel SDK16 VIO/VO diagnostic; one initial yaw/position anchor only. |
| [scripts/simulation/vio/analysis/freeze_vio_chain.py](simulation/vio/analysis/freeze_vio_chain.py) | Freeze completed experimental chain using verified consumed-input snapshots. |
| [scripts/simulation/vio/analysis/freeze_vio_run.py](simulation/vio/analysis/freeze_vio_run.py) | Freeze a completed run and its exact consumed inputs before further edits. |
| [scripts/simulation/vio/analysis/plot_vio_chain_covariance.py](simulation/vio/analysis/plot_vio_chain_covariance.py) | Plot frozen consistency measurements; does not modify or calibrate covariance. |
| [scripts/simulation/vio/analysis/plot_vio_sdk16_errors.py](simulation/vio/analysis/plot_vio_sdk16_errors.py) | Plot measured SDK16 error and covariance, without altering estimates. |

## 仿真环境与协议验收

| 文件 | 用途 |
| --- | --- |
| [scripts/simulation/validation/__init__.py](simulation/validation/__init__.py) | Simulation tools grouped by responsibility. |
| [scripts/simulation/validation/check_sim_environment.py](simulation/validation/check_sim_environment.py) | Read-only S0 dependency audit. Runtime topic/TF/clock readiness is not asserted. |
| [scripts/simulation/validation/run_mission_protocol_smoke.py](simulation/validation/run_mission_protocol_smoke.py) | Owned ROS Action/service protocol smoke. Synthetic data; no vehicle I/O. |
| [scripts/simulation/validation/run_person_localization.py](simulation/validation/run_person_localization.py) | Owned Gazebo RGB-D person test with known static camera pose; no PX4 or flight. |
| [scripts/simulation/validation/run_planning_context_smoke.py](simulation/validation/run_planning_context_smoke.py) | Real EGO + context bridge DDS regression on a synthetic ESDF, no flight outputs. |
| [scripts/simulation/validation/run_sim_scenario.py](simulation/validation/run_sim_scenario.py) | S0 host regression runner with immutable configuration and bounded subprocesses. |
| [scripts/simulation/validation/sim_validation.py](simulation/validation/sim_validation.py) | Shared S0 artifact contracts. No ROS imports or simulation side effects. |

## OAK-D 硬件采集与观测

| 文件 | 用途 |
| --- | --- |
| [scripts/hardware/oakd/__init__.py](hardware/oakd/__init__.py) | Workspace tools grouped by responsibility. |
| [scripts/hardware/oakd/audit_oakd_vio_hardware.py](hardware/oakd/audit_oakd_vio_hardware.py) | Collect raw OAK-D VIO evidence in an isolated ROS domain, without flight nodes. |
| [scripts/hardware/oakd/measure_oakd_vio_performance.sh](hardware/oakd/measure_oakd_vio_performance.sh) | 采集 OAK-D VIO 性能指标。 |
| [scripts/hardware/oakd/oakd_vio_motion_ui.py](hardware/oakd/oakd_vio_motion_ui.py) | Read-only hardware stereo/trajectory UI and operator-marked motion capture. |
| [scripts/hardware/oakd/run_vio_monitor.sh](hardware/oakd/run_vio_monitor.sh) | 启动 VIO 监视器。 |

## MID360 与 LIO 硬件

| 文件 | 用途 |
| --- | --- |
| [scripts/hardware/lidar/__init__.py](hardware/lidar/__init__.py) | Workspace tools grouped by responsibility. |
| [scripts/hardware/lidar/run_mid360_fastlio_rviz.sh](hardware/lidar/run_mid360_fastlio_rviz.sh) | 启动 MID360、FAST-LIO 与 RViz。 |

## PX4 台架与串口通信

| 文件 | 用途 |
| --- | --- |
| [scripts/hardware/px4/__init__.py](hardware/px4/__init__.py) | Workspace tools grouped by responsibility. |
| [scripts/hardware/px4/inspect_px4_usb.py](hardware/px4/inspect_px4_usb.py) | Passive USB MAVLink capture. Never sends heartbeat, requests or commands. |
| [scripts/hardware/px4/run_px4_microxrce.sh](hardware/px4/run_px4_microxrce.sh) | 启动 PX4 Micro XRCE-DDS Agent。 |

## 导航诊断与记录

| 文件 | 用途 |
| --- | --- |
| [scripts/diagnostics/__init__.py](diagnostics/__init__.py) | Workspace tools grouped by responsibility. |
| [scripts/diagnostics/check_nav_stack.sh](diagnostics/check_nav_stack.sh) | 检查导航图与输入状态。 |
| [scripts/diagnostics/nav_debug_lib.sh](diagnostics/nav_debug_lib.sh) | 导航诊断脚本共用的环境与话题配置。 |
| [scripts/diagnostics/record_nav_debug_bag.sh](diagnostics/record_nav_debug_bag.sh) | 采集导航诊断 ROS bag。 |
| [scripts/diagnostics/run_rviz_nav.sh](diagnostics/run_rviz_nav.sh) | 启动导航 RViz。 |

## 目录维护

| 文件 | 用途 |
| --- | --- |
| [scripts/tools/__init__.py](tools/__init__.py) | Workspace tools grouped by responsibility. |
| [scripts/tools/maintain_catalogs.py](tools/maintain_catalogs.py) | Generate workspace catalogs and check local documentation/script syntax without running tools. |

## 脚本公共路径支持

| 文件 | 用途 |
| --- | --- |
| [scripts/support/__init__.py](support/__init__.py) | Workspace tools grouped by responsibility. |
| [scripts/support/paths.py](support/paths.py) | Locate workspace tool modules without flattening their files into scripts/. |

## 脚本默认配置

| 文件 | 用途 |
| --- | --- |
| [scripts/config/nav_launch.env](config/nav_launch.env) | 导航启动的公共环境变量默认值。 |

## 构建

| 文件 | 用途 |
| --- | --- |
| [scripts/build/build_algorithm_sim.sh](build/build_algorithm_sim.sh) | 构建 EGO、nvblox、算法导航与显示插件。 |
| [scripts/build/build_livox_sdk2.sh](build/build_livox_sdk2.sh) | 构建 Livox SDK2。 |
| [scripts/build/build_navigation_actions.sh](build/build_navigation_actions.sh) | 增量构建第一方导航 Action、Mission 和 BT。 |
| [scripts/build/build_nvidia_3d_nav_deps.sh](build/build_nvidia_3d_nav_deps.sh) | 构建 NVIDIA 三维导航依赖。 |
| [scripts/build/build_openvins_candidate.py](build/build_openvins_candidate.py) | Build the pinned OpenVINS candidate in its own prefix; no production install. |
| [scripts/build/build_px4_flight.sh](build/build_px4_flight.sh) | 构建隔离的任务、BT 与 PX4 飞行后端。 |
| [scripts/build/build_px4_sim.sh](build/build_px4_sim.sh) | 准备并构建锁定的 PX4 SITL、Agent 与消息包。 |
| [scripts/build/build_px4_vio.py](build/build_px4_vio.py) | Separate SITL telemetry build using the pinned upstream DDS generator. |
| [scripts/build/build_px4_vio.sh](build/build_px4_vio.sh) | 调用独立 PX4 VIO 遥测构建器。 |
| [scripts/build/build_uav_stack.sh](build/build_uav_stack.sh) | 使用 build_uav／install_uav 构建硬件感知与定位栈。 |
| [scripts/build/build_vio_motion.sh](build/build_vio_motion.sh) | 构建独立 VIO 运动载台并记录构建回执。 |
| [scripts/build/build_vio_node.sh](build/build_vio_node.sh) | 构建当前默认 SDK16 ROS VIO 节点。 |
| [scripts/build/build_vio_same_aircraft_motion.sh](build/build_vio_same_aircraft_motion.sh) | 构建同机 VIO 运动插件并记录回执。 |
| [scripts/build/build_vio_sdk16_default.py](build/build_vio_sdk16_default.py) | Build default SDK16 VIO, preserving optimization and the SDK15 rollback. |
| [scripts/build/build_vio_sdk16_marginal_candidate.py](build/build_vio_sdk16_marginal_candidate.py) | Isolated, source-pinned SDK16 covariance export repair; never installs SDK15. |
| [scripts/build/build_vio_sdk16_ros_candidate.py](build/build_vio_sdk16_ros_candidate.py) | Rebuild the disarmed SDK16 ROS candidate without changing installed SDK15. |
| [scripts/build/build_vio_sdk16_schur64.py](build/build_vio_sdk16_schur64.py) | Isolated double precision Schur candidate; never replaces the default SDK. |
| [scripts/build/build_vio_sdk16_schur_diagnostic.py](build/build_vio_sdk16_schur_diagnostic.py) | Isolated passive Hessian capture; never changes the default SDK or optimizer. |
| [scripts/build/build_vio_sdk_candidate.py](build/build_vio_sdk_candidate.py) | Build a pinned native SDK candidate in cache, without installing ROS or PX4. |

## 安装与依赖准备

| 文件 | 用途 |
| --- | --- |
| [scripts/setup/apply_vendor_patches.sh](setup/apply_vendor_patches.sh) | 调用 vendor 补丁应用／精确校验工具。 |
| [scripts/setup/install_cuda_13_2_stack.sh](setup/install_cuda_13_2_stack.sh) | 安装 CUDA 13.2；执行前阅读安装文档。 |
| [scripts/setup/install_nvidia_3d_nav_rosdeps.sh](setup/install_nvidia_3d_nav_rosdeps.sh) | 安装 NVIDIA ROS 依赖。 |
| [scripts/setup/install_vendor_lfs_assets.sh](setup/install_vendor_lfs_assets.sh) | 准备或校验 vendor LFS 资产。 |
| [scripts/setup/install_vpi_4_stack.sh](setup/install_vpi_4_stack.sh) | 安装 VPI 4；执行前核对环境。 |
| [scripts/setup/prepare_ego_vendor.sh](setup/prepare_ego_vendor.sh) | 准备锁定的 EGO 上游源码并应用受管补丁。 |
| [scripts/setup/vendor_lfs.py](setup/vendor_lfs.py) | Prepare and check LFS working files in the workspace and its submodules. |
| [scripts/setup/vendor_patches.py](setup/vendor_patches.py) | Validate vendor trees against pinned commits using disposable Git indexes. |

## 运行与交互

| 文件 | 用途 |
| --- | --- |

## 场景、标定与资产准备

| 文件 | 用途 |
| --- | --- |

## 在线检查与故障回归

| 文件 | 用途 |
| --- | --- |

## 离线分析、验收与归档

| 文件 | 用途 |
| --- | --- |

## 共享实现与原生辅助源码

| 文件 | 用途 |
| --- | --- |
| [scripts/hardware/__init__.py](hardware/__init__.py) | Workspace tools grouped by responsibility. |

## 包内与场景启动脚本

| 文件 | 用途 |
| --- | --- |
| [src/oakd_perception/scripts/run_oakd_active_max.sh](../src/oakd_perception/scripts/run_oakd_active_max.sh) | 启动 OAK-D 主动双目强度配置。 |
| [src/oakd_perception/scripts/run_oakd_balance.sh](../src/oakd_perception/scripts/run_oakd_balance.sh) | 启动 OAK-D 均衡深度配置。 |
| [src/oakd_perception/scripts/run_oakd_indoor.sh](../src/oakd_perception/scripts/run_oakd_indoor.sh) | 启动 OAK-D 室内深度配置。 |
| [src/oakd_perception/scripts/run_oakd_outdoor.sh](../src/oakd_perception/scripts/run_oakd_outdoor.sh) | 启动 OAK-D 室外深度配置。 |
| [src/uav_bringup/scripts/gazebo_odometry_velocity.py](../src/uav_bringup/scripts/gazebo_odometry_velocity.py) | 由 Gazebo／VIO 位姿计算平滑三维机体系速度，只修改 twist。 |

## 兼容入口

历史脚本名统一放在 [compat/](compat/README.md)，完整映射在该目录维护；根目录不再平铺兼容链接。

<!-- END SCRIPT CATALOG -->

## 本地独立回归与包内测试

`scripts/tests/local/test_*` 由 Git 忽略，不写入上面的版本化目录；包内测试保留在 `src/<package>/test/`。本机可列出全部独立测试：

```bash
rg --files scripts -g 'test_*.py' | sort
```

运行相关测试时按已有环境选择范围；ROS 工作区常需禁用 pytest 的自动插件加载：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./scripts/with_venv.sh python -m pytest <相关测试路径>
```

## 更新与静态检查

```bash
python3 scripts/tools/maintain_catalogs.py --write
python3 scripts/tools/maintain_catalogs.py --check
```

自动区域由工具维护，新增脚本可在源码首行 docstring 描述用途；Shell 和原生辅助源码的说明维护在 `maintain_catalogs.py`。检查只核对目录、项目内 Markdown 链接／锚点与 Python／Bash 语法，不启动被列出的工具。

`sim.sh px4-vio-pocket --sdk-schur64 --vio-primary --device <实际虚拟摇杆路径>` 启动未解锁的 VIO／深度／EGO 操作会话；`assess_vio_static_avoidance.py <自动验收运行目录>` 检查真实静态绕障证据，见 [操作文档](../docs/flight/VIO_POCKET_AVOIDANCE.md)。
