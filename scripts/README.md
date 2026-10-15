# 工作区脚本总目录

[项目首页](../README.md) · [文档总目录](../docs/README.md) · [维护约定](../docs/MAINTENANCE.md)

所有命令从项目根目录运行。构建脚本位于 `scripts/build/`，安装与依赖准备位于 `scripts/setup/`；根目录旧 `.sh`／构建 `.py` 文件通过相对符号链接继续可用。Python 构建器已归入 `build/`；运行器、共享模块与原生辅助源码按现有导入布局维护，下方按用途完整分类。

## 环境与使用方式

| 工具类型 | 调用方式 | 说明 |
| --- | --- | --- |
| 算法仿真 | `./scripts/sim.sh <子命令>` | 受管会话复用 ROS 域、Gazebo 分区与场景 |
| ROS／硬件 Python | `./scripts/with_venv.sh python scripts/<工具>.py ...` | .venv、ROS 2 与 install_uav |
| PX4 Python | `./scripts/with_px4_sim.sh python scripts/<工具>.py ...` | 隔离 PX4 消息环境；额外任务环境按工具手册加载 |
| 离线纯 Python | `python3 scripts/<工具>.py ...` | 是否需要 NumPy 等依赖，以源码与手册为准 |
| Python 构建器 | `./scripts/with_venv.sh python scripts/build/<工具>.py ...` | PX4 构建器使用 `with_px4_sim.sh`；旧路径继续可用 |
| 构建／安装 Shell | `./scripts/build/<工具>.sh`／`./scripts/setup/<工具>.sh` | 安装脚本可能修改系统依赖，参数和前置条件见对应手册 |
| C++／头文件 | 由相应 Python 构建器消费 | 不作为可直接执行的命令 |

目录描述说明默认用途，具体模式和参数以源码、对应手册及工具帮助为准。在线 `check_*`、`audit_*` 和 `run_*` 中部分工具会启动会话、发仿真目标或注入故障；离线分析工具只读取已完成运行的数据。`--help` 仅在工具支持时使用，不能用批量执行脚本来验证语法。

## 推荐入口

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
| [scripts/maintain_catalogs.py](maintain_catalogs.py) | Generate workspace catalogs and check local documentation/script syntax without running tools. |
| [scripts/sim.sh](sim.sh) | 算法仿真与 PX4 子命令统一分发入口；具体参数见对应手册。 |
| [scripts/sim_control.py](sim_control.py) | Manage one Gazebo EGO/nvblox session in this workspace (Linux). |
| [scripts/sim_ros_control.py](sim_ros_control.py) | Bounded ROS operations for sim.sh; run via with_venv.sh. |
| [scripts/with_px4_sim.sh](with_px4_sim.sh) | 加载隔离的 PX4 消息与仿真环境。 |
| [scripts/with_venv.sh](with_venv.sh) | 进入工作区，加载 .venv、系统 ROS 2 与 install_uav。 |

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
| [scripts/vendor_lfs.py](vendor_lfs.py) | Prepare and check LFS working files in the workspace and its submodules. |
| [scripts/vendor_patches.py](vendor_patches.py) | Validate vendor trees against pinned commits using disposable Git indexes. |

## 运行与交互

| 文件 | 用途 |
| --- | --- |
| [scripts/localization_selection_session.py](localization_selection_session.py) | Ground-only source selector attached to owned joint SITL; no flight publishers. |
| [scripts/oakd_vio_motion_ui.py](oakd_vio_motion_ui.py) | Read-only hardware stereo/trajectory UI and operator-marked motion capture. |
| [scripts/pocket_panel.py](pocket_panel.py) | Desktop RadioMaster Pocket UAV panel: AETR channels and optional uinput output. |
| [scripts/record_nav_debug_bag.sh](record_nav_debug_bag.sh) | 采集导航诊断 ROS bag。 |
| [scripts/run_cuvslam_pose_transport.py](run_cuvslam_pose_transport.py) | Synthetic DDS tests of pose warmup and reset retirement, never real VIO/flight. |
| [scripts/run_flow_manual_tasks.py](run_flow_manual_tasks.py) | Owned no-EV/no-GNSS native Position test with a simulated RC receiver. |
| [scripts/run_fusion_switch_sitl.py](run_fusion_switch_sitl.py) | Disarmed synthetic EV + rendered flow/range + actual PX4 selection audit. |
| [scripts/run_mid360_fastlio_rviz.sh](run_mid360_fastlio_rviz.sh) | 启动 MID360、FAST-LIO 与 RViz。 |
| [scripts/run_mission_protocol_smoke.py](run_mission_protocol_smoke.py) | Owned ROS Action/service protocol smoke. Synthetic data; no vehicle I/O. |
| [scripts/run_planning_context_smoke.py](run_planning_context_smoke.py) | Real EGO + context bridge DDS regression on a synthetic ESDF, no flight outputs. |
| [scripts/run_px4_flight.sh](run_px4_flight.sh) | 在隔离环境中启动 PX4 任务运行器。 |
| [scripts/run_px4_flight_tasks.py](run_px4_flight_tasks.py) | Exercise real ExecuteMission against owned x500 SITL, with independent truth. |
| [scripts/run_px4_microxrce.sh](run_px4_microxrce.sh) | 启动 PX4 Micro XRCE-DDS Agent。 |
| [scripts/run_px4_mtf01.py](run_px4_mtf01.py) | 默认未解锁光流／测距审计；显式 --flight 支持独立光流飞行仿真。 |
| [scripts/run_px4_sitl_smoke.py](run_px4_sitl_smoke.py) | Owned, disarmed x500 smoke session. Run with scripts/with_px4_sim.sh. |
| [scripts/run_px4_vio_sensors.py](run_px4_vio_sensors.py) | Owned official PX4/Gazebo/QGC VIO audits and explicit warehouse BT flight. |
| [scripts/run_px4_vio_sensors.sh](run_px4_vio_sensors.sh) | 在隔离环境中启动 VIO 传感器／实验任务运行器。 |
| [scripts/run_rviz_nav.sh](run_rviz_nav.sh) | 启动导航 RViz。 |
| [scripts/run_sim_scenario.py](run_sim_scenario.py) | S0 host regression runner with immutable configuration and bounded subprocesses. |
| [scripts/run_vio_monitor.sh](run_vio_monitor.sh) | 启动 VIO 监视器。 |
| [scripts/run_vio_node_smoke.py](run_vio_node_smoke.py) | Check real cuVSLAM node/CUDA loading without camera or FMU publishers. |
| [scripts/run_vio_transport_smoke.py](run_vio_transport_smoke.py) | Isolated DDS contract test with synthetic odometry/status, never real VIO. |
| [scripts/select_localization.py](select_localization.py) | One identity-bound ground source selection request; no Pocket dependency. |
| [scripts/usb_bt_operator.py](usb_bt_operator.py) | Explicit USB switch mapping to the bound BT; sticks remain QGC's input. |
| [scripts/usb_joystick.py](usb_joystick.py) | Read Linux USB joystick events; inspection is the default, no flight output. |

## 场景、标定与资产准备

| 文件 | 用途 |
| --- | --- |
| [scripts/prepare_mtf01_assets.py](prepare_mtf01_assets.py) | MTF-01 reference geometry: rendered down-facing camera and GPU single ray. |
| [scripts/prepare_openvins_config.py](prepare_openvins_config.py) | Generate OpenVINS calibration from physical mount TF and camera intrinsics. |
| [scripts/prepare_px4_depth_assets.py](prepare_px4_depth_assets.py) | Same-aircraft ideal RGBD reference; fixed 5 degrees down, not OAK-D calibration. |
| [scripts/prepare_px4_sim.py](prepare_px4_sim.py) | Prepare locked SITL dependencies without changing existing checkouts. |
| [scripts/prepare_vio_replay_calibration.py](prepare_vio_replay_calibration.py) | Resolve SDK15 dump body-reference ambiguity using physical mount calibration. |
| [scripts/prepare_vio_sensor_assets.py](prepare_vio_sensor_assets.py) | Generate an explicit stereo/IMU reference rig and textured SITL test world. |

## 在线检查与故障回归

| 文件 | 用途 |
| --- | --- |
| [scripts/audit_oakd_vio_hardware.py](audit_oakd_vio_hardware.py) | Collect raw OAK-D VIO evidence in an isolated ROS domain, without flight nodes. |
| [scripts/audit_vio_sdk_dump.py](audit_vio_sdk_dump.py) | Read actual SDK-consumed input dump; diagnostic receipt, never flight evidence. |
| [scripts/audit_vio_truth_topics.py](audit_vio_truth_topics.py) | Read-only Gazebo topic graph proof for an active owned dynamic audit. |
| [scripts/check_continuous_navigation.py](check_continuous_navigation.py) | Deterministic ROS planner/executor check with an incrementally observed corridor. |
| [scripts/check_gazebo_algorithm_mission.py](check_gazebo_algorithm_mission.py) | Owned C++ BT/MissionServer regression in the managed algorithm lab. |
| [scripts/check_gazebo_autonomous.py](check_gazebo_autonomous.py) | Bounded autonomous mapping trial with independent Gazebo geometry checks. |
| [scripts/check_gazebo_expanded_goal.py](check_gazebo_expanded_goal.py) | Expanded-world RViz goal regression with independent SDF geometry checks. |
| [scripts/check_gazebo_exploration.py](check_gazebo_exploration.py) | Online exploration regression for lab/expanded (sends a simulation goal). |
| [scripts/check_gazebo_failures.py](check_gazebo_failures.py) | Negative goals for the fixed uav_ego_lab scene; run only while stopped. |
| [scripts/check_gazebo_goal.py](check_gazebo_goal.py) | One goal regression in uav_ego_lab; ground-truth cylinder/wall clearance. |
| [scripts/check_gazebo_map_loss.py](check_gazebo_map_loss.py) | Fault injection for the dedicated Gazebo algorithm test process only. |
| [scripts/check_gazebo_navigation_action.py](check_gazebo_navigation_action.py) | Ordinary Action-client regression for the algorithm velocity-model profile. |
| [scripts/check_nav_stack.sh](check_nav_stack.sh) | 检查导航图与输入状态。 |
| [scripts/check_px4_interfaces.py](check_px4_interfaces.py) | Compare every DDS-exported message and its nested types with pinned px4_msgs. |
| [scripts/check_sim_environment.py](check_sim_environment.py) | Read-only S0 dependency audit. Runtime topic/TF/clock readiness is not asserted. |
| [scripts/inspect_px4_usb.py](inspect_px4_usb.py) | Passive USB MAVLink capture. Never sends heartbeat, requests or commands. |
| [scripts/measure_oakd_vio_performance.sh](measure_oakd_vio_performance.sh) | 采集 OAK-D VIO 性能指标。 |
| [scripts/pocket_validation_input.py](pocket_validation_input.py) | Owned uinput stimulus for explicit computer-only flight acceptance cases. |
| [scripts/px4_camera_mount_audit.py](px4_camera_mount_audit.py) | Read-only Gazebo Scene pose audit against the advertised body optical TF. |
| [scripts/px4_depth_audit.py](px4_depth_audit.py) | Read-only audit of the PX4 reference depth stream (OakD-Lite model, not OAK-D Pro W calibration). |
| [scripts/px4_mapping_tf.py](px4_mapping_tf.py) | Disarmed-only mapping TF observer. No PX4 input publishers or truth poses. |
| [scripts/px4_planning_audit.py](px4_planning_audit.py) | Read-only real-map planning graph audit; never submits a goal or FMU input. |
| [scripts/px4_vio_flight_session.py](px4_vio_flight_session.py) | Continuous EV writer for owned warehouse SITL; no command/setpoint publishers. |
| [scripts/px4_vio_motion_audit.py](px4_vio_motion_audit.py) | Independent timestamp-matched VIO error audit; truth is never fed to VIO. |
| [scripts/px4_vio_pose_audit.py](px4_vio_pose_audit.py) | Observe normalized SDK poses and the reset retirement boundary, without FMU writes. |
| [scripts/px4_vio_real_fusion.py](px4_vio_real_fusion.py) | Owned, disarmed same-aircraft cuVSLAM -> PX4 EV audit. No synthetic inputs. |
| [scripts/px4_vio_sensor_audit.py](px4_vio_sensor_audit.py) | Observe physical simulated stereo/IMU and actual cuVSLAM outputs, never inject pose. |
| [scripts/px4_vision_audit.py](px4_vision_audit.py) | Synthetic EV input -> actual PX4 EKF fusion audit, disarmed only. |
| [scripts/survey_gazebo_map.py](survey_gazebo_map.py) | Offline Gazebo depth survey (teleported camera poses, NOT flight validation). |

## 离线分析、验收与归档

| 文件 | 用途 |
| --- | --- |
| [scripts/analyze_observation_grid.py](analyze_observation_grid.py) | Offline ESDF evidence and ideal camera visibility; never produces a flight map. |
| [scripts/analyze_sdk_covariance.py](analyze_sdk_covariance.py) | Offline covariance evidence; no change to source quality or flight admission. |
| [scripts/analyze_vio_chain_flight.py](analyze_vio_chain_flight.py) | Offline receipt-time trajectory screening; never a control/estimator input. |
| [scripts/analyze_vio_chain_map.py](analyze_vio_chain_map.py) | Read-only collision screening of an actual flight's frozen depth map. |
| [scripts/analyze_vio_chain_native.py](analyze_vio_chain_native.py) | Screen actual PX4 native fusion in one completed flight's ULog. |
| [scripts/analyze_vio_final_rest.py](analyze_vio_final_rest.py) | Separate pose marginal diagnostics in an already frozen final-rest window. |
| [scripts/analyze_vio_flight_images.py](analyze_vio_flight_images.py) | Offline image structure diagnostics. Candidate corners are NOT cuVSLAM inliers. |
| [scripts/analyze_vio_physical_replay.py](analyze_vio_physical_replay.py) | Recompute physical-reference replay diagnostics, never flight acceptance. |
| [scripts/analyze_vio_sdk16_marginal.py](analyze_vio_sdk16_marginal.py) | Validate repaired SDK exports and same-trajectory covariance counterfactuals. |
| [scripts/assess_px4_bt_suite.py](assess_px4_bt_suite.py) | Read-only acceptance of four owned BT/SITL runs; never starts flight or writes FMU. |
| [scripts/assess_px4_real_vio_fusion.py](assess_px4_real_vio_fusion.py) | Independently inspect saved source/output/EKF chronology, without ROS or VIO code. |
| [scripts/assess_vio_motion_evidence.py](assess_vio_motion_evidence.py) | Replay archived reference-motion error independently, without ROS or live graph. |
| [scripts/assess_vio_navigation.py](assess_vio_navigation.py) | Accept a completed simulated mission against an explicit positioning limit. |
| [scripts/assess_vio_sdk_candidate.py](assess_vio_sdk_candidate.py) | Native candidate diagnostics, independent of ROS and flight admission. |
| [scripts/assess_vio_sdk_profile.py](assess_vio_sdk_profile.py) | Read Nsight's exported SQLite; overlap is evidence, not causal attribution. |
| [scripts/assess_vio_static_avoidance.py](assess_vio_static_avoidance.py) | Audit actual EGO/truth evidence; an empty-path mission cannot pass avoidance. |
| [scripts/assess_vio_timing.py](assess_vio_timing.py) | Compare original-sample timing across owned publishers; no control/admission effects. |
| [scripts/assess_vio_warehouse.py](assess_vio_warehouse.py) | Read-only warehouse scene/source qualification; never authorizes flight. |
| [scripts/assess_warehouse_flight.py](assess_warehouse_flight.py) | Read-only actual warehouse BT closure audit; low flight never qualifies 1.5 m. |
| [scripts/diagnose_vio_chain.py](diagnose_vio_chain.py) | Read-only VIO transport, EKF noise and control-lag diagnosis of a completed run. |
| [scripts/diagnose_vio_deadline.py](diagnose_vio_deadline.py) | Locate original-sample deadline handover; read-only, never reauthorize a source. |
| [scripts/diagnose_vio_motion.py](diagnose_vio_motion.py) | Read-only per-axis error and covariance chronology; single initial alignment. |
| [scripts/diagnose_vio_sdk16_errors.py](diagnose_vio_sdk16_errors.py) | SDK16 error diagnostics against observer-only truth, with no trajectory fit. |
| [scripts/diagnose_vio_sdk16_information.py](diagnose_vio_sdk16_information.py) | Separate SDK conditional export correctness from global covariance validity. |
| [scripts/diagnose_vio_sdk16_replay.py](diagnose_vio_sdk16_replay.py) | Same-pixel SDK16 VIO/VO diagnostic; one initial yaw/position anchor only. |
| [scripts/freeze_vio_chain.py](freeze_vio_chain.py) | Freeze completed experimental chain using verified consumed-input snapshots. |
| [scripts/freeze_vio_run.py](freeze_vio_run.py) | Freeze a completed run and its exact consumed inputs before further edits. |
| [scripts/plot_vio_chain_covariance.py](plot_vio_chain_covariance.py) | Plot frozen consistency measurements; does not modify or calibrate covariance. |
| [scripts/plot_vio_sdk16_errors.py](plot_vio_sdk16_errors.py) | Plot measured SDK16 error and covariance, without altering estimates. |
| [scripts/record_vio_motion_build.py](record_vio_motion_build.py) | Bind a successfully built fixture to its source and binary, not an old cache. |
| [scripts/record_vio_same_aircraft_build.py](record_vio_same_aircraft_build.py) | 记录同机运动插件源码与二进制回执。 |

## 共享实现与原生辅助源码

| 文件 | 用途 |
| --- | --- |
| [scripts/bt_flight_client.py](bt_flight_client.py) | Owned BT subprocess and read-only root-result observer for SITL acceptance. |
| [scripts/bt_operator_intent.py](bt_operator_intent.py) | Send one operator intent to the current isolated SITL BT, without a USB radio. |
| [scripts/mtf01_audit.py](mtf01_audit.py) | Assess actual PX4 uORB receipts for the isolated flow/range reference. |
| [scripts/mtf01_flow_bridge.py](mtf01_flow_bridge.py) | Image-derived flow for a fixed +90 degree pitch camera, no truth input. |
| [scripts/mtf01_range_bridge.py](mtf01_range_bridge.py) | Single-ray Gazebo range -> PX4, fixed down-facing body installation only. |
| [scripts/nav_debug_lib.sh](nav_debug_lib.sh) | 导航诊断脚本共用的环境与话题配置。 |
| [scripts/nav_launch.env](nav_launch.env) | 导航启动的公共环境变量默认值。 |
| [scripts/no_gnss_flight_cases.py](no_gnss_flight_cases.py) | Actual QGC/uinput -> PX4 acceptance stimuli, never fabricated FMU telemetry. |
| [scripts/openvins_covariance_replay.cpp](openvins_covariance_replay.cpp) | OpenVINS 历史候选原生协方差回放诊断。 |
| [scripts/openvins_pose_covariance.hpp](openvins_pose_covariance.hpp) | OpenVINS 历史候选位姿协方差辅助实现。 |
| [scripts/openvins_ros_node.cpp](openvins_ros_node.cpp) | OpenVINS 历史候选 ROS 接口源码；当前路线暂停。 |
| [scripts/pocket_operator_session.py](pocket_operator_session.py) | Owned Pocket SITL control with explicit legacy USB or UAV AETR profiles. |
| [scripts/pocket_panel_model.py](pocket_panel_model.py) | UAV Model 5 channel model; independent of Tk, ROS and Linux input. |
| [scripts/pocket_switches.py](pocket_switches.py) | Pocket SA/SB/SD stable edges, bound to the inspected USB axes. |
| [scripts/pocket_uav_switches.py](pocket_uav_switches.py) | UAV AETR virtual profile, deliberately separate from the legacy USB mapping. |
| [scripts/pocket_virtual_joystick.py](pocket_virtual_joystick.py) | Linux uinput Pocket transport using only the standard library. |
| [scripts/sim_validation.py](sim_validation.py) | Shared S0 artifact contracts. No ROS imports or simulation side effects. |
| [scripts/vio_chain_mapping.py](vio_chain_mapping.py) | Attach actual depth to the VIO aircraft and start the owned mapping graph. |
| [scripts/vio_component.launch.py](vio_component.launch.py) | Owned single-thread SDK container, using the same parameters and source topics. |
| [scripts/vio_covariance_consistency.py](vio_covariance_consistency.py) | Truth-observer-only six-dimensional consistency rejection test. |
| [scripts/vio_cpu_affinity.py](vio_cpu_affinity.py) | Explicit process-local scheduling comparison; no system-wide CPU changes. |
| [scripts/vio_evidence.py](vio_evidence.py) | Preserve invalid SDK values in raw arrays and explicit nullable diagnostics. |
| [scripts/vio_gc_pause_audit.py](vio_gc_pause_audit.py) | Passive Python GC pause evidence; never changes thresholds or GC behavior. |
| [scripts/vio_live_evidence.py](vio_live_evidence.py) | Freeze completed mission evidence in a process independent of ROS callbacks. |
| [scripts/vio_mtf01_joint.py](vio_mtf01_joint.py) | Disarmed joint observation only: existing VIO contract plus independent MTF aids. |
| [scripts/vio_observer_gc.py](vio_observer_gc.py) | Keep automatic cyclic GC outside a bounded owned observation/control window. |
| [scripts/vio_pose_window.py](vio_pose_window.py) | Source-rate evidence uses ROS/sample time; retirement uses monotonic time. |
| [scripts/vio_render_device.py](vio_render_device.py) | Explicit renderer selection for owned Gazebo only; never changes machine settings. |
| [scripts/vio_same_aircraft_dynamic.py](vio_same_aircraft_dynamic.py) | Explicit disarmed x500 physics fixture; this policy never admits flight. |
| [scripts/vio_sdk16_acceptance.py](vio_sdk16_acceptance.py) | Fail-closed VIO candidate acceptance receipt; publishes no ROS controls. |
| [scripts/vio_sdk16_conditional_covariance.hpp](vio_sdk16_conditional_covariance.hpp) | SDK16 条件位姿协方差导出辅助实现。 |
| [scripts/vio_sdk16_covariance.hpp](vio_sdk16_covariance.hpp) | SDK16 协方差导出共用实现。 |
| [scripts/vio_sdk16_marginal_covariance.hpp](vio_sdk16_marginal_covariance.hpp) | SDK16 边缘位姿协方差导出辅助实现。 |
| [scripts/vio_sdk16_state_audit.py](vio_sdk16_state_audit.py) | Observe exact SDK16 Track initialization state; no estimator/control writes. |
| [scripts/vio_sdk16_world_covariance.hpp](vio_sdk16_world_covariance.hpp) | SDK16 世界坐标系协方差转换辅助实现。 |
| [scripts/vio_sdk_covariance_replay.cpp](vio_sdk_covariance_replay.cpp) | 原生 SDK 位姿／协方差回放诊断。 |
| [scripts/vio_sdk_parameters.py](vio_sdk_parameters.py) | Parameters compatible with the reviewed cuVSLAM 15 stream sequencer. |
| [scripts/vio_sdk_profile.py](vio_sdk_profile.py) | Optional CUDA/NVTX trace; profiling is never source qualification. |
| [scripts/vio_sdk_runtime.py](vio_sdk_runtime.py) | Read-only SDK parameter receipt; never changes estimator parameters. |
| [scripts/vio_sdk_selection.py](vio_sdk_selection.py) | One explicit VIO default; missing or drifted SDK16 fails without fallback. |
| [scripts/vio_sdk_target.py](vio_sdk_target.py) | Give the profiled SDK its own session so SIGINT cannot cancel Nsight export. |
| [scripts/vio_sensor_quality.py](vio_sensor_quality.py) | Numerical acceptance of observed sensor timestamps; independent of ROS callbacks. |
| [scripts/vio_sensor_transforms.py](vio_sensor_transforms.py) | Sensor mount rotations shared by generated geometry and runtime TF. |
| [scripts/vio_sim_clock.py](vio_sim_clock.py) | Simulation-only clock from unchanged Gazebo IMU physics timestamps. |
| [scripts/vio_stereo_ui.py](vio_stereo_ui.py) | Read-only stereo/SDK/PX4 viewer; bounded timestamped images, no control publishers. |
| [scripts/vio_ui_lifecycle.py](vio_ui_lifecycle.py) | Viewer retention requires current independent PX4 ground telemetry. |
| [scripts/vio_warehouse_scene.py](vio_warehouse_scene.py) | Deterministic warehouse geometry and independent free-volume checks. |

## 包内与场景启动脚本

| 文件 | 用途 |
| --- | --- |
| [simulation/scripts/run_gazebo_harmonic_nav.sh](../simulation/scripts/run_gazebo_harmonic_nav.sh) | 启动 Gazebo 场景与传感器桥，保持单实例锁。 |
| [simulation/scripts/run_isaac_sim_45_ui.sh](../simulation/scripts/run_isaac_sim_45_ui.sh) | 从已有 Isaac Sim 4.5 安装启动 UI。 |
| [simulation/scripts/run_uav_obstacle_course.sh](../simulation/scripts/run_uav_obstacle_course.sh) | 选择障碍场地并调用 Gazebo 场景启动器。 |
| [src/oakd_perception/scripts/run_oakd_active_max.sh](../src/oakd_perception/scripts/run_oakd_active_max.sh) | 启动 OAK-D 主动双目强度配置。 |
| [src/oakd_perception/scripts/run_oakd_balance.sh](../src/oakd_perception/scripts/run_oakd_balance.sh) | 启动 OAK-D 均衡深度配置。 |
| [src/oakd_perception/scripts/run_oakd_indoor.sh](../src/oakd_perception/scripts/run_oakd_indoor.sh) | 启动 OAK-D 室内深度配置。 |
| [src/oakd_perception/scripts/run_oakd_outdoor.sh](../src/oakd_perception/scripts/run_oakd_outdoor.sh) | 启动 OAK-D 室外深度配置。 |
| [src/uav_bringup/scripts/gazebo_odometry_velocity.py](../src/uav_bringup/scripts/gazebo_odometry_velocity.py) | 由 Gazebo／VIO 位姿计算平滑三维机体系速度，只修改 twist。 |

## 兼容入口

旧路径为相对符号链接，与规范路径使用同一份实现。

| 旧路径 | 规范路径 |
| --- | --- |
| [scripts/apply_vendor_patches.sh](apply_vendor_patches.sh) | [scripts/setup/apply_vendor_patches.sh](setup/apply_vendor_patches.sh) |
| [scripts/build_algorithm_sim.sh](build_algorithm_sim.sh) | [scripts/build/build_algorithm_sim.sh](build/build_algorithm_sim.sh) |
| [scripts/build_livox_sdk2.sh](build_livox_sdk2.sh) | [scripts/build/build_livox_sdk2.sh](build/build_livox_sdk2.sh) |
| [scripts/build_navigation_actions.sh](build_navigation_actions.sh) | [scripts/build/build_navigation_actions.sh](build/build_navigation_actions.sh) |
| [scripts/build_nvidia_3d_nav_deps.sh](build_nvidia_3d_nav_deps.sh) | [scripts/build/build_nvidia_3d_nav_deps.sh](build/build_nvidia_3d_nav_deps.sh) |
| [scripts/build_openvins_candidate.py](build_openvins_candidate.py) | [scripts/build/build_openvins_candidate.py](build/build_openvins_candidate.py) |
| [scripts/build_px4_flight.sh](build_px4_flight.sh) | [scripts/build/build_px4_flight.sh](build/build_px4_flight.sh) |
| [scripts/build_px4_sim.sh](build_px4_sim.sh) | [scripts/build/build_px4_sim.sh](build/build_px4_sim.sh) |
| [scripts/build_px4_vio.py](build_px4_vio.py) | [scripts/build/build_px4_vio.py](build/build_px4_vio.py) |
| [scripts/build_px4_vio.sh](build_px4_vio.sh) | [scripts/build/build_px4_vio.sh](build/build_px4_vio.sh) |
| [scripts/build_uav_stack.sh](build_uav_stack.sh) | [scripts/build/build_uav_stack.sh](build/build_uav_stack.sh) |
| [scripts/build_vio_motion.sh](build_vio_motion.sh) | [scripts/build/build_vio_motion.sh](build/build_vio_motion.sh) |
| [scripts/build_vio_node.sh](build_vio_node.sh) | [scripts/build/build_vio_node.sh](build/build_vio_node.sh) |
| [scripts/build_vio_same_aircraft_motion.sh](build_vio_same_aircraft_motion.sh) | [scripts/build/build_vio_same_aircraft_motion.sh](build/build_vio_same_aircraft_motion.sh) |
| [scripts/build_vio_sdk16_default.py](build_vio_sdk16_default.py) | [scripts/build/build_vio_sdk16_default.py](build/build_vio_sdk16_default.py) |
| [scripts/build_vio_sdk16_marginal_candidate.py](build_vio_sdk16_marginal_candidate.py) | [scripts/build/build_vio_sdk16_marginal_candidate.py](build/build_vio_sdk16_marginal_candidate.py) |
| [scripts/build_vio_sdk16_ros_candidate.py](build_vio_sdk16_ros_candidate.py) | [scripts/build/build_vio_sdk16_ros_candidate.py](build/build_vio_sdk16_ros_candidate.py) |
| [scripts/build_vio_sdk16_schur64.py](build_vio_sdk16_schur64.py) | [scripts/build/build_vio_sdk16_schur64.py](build/build_vio_sdk16_schur64.py) |
| [scripts/build_vio_sdk16_schur_diagnostic.py](build_vio_sdk16_schur_diagnostic.py) | [scripts/build/build_vio_sdk16_schur_diagnostic.py](build/build_vio_sdk16_schur_diagnostic.py) |
| [scripts/build_vio_sdk_candidate.py](build_vio_sdk_candidate.py) | [scripts/build/build_vio_sdk_candidate.py](build/build_vio_sdk_candidate.py) |
| [scripts/install_cuda_13_2_stack.sh](install_cuda_13_2_stack.sh) | [scripts/setup/install_cuda_13_2_stack.sh](setup/install_cuda_13_2_stack.sh) |
| [scripts/install_nvidia_3d_nav_rosdeps.sh](install_nvidia_3d_nav_rosdeps.sh) | [scripts/setup/install_nvidia_3d_nav_rosdeps.sh](setup/install_nvidia_3d_nav_rosdeps.sh) |
| [scripts/install_vendor_lfs_assets.sh](install_vendor_lfs_assets.sh) | [scripts/setup/install_vendor_lfs_assets.sh](setup/install_vendor_lfs_assets.sh) |
| [scripts/install_vpi_4_stack.sh](install_vpi_4_stack.sh) | [scripts/setup/install_vpi_4_stack.sh](setup/install_vpi_4_stack.sh) |
| [scripts/prepare_ego_vendor.sh](prepare_ego_vendor.sh) | [scripts/setup/prepare_ego_vendor.sh](setup/prepare_ego_vendor.sh) |

<!-- END SCRIPT CATALOG -->

## 本地独立回归与包内测试

`scripts/test_*` 由 Git 忽略，不写入上面的版本化目录；包内测试保留在 `src/<package>/test/`。本机可列出全部独立测试：

```bash
rg --files scripts -g 'test_*.py' | sort
```

运行相关测试时按已有环境选择范围；ROS 工作区常需禁用 pytest 的自动插件加载：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./scripts/with_venv.sh python -m pytest <相关测试路径>
```

## 更新与静态检查

```bash
python3 scripts/maintain_catalogs.py --write
python3 scripts/maintain_catalogs.py --check
```

自动区域由工具维护，新增脚本可在源码首行 docstring 描述用途；Shell 和原生辅助源码的说明维护在 `maintain_catalogs.py`。检查只核对目录、项目内 Markdown 链接／锚点与 Python／Bash 语法，不启动被列出的工具。

`sim.sh px4-vio-pocket --sdk-schur64 --vio-primary --device <实际虚拟摇杆路径>` 启动未解锁的 VIO／深度／EGO 操作会话；`assess_vio_static_avoidance.py <自动验收运行目录>` 检查真实静态绕障证据，见 [操作文档](../docs/flight/VIO_POCKET_AVOIDANCE.md)。
