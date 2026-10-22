# 历史脚本名兼容入口

[脚本总目录](../README.md) · [迁移表](layout.json)

兼容链接集中存放在本目录，指向分类目录中的唯一实现。旧 `scripts/<文件名>` 已从根目录移出，当前文档使用规范路径；需要按历史文件名运行时使用 `scripts/compat/<文件名>`。

```bash
# 规范入口
./scripts/sim.sh run full --dry-run
./scripts/build/build_px4_flight.sh --help

# 按历史文件名执行同一实现
python3 scripts/compat/sim_control.py --help
```

复制工具时应保留仓库目录结构和符号链接。不要在兼容链接处增加独立实现；修改目标文件即可。Python 规范入口通过公共路径支持保留旧模块名导入，环境包装器也配置兼容导入目录。

旧公共 Shell 启动链接仍会提示迁移到 `sim.sh run manual`／`run full`；专项开发回归使用对应分类目录中的 Python 运行器和环境包装器。

`layout.json` 的 `scripts` 键记录迁移前的根路径与规范路径，实际兼容链接使用本目录下的同名文件；`resources` 键记录 VIO 资源仍保留的旧路径。目录检查验证所有映射和链接。历史报告不改写，其中的旧路径可按本表定位。

<!-- BEGIN COMPATIBILITY CATALOG -->

| 兼容入口 | 规范路径 |
| --- | --- |
| [scripts/compat/analyze_observation_grid.py](analyze_observation_grid.py) | [scripts/simulation/algorithm/analyze_observation_grid.py](../simulation/algorithm/analyze_observation_grid.py) |
| [scripts/compat/analyze_sdk_covariance.py](analyze_sdk_covariance.py) | [scripts/simulation/vio/analysis/analyze_sdk_covariance.py](../simulation/vio/analysis/analyze_sdk_covariance.py) |
| [scripts/compat/analyze_vio_chain_flight.py](analyze_vio_chain_flight.py) | [scripts/simulation/vio/analysis/analyze_vio_chain_flight.py](../simulation/vio/analysis/analyze_vio_chain_flight.py) |
| [scripts/compat/analyze_vio_chain_map.py](analyze_vio_chain_map.py) | [scripts/simulation/vio/analysis/analyze_vio_chain_map.py](../simulation/vio/analysis/analyze_vio_chain_map.py) |
| [scripts/compat/analyze_vio_chain_native.py](analyze_vio_chain_native.py) | [scripts/simulation/vio/analysis/analyze_vio_chain_native.py](../simulation/vio/analysis/analyze_vio_chain_native.py) |
| [scripts/compat/analyze_vio_final_rest.py](analyze_vio_final_rest.py) | [scripts/simulation/vio/analysis/analyze_vio_final_rest.py](../simulation/vio/analysis/analyze_vio_final_rest.py) |
| [scripts/compat/analyze_vio_flight_images.py](analyze_vio_flight_images.py) | [scripts/simulation/vio/analysis/analyze_vio_flight_images.py](../simulation/vio/analysis/analyze_vio_flight_images.py) |
| [scripts/compat/analyze_vio_physical_replay.py](analyze_vio_physical_replay.py) | [scripts/simulation/vio/analysis/analyze_vio_physical_replay.py](../simulation/vio/analysis/analyze_vio_physical_replay.py) |
| [scripts/compat/analyze_vio_sdk16_marginal.py](analyze_vio_sdk16_marginal.py) | [scripts/simulation/vio/analysis/analyze_vio_sdk16_marginal.py](../simulation/vio/analysis/analyze_vio_sdk16_marginal.py) |
| [scripts/compat/apply_vendor_patches.sh](apply_vendor_patches.sh) | [scripts/setup/apply_vendor_patches.sh](../setup/apply_vendor_patches.sh) |
| [scripts/compat/assess_px4_bt_suite.py](assess_px4_bt_suite.py) | [scripts/simulation/px4/assess_px4_bt_suite.py](../simulation/px4/assess_px4_bt_suite.py) |
| [scripts/compat/assess_px4_real_vio_fusion.py](assess_px4_real_vio_fusion.py) | [scripts/simulation/vio/analysis/assess_px4_real_vio_fusion.py](../simulation/vio/analysis/assess_px4_real_vio_fusion.py) |
| [scripts/compat/assess_vio_motion_evidence.py](assess_vio_motion_evidence.py) | [scripts/simulation/vio/analysis/assess_vio_motion_evidence.py](../simulation/vio/analysis/assess_vio_motion_evidence.py) |
| [scripts/compat/assess_vio_navigation.py](assess_vio_navigation.py) | [scripts/simulation/vio/analysis/assess_vio_navigation.py](../simulation/vio/analysis/assess_vio_navigation.py) |
| [scripts/compat/assess_vio_sdk_candidate.py](assess_vio_sdk_candidate.py) | [scripts/simulation/vio/analysis/assess_vio_sdk_candidate.py](../simulation/vio/analysis/assess_vio_sdk_candidate.py) |
| [scripts/compat/assess_vio_sdk_profile.py](assess_vio_sdk_profile.py) | [scripts/simulation/vio/analysis/assess_vio_sdk_profile.py](../simulation/vio/analysis/assess_vio_sdk_profile.py) |
| [scripts/compat/assess_vio_static_avoidance.py](assess_vio_static_avoidance.py) | [scripts/simulation/vio/analysis/assess_vio_static_avoidance.py](../simulation/vio/analysis/assess_vio_static_avoidance.py) |
| [scripts/compat/assess_vio_timing.py](assess_vio_timing.py) | [scripts/simulation/vio/analysis/assess_vio_timing.py](../simulation/vio/analysis/assess_vio_timing.py) |
| [scripts/compat/assess_vio_warehouse.py](assess_vio_warehouse.py) | [scripts/simulation/vio/analysis/assess_vio_warehouse.py](../simulation/vio/analysis/assess_vio_warehouse.py) |
| [scripts/compat/assess_warehouse_flight.py](assess_warehouse_flight.py) | [scripts/simulation/vio/analysis/assess_warehouse_flight.py](../simulation/vio/analysis/assess_warehouse_flight.py) |
| [scripts/compat/audit_oakd_vio_hardware.py](audit_oakd_vio_hardware.py) | [scripts/hardware/oakd/audit_oakd_vio_hardware.py](../hardware/oakd/audit_oakd_vio_hardware.py) |
| [scripts/compat/audit_vio_sdk_dump.py](audit_vio_sdk_dump.py) | [scripts/simulation/vio/analysis/audit_vio_sdk_dump.py](../simulation/vio/analysis/audit_vio_sdk_dump.py) |
| [scripts/compat/audit_vio_truth_topics.py](audit_vio_truth_topics.py) | [scripts/simulation/vio/analysis/audit_vio_truth_topics.py](../simulation/vio/analysis/audit_vio_truth_topics.py) |
| [scripts/compat/bt_flight_client.py](bt_flight_client.py) | [scripts/simulation/px4/bt_flight_client.py](../simulation/px4/bt_flight_client.py) |
| [scripts/compat/bt_operator_intent.py](bt_operator_intent.py) | [scripts/simulation/pocket/bt_operator_intent.py](../simulation/pocket/bt_operator_intent.py) |
| [scripts/compat/build_algorithm_sim.sh](build_algorithm_sim.sh) | [scripts/build/build_algorithm_sim.sh](../build/build_algorithm_sim.sh) |
| [scripts/compat/build_livox_sdk2.sh](build_livox_sdk2.sh) | [scripts/build/build_livox_sdk2.sh](../build/build_livox_sdk2.sh) |
| [scripts/compat/build_navigation_actions.sh](build_navigation_actions.sh) | [scripts/build/build_navigation_actions.sh](../build/build_navigation_actions.sh) |
| [scripts/compat/build_nvidia_3d_nav_deps.sh](build_nvidia_3d_nav_deps.sh) | [scripts/build/build_nvidia_3d_nav_deps.sh](../build/build_nvidia_3d_nav_deps.sh) |
| [scripts/compat/build_openvins_candidate.py](build_openvins_candidate.py) | [scripts/build/build_openvins_candidate.py](../build/build_openvins_candidate.py) |
| [scripts/compat/build_px4_flight.sh](build_px4_flight.sh) | [scripts/build/build_px4_flight.sh](../build/build_px4_flight.sh) |
| [scripts/compat/build_px4_sim.sh](build_px4_sim.sh) | [scripts/build/build_px4_sim.sh](../build/build_px4_sim.sh) |
| [scripts/compat/build_px4_vio.py](build_px4_vio.py) | [scripts/build/build_px4_vio.py](../build/build_px4_vio.py) |
| [scripts/compat/build_px4_vio.sh](build_px4_vio.sh) | [scripts/build/build_px4_vio.sh](../build/build_px4_vio.sh) |
| [scripts/compat/build_uav_stack.sh](build_uav_stack.sh) | [scripts/build/build_uav_stack.sh](../build/build_uav_stack.sh) |
| [scripts/compat/build_vio_motion.sh](build_vio_motion.sh) | [scripts/build/build_vio_motion.sh](../build/build_vio_motion.sh) |
| [scripts/compat/build_vio_node.sh](build_vio_node.sh) | [scripts/build/build_vio_node.sh](../build/build_vio_node.sh) |
| [scripts/compat/build_vio_same_aircraft_motion.sh](build_vio_same_aircraft_motion.sh) | [scripts/build/build_vio_same_aircraft_motion.sh](../build/build_vio_same_aircraft_motion.sh) |
| [scripts/compat/build_vio_sdk16_default.py](build_vio_sdk16_default.py) | [scripts/build/build_vio_sdk16_default.py](../build/build_vio_sdk16_default.py) |
| [scripts/compat/build_vio_sdk16_marginal_candidate.py](build_vio_sdk16_marginal_candidate.py) | [scripts/build/build_vio_sdk16_marginal_candidate.py](../build/build_vio_sdk16_marginal_candidate.py) |
| [scripts/compat/build_vio_sdk16_ros_candidate.py](build_vio_sdk16_ros_candidate.py) | [scripts/build/build_vio_sdk16_ros_candidate.py](../build/build_vio_sdk16_ros_candidate.py) |
| [scripts/compat/build_vio_sdk16_schur64.py](build_vio_sdk16_schur64.py) | [scripts/build/build_vio_sdk16_schur64.py](../build/build_vio_sdk16_schur64.py) |
| [scripts/compat/build_vio_sdk16_schur_diagnostic.py](build_vio_sdk16_schur_diagnostic.py) | [scripts/build/build_vio_sdk16_schur_diagnostic.py](../build/build_vio_sdk16_schur_diagnostic.py) |
| [scripts/compat/build_vio_sdk_candidate.py](build_vio_sdk_candidate.py) | [scripts/build/build_vio_sdk_candidate.py](../build/build_vio_sdk_candidate.py) |
| [scripts/compat/check_continuous_navigation.py](check_continuous_navigation.py) | [scripts/simulation/algorithm/check_continuous_navigation.py](../simulation/algorithm/check_continuous_navigation.py) |
| [scripts/compat/check_gazebo_algorithm_mission.py](check_gazebo_algorithm_mission.py) | [scripts/simulation/algorithm/check_gazebo_algorithm_mission.py](../simulation/algorithm/check_gazebo_algorithm_mission.py) |
| [scripts/compat/check_gazebo_autonomous.py](check_gazebo_autonomous.py) | [scripts/simulation/algorithm/check_gazebo_autonomous.py](../simulation/algorithm/check_gazebo_autonomous.py) |
| [scripts/compat/check_gazebo_expanded_goal.py](check_gazebo_expanded_goal.py) | [scripts/simulation/algorithm/check_gazebo_expanded_goal.py](../simulation/algorithm/check_gazebo_expanded_goal.py) |
| [scripts/compat/check_gazebo_exploration.py](check_gazebo_exploration.py) | [scripts/simulation/algorithm/check_gazebo_exploration.py](../simulation/algorithm/check_gazebo_exploration.py) |
| [scripts/compat/check_gazebo_failures.py](check_gazebo_failures.py) | [scripts/simulation/algorithm/check_gazebo_failures.py](../simulation/algorithm/check_gazebo_failures.py) |
| [scripts/compat/check_gazebo_goal.py](check_gazebo_goal.py) | [scripts/simulation/algorithm/check_gazebo_goal.py](../simulation/algorithm/check_gazebo_goal.py) |
| [scripts/compat/check_gazebo_map_loss.py](check_gazebo_map_loss.py) | [scripts/simulation/algorithm/check_gazebo_map_loss.py](../simulation/algorithm/check_gazebo_map_loss.py) |
| [scripts/compat/check_gazebo_navigation_action.py](check_gazebo_navigation_action.py) | [scripts/simulation/algorithm/check_gazebo_navigation_action.py](../simulation/algorithm/check_gazebo_navigation_action.py) |
| [scripts/compat/check_nav_stack.sh](check_nav_stack.sh) | [scripts/diagnostics/check_nav_stack.sh](../diagnostics/check_nav_stack.sh) |
| [scripts/compat/check_px4_interfaces.py](check_px4_interfaces.py) | [scripts/simulation/px4/check_px4_interfaces.py](../simulation/px4/check_px4_interfaces.py) |
| [scripts/compat/check_sim_environment.py](check_sim_environment.py) | [scripts/simulation/validation/check_sim_environment.py](../simulation/validation/check_sim_environment.py) |
| [scripts/compat/diagnose_vio_chain.py](diagnose_vio_chain.py) | [scripts/simulation/vio/analysis/diagnose_vio_chain.py](../simulation/vio/analysis/diagnose_vio_chain.py) |
| [scripts/compat/diagnose_vio_deadline.py](diagnose_vio_deadline.py) | [scripts/simulation/vio/analysis/diagnose_vio_deadline.py](../simulation/vio/analysis/diagnose_vio_deadline.py) |
| [scripts/compat/diagnose_vio_motion.py](diagnose_vio_motion.py) | [scripts/simulation/vio/analysis/diagnose_vio_motion.py](../simulation/vio/analysis/diagnose_vio_motion.py) |
| [scripts/compat/diagnose_vio_sdk16_errors.py](diagnose_vio_sdk16_errors.py) | [scripts/simulation/vio/analysis/diagnose_vio_sdk16_errors.py](../simulation/vio/analysis/diagnose_vio_sdk16_errors.py) |
| [scripts/compat/diagnose_vio_sdk16_information.py](diagnose_vio_sdk16_information.py) | [scripts/simulation/vio/analysis/diagnose_vio_sdk16_information.py](../simulation/vio/analysis/diagnose_vio_sdk16_information.py) |
| [scripts/compat/diagnose_vio_sdk16_replay.py](diagnose_vio_sdk16_replay.py) | [scripts/simulation/vio/analysis/diagnose_vio_sdk16_replay.py](../simulation/vio/analysis/diagnose_vio_sdk16_replay.py) |
| [scripts/compat/freeze_vio_chain.py](freeze_vio_chain.py) | [scripts/simulation/vio/analysis/freeze_vio_chain.py](../simulation/vio/analysis/freeze_vio_chain.py) |
| [scripts/compat/freeze_vio_run.py](freeze_vio_run.py) | [scripts/simulation/vio/analysis/freeze_vio_run.py](../simulation/vio/analysis/freeze_vio_run.py) |
| [scripts/compat/inspect_px4_usb.py](inspect_px4_usb.py) | [scripts/hardware/px4/inspect_px4_usb.py](../hardware/px4/inspect_px4_usb.py) |
| [scripts/compat/install_cuda_13_2_stack.sh](install_cuda_13_2_stack.sh) | [scripts/setup/install_cuda_13_2_stack.sh](../setup/install_cuda_13_2_stack.sh) |
| [scripts/compat/install_nvidia_3d_nav_rosdeps.sh](install_nvidia_3d_nav_rosdeps.sh) | [scripts/setup/install_nvidia_3d_nav_rosdeps.sh](../setup/install_nvidia_3d_nav_rosdeps.sh) |
| [scripts/compat/install_vendor_lfs_assets.sh](install_vendor_lfs_assets.sh) | [scripts/setup/install_vendor_lfs_assets.sh](../setup/install_vendor_lfs_assets.sh) |
| [scripts/compat/install_vpi_4_stack.sh](install_vpi_4_stack.sh) | [scripts/setup/install_vpi_4_stack.sh](../setup/install_vpi_4_stack.sh) |
| [scripts/compat/localization_selection_session.py](localization_selection_session.py) | [scripts/simulation/px4/localization_selection_session.py](../simulation/px4/localization_selection_session.py) |
| [scripts/compat/maintain_catalogs.py](maintain_catalogs.py) | [scripts/tools/maintain_catalogs.py](../tools/maintain_catalogs.py) |
| [scripts/compat/measure_oakd_vio_performance.sh](measure_oakd_vio_performance.sh) | [scripts/hardware/oakd/measure_oakd_vio_performance.sh](../hardware/oakd/measure_oakd_vio_performance.sh) |
| [scripts/compat/mtf01_audit.py](mtf01_audit.py) | [scripts/simulation/flow/mtf01_audit.py](../simulation/flow/mtf01_audit.py) |
| [scripts/compat/mtf01_flow_bridge.py](mtf01_flow_bridge.py) | [scripts/simulation/flow/mtf01_flow_bridge.py](../simulation/flow/mtf01_flow_bridge.py) |
| [scripts/compat/mtf01_range_bridge.py](mtf01_range_bridge.py) | [scripts/simulation/flow/mtf01_range_bridge.py](../simulation/flow/mtf01_range_bridge.py) |
| [scripts/compat/nav_debug_lib.sh](nav_debug_lib.sh) | [scripts/diagnostics/nav_debug_lib.sh](../diagnostics/nav_debug_lib.sh) |
| [scripts/compat/nav_launch.env](nav_launch.env) | [scripts/config/nav_launch.env](../config/nav_launch.env) |
| [scripts/compat/no_gnss_flight_cases.py](no_gnss_flight_cases.py) | [scripts/simulation/px4/no_gnss_flight_cases.py](../simulation/px4/no_gnss_flight_cases.py) |
| [scripts/compat/oakd_vio_motion_ui.py](oakd_vio_motion_ui.py) | [scripts/hardware/oakd/oakd_vio_motion_ui.py](../hardware/oakd/oakd_vio_motion_ui.py) |
| [scripts/compat/openvins_covariance_replay.cpp](openvins_covariance_replay.cpp) | [scripts/simulation/vio/sdk/openvins_covariance_replay.cpp](../simulation/vio/sdk/openvins_covariance_replay.cpp) |
| [scripts/compat/openvins_pose_covariance.hpp](openvins_pose_covariance.hpp) | [scripts/simulation/vio/sdk/openvins_pose_covariance.hpp](../simulation/vio/sdk/openvins_pose_covariance.hpp) |
| [scripts/compat/openvins_ros_node.cpp](openvins_ros_node.cpp) | [scripts/simulation/vio/sdk/openvins_ros_node.cpp](../simulation/vio/sdk/openvins_ros_node.cpp) |
| [scripts/compat/plot_vio_chain_covariance.py](plot_vio_chain_covariance.py) | [scripts/simulation/vio/analysis/plot_vio_chain_covariance.py](../simulation/vio/analysis/plot_vio_chain_covariance.py) |
| [scripts/compat/plot_vio_sdk16_errors.py](plot_vio_sdk16_errors.py) | [scripts/simulation/vio/analysis/plot_vio_sdk16_errors.py](../simulation/vio/analysis/plot_vio_sdk16_errors.py) |
| [scripts/compat/pocket_input.py](pocket_input.py) | [scripts/simulation/pocket/pocket_input.py](../simulation/pocket/pocket_input.py) |
| [scripts/compat/pocket_operator_session.py](pocket_operator_session.py) | [scripts/simulation/pocket/pocket_operator_session.py](../simulation/pocket/pocket_operator_session.py) |
| [scripts/compat/pocket_panel.py](pocket_panel.py) | [scripts/simulation/pocket/pocket_panel.py](../simulation/pocket/pocket_panel.py) |
| [scripts/compat/pocket_panel_model.py](pocket_panel_model.py) | [scripts/simulation/pocket/pocket_panel_model.py](../simulation/pocket/pocket_panel_model.py) |
| [scripts/compat/pocket_switches.py](pocket_switches.py) | [scripts/simulation/pocket/pocket_switches.py](../simulation/pocket/pocket_switches.py) |
| [scripts/compat/pocket_uav_switches.py](pocket_uav_switches.py) | [scripts/simulation/pocket/pocket_uav_switches.py](../simulation/pocket/pocket_uav_switches.py) |
| [scripts/compat/pocket_validation_input.py](pocket_validation_input.py) | [scripts/simulation/pocket/pocket_validation_input.py](../simulation/pocket/pocket_validation_input.py) |
| [scripts/compat/pocket_virtual_joystick.py](pocket_virtual_joystick.py) | [scripts/simulation/pocket/pocket_virtual_joystick.py](../simulation/pocket/pocket_virtual_joystick.py) |
| [scripts/compat/prepare_ego_vendor.sh](prepare_ego_vendor.sh) | [scripts/setup/prepare_ego_vendor.sh](../setup/prepare_ego_vendor.sh) |
| [scripts/compat/prepare_mtf01_assets.py](prepare_mtf01_assets.py) | [scripts/simulation/flow/prepare_mtf01_assets.py](../simulation/flow/prepare_mtf01_assets.py) |
| [scripts/compat/prepare_openvins_config.py](prepare_openvins_config.py) | [scripts/simulation/vio/sdk/prepare_openvins_config.py](../simulation/vio/sdk/prepare_openvins_config.py) |
| [scripts/compat/prepare_px4_depth_assets.py](prepare_px4_depth_assets.py) | [scripts/simulation/px4/prepare_px4_depth_assets.py](../simulation/px4/prepare_px4_depth_assets.py) |
| [scripts/compat/prepare_px4_sim.py](prepare_px4_sim.py) | [scripts/simulation/px4/prepare_px4_sim.py](../simulation/px4/prepare_px4_sim.py) |
| [scripts/compat/prepare_vio_replay_calibration.py](prepare_vio_replay_calibration.py) | [scripts/simulation/vio/sdk/prepare_vio_replay_calibration.py](../simulation/vio/sdk/prepare_vio_replay_calibration.py) |
| [scripts/compat/prepare_vio_sensor_assets.py](prepare_vio_sensor_assets.py) | [scripts/simulation/vio/runtime/prepare_vio_sensor_assets.py](../simulation/vio/runtime/prepare_vio_sensor_assets.py) |
| [scripts/compat/px4_camera_mount_audit.py](px4_camera_mount_audit.py) | [scripts/simulation/px4/px4_camera_mount_audit.py](../simulation/px4/px4_camera_mount_audit.py) |
| [scripts/compat/px4_depth_audit.py](px4_depth_audit.py) | [scripts/simulation/px4/px4_depth_audit.py](../simulation/px4/px4_depth_audit.py) |
| [scripts/compat/px4_mapping_tf.py](px4_mapping_tf.py) | [scripts/simulation/px4/px4_mapping_tf.py](../simulation/px4/px4_mapping_tf.py) |
| [scripts/compat/px4_planning_audit.py](px4_planning_audit.py) | [scripts/simulation/px4/px4_planning_audit.py](../simulation/px4/px4_planning_audit.py) |
| [scripts/compat/px4_vio_flight_session.py](px4_vio_flight_session.py) | [scripts/simulation/vio/runtime/px4_vio_flight_session.py](../simulation/vio/runtime/px4_vio_flight_session.py) |
| [scripts/compat/px4_vio_motion_audit.py](px4_vio_motion_audit.py) | [scripts/simulation/vio/runtime/px4_vio_motion_audit.py](../simulation/vio/runtime/px4_vio_motion_audit.py) |
| [scripts/compat/px4_vio_pose_audit.py](px4_vio_pose_audit.py) | [scripts/simulation/vio/runtime/px4_vio_pose_audit.py](../simulation/vio/runtime/px4_vio_pose_audit.py) |
| [scripts/compat/px4_vio_real_fusion.py](px4_vio_real_fusion.py) | [scripts/simulation/vio/runtime/px4_vio_real_fusion.py](../simulation/vio/runtime/px4_vio_real_fusion.py) |
| [scripts/compat/px4_vio_sensor_audit.py](px4_vio_sensor_audit.py) | [scripts/simulation/vio/runtime/px4_vio_sensor_audit.py](../simulation/vio/runtime/px4_vio_sensor_audit.py) |
| [scripts/compat/px4_vision_audit.py](px4_vision_audit.py) | [scripts/simulation/px4/px4_vision_audit.py](../simulation/px4/px4_vision_audit.py) |
| [scripts/compat/record_nav_debug_bag.sh](record_nav_debug_bag.sh) | [scripts/diagnostics/record_nav_debug_bag.sh](../diagnostics/record_nav_debug_bag.sh) |
| [scripts/compat/record_vio_motion_build.py](record_vio_motion_build.py) | [scripts/simulation/vio/runtime/record_vio_motion_build.py](../simulation/vio/runtime/record_vio_motion_build.py) |
| [scripts/compat/record_vio_same_aircraft_build.py](record_vio_same_aircraft_build.py) | [scripts/simulation/vio/runtime/record_vio_same_aircraft_build.py](../simulation/vio/runtime/record_vio_same_aircraft_build.py) |
| [scripts/compat/run_cuvslam_pose_transport.py](run_cuvslam_pose_transport.py) | [scripts/simulation/vio/runtime/run_cuvslam_pose_transport.py](../simulation/vio/runtime/run_cuvslam_pose_transport.py) |
| [scripts/compat/run_flow_manual_tasks.py](run_flow_manual_tasks.py) | [scripts/simulation/flow/run_flow_manual_tasks.py](../simulation/flow/run_flow_manual_tasks.py) |
| [scripts/compat/run_fusion_switch_sitl.py](run_fusion_switch_sitl.py) | [scripts/simulation/flow/run_fusion_switch_sitl.py](../simulation/flow/run_fusion_switch_sitl.py) |
| [scripts/compat/run_mid360_fastlio_rviz.sh](run_mid360_fastlio_rviz.sh) | [scripts/hardware/lidar/run_mid360_fastlio_rviz.sh](../hardware/lidar/run_mid360_fastlio_rviz.sh) |
| [scripts/compat/run_mission_protocol_smoke.py](run_mission_protocol_smoke.py) | [scripts/simulation/validation/run_mission_protocol_smoke.py](../simulation/validation/run_mission_protocol_smoke.py) |
| [scripts/compat/run_planning_context_smoke.py](run_planning_context_smoke.py) | [scripts/simulation/validation/run_planning_context_smoke.py](../simulation/validation/run_planning_context_smoke.py) |
| [scripts/compat/run_px4_flight.sh](run_px4_flight.sh) | [scripts/simulation/compat.sh](../simulation/compat.sh) |
| [scripts/compat/run_px4_flight_tasks.py](run_px4_flight_tasks.py) | [scripts/simulation/px4/run_px4_flight_tasks.py](../simulation/px4/run_px4_flight_tasks.py) |
| [scripts/compat/run_px4_microxrce.sh](run_px4_microxrce.sh) | [scripts/hardware/px4/run_px4_microxrce.sh](../hardware/px4/run_px4_microxrce.sh) |
| [scripts/compat/run_px4_mtf01.py](run_px4_mtf01.py) | [scripts/simulation/flow/run_px4_mtf01.py](../simulation/flow/run_px4_mtf01.py) |
| [scripts/compat/run_px4_sitl_smoke.py](run_px4_sitl_smoke.py) | [scripts/simulation/px4/run_px4_sitl_smoke.py](../simulation/px4/run_px4_sitl_smoke.py) |
| [scripts/compat/run_px4_vio_sensors.py](run_px4_vio_sensors.py) | [scripts/simulation/vio/runtime/run_px4_vio_sensors.py](../simulation/vio/runtime/run_px4_vio_sensors.py) |
| [scripts/compat/run_px4_vio_sensors.sh](run_px4_vio_sensors.sh) | [scripts/simulation/compat.sh](../simulation/compat.sh) |
| [scripts/compat/run_rviz_nav.sh](run_rviz_nav.sh) | [scripts/diagnostics/run_rviz_nav.sh](../diagnostics/run_rviz_nav.sh) |
| [scripts/compat/run_sim_scenario.py](run_sim_scenario.py) | [scripts/simulation/validation/run_sim_scenario.py](../simulation/validation/run_sim_scenario.py) |
| [scripts/compat/run_vio_monitor.sh](run_vio_monitor.sh) | [scripts/hardware/oakd/run_vio_monitor.sh](../hardware/oakd/run_vio_monitor.sh) |
| [scripts/compat/run_vio_node_smoke.py](run_vio_node_smoke.py) | [scripts/simulation/vio/runtime/run_vio_node_smoke.py](../simulation/vio/runtime/run_vio_node_smoke.py) |
| [scripts/compat/run_vio_transport_smoke.py](run_vio_transport_smoke.py) | [scripts/simulation/vio/runtime/run_vio_transport_smoke.py](../simulation/vio/runtime/run_vio_transport_smoke.py) |
| [scripts/compat/select_localization.py](select_localization.py) | [scripts/simulation/px4/select_localization.py](../simulation/px4/select_localization.py) |
| [scripts/compat/sim_control.py](sim_control.py) | [scripts/simulation/algorithm/sim_control.py](../simulation/algorithm/sim_control.py) |
| [scripts/compat/sim_menu.py](sim_menu.py) | [scripts/simulation/menu.py](../simulation/menu.py) |
| [scripts/compat/sim_ros_control.py](sim_ros_control.py) | [scripts/simulation/algorithm/sim_ros_control.py](../simulation/algorithm/sim_ros_control.py) |
| [scripts/compat/sim_validation.py](sim_validation.py) | [scripts/simulation/validation/sim_validation.py](../simulation/validation/sim_validation.py) |
| [scripts/compat/survey_gazebo_map.py](survey_gazebo_map.py) | [scripts/simulation/algorithm/survey_gazebo_map.py](../simulation/algorithm/survey_gazebo_map.py) |
| [scripts/compat/usb_bt_operator.py](usb_bt_operator.py) | [scripts/simulation/pocket/usb_bt_operator.py](../simulation/pocket/usb_bt_operator.py) |
| [scripts/compat/usb_joystick.py](usb_joystick.py) | [scripts/simulation/pocket/usb_joystick.py](../simulation/pocket/usb_joystick.py) |
| [scripts/compat/vendor_lfs.py](vendor_lfs.py) | [scripts/setup/vendor_lfs.py](../setup/vendor_lfs.py) |
| [scripts/compat/vendor_patches.py](vendor_patches.py) | [scripts/setup/vendor_patches.py](../setup/vendor_patches.py) |
| [scripts/compat/vio_chain_mapping.py](vio_chain_mapping.py) | [scripts/simulation/vio/runtime/vio_chain_mapping.py](../simulation/vio/runtime/vio_chain_mapping.py) |
| [scripts/compat/vio_component.launch.py](vio_component.launch.py) | [scripts/simulation/vio/runtime/vio_component.launch.py](../simulation/vio/runtime/vio_component.launch.py) |
| [scripts/compat/vio_covariance_consistency.py](vio_covariance_consistency.py) | [scripts/simulation/vio/runtime/vio_covariance_consistency.py](../simulation/vio/runtime/vio_covariance_consistency.py) |
| [scripts/compat/vio_cpu_affinity.py](vio_cpu_affinity.py) | [scripts/simulation/vio/runtime/vio_cpu_affinity.py](../simulation/vio/runtime/vio_cpu_affinity.py) |
| [scripts/compat/vio_evidence.py](vio_evidence.py) | [scripts/simulation/vio/runtime/vio_evidence.py](../simulation/vio/runtime/vio_evidence.py) |
| [scripts/compat/vio_gc_pause_audit.py](vio_gc_pause_audit.py) | [scripts/simulation/vio/runtime/vio_gc_pause_audit.py](../simulation/vio/runtime/vio_gc_pause_audit.py) |
| [scripts/compat/vio_live_evidence.py](vio_live_evidence.py) | [scripts/simulation/vio/runtime/vio_live_evidence.py](../simulation/vio/runtime/vio_live_evidence.py) |
| [scripts/compat/vio_mtf01_joint.py](vio_mtf01_joint.py) | [scripts/simulation/vio/runtime/vio_mtf01_joint.py](../simulation/vio/runtime/vio_mtf01_joint.py) |
| [scripts/compat/vio_observer_gc.py](vio_observer_gc.py) | [scripts/simulation/vio/runtime/vio_observer_gc.py](../simulation/vio/runtime/vio_observer_gc.py) |
| [scripts/compat/vio_pose_window.py](vio_pose_window.py) | [scripts/simulation/vio/runtime/vio_pose_window.py](../simulation/vio/runtime/vio_pose_window.py) |
| [scripts/compat/vio_render_device.py](vio_render_device.py) | [scripts/simulation/vio/runtime/vio_render_device.py](../simulation/vio/runtime/vio_render_device.py) |
| [scripts/compat/vio_same_aircraft_dynamic.py](vio_same_aircraft_dynamic.py) | [scripts/simulation/vio/runtime/vio_same_aircraft_dynamic.py](../simulation/vio/runtime/vio_same_aircraft_dynamic.py) |
| [scripts/compat/vio_sdk16_acceptance.py](vio_sdk16_acceptance.py) | [scripts/simulation/vio/sdk/vio_sdk16_acceptance.py](../simulation/vio/sdk/vio_sdk16_acceptance.py) |
| [scripts/compat/vio_sdk16_conditional_covariance.hpp](vio_sdk16_conditional_covariance.hpp) | [scripts/simulation/vio/sdk/vio_sdk16_conditional_covariance.hpp](../simulation/vio/sdk/vio_sdk16_conditional_covariance.hpp) |
| [scripts/compat/vio_sdk16_covariance.hpp](vio_sdk16_covariance.hpp) | [scripts/simulation/vio/sdk/vio_sdk16_covariance.hpp](../simulation/vio/sdk/vio_sdk16_covariance.hpp) |
| [scripts/compat/vio_sdk16_marginal_covariance.hpp](vio_sdk16_marginal_covariance.hpp) | [scripts/simulation/vio/sdk/vio_sdk16_marginal_covariance.hpp](../simulation/vio/sdk/vio_sdk16_marginal_covariance.hpp) |
| [scripts/compat/vio_sdk16_state_audit.py](vio_sdk16_state_audit.py) | [scripts/simulation/vio/sdk/vio_sdk16_state_audit.py](../simulation/vio/sdk/vio_sdk16_state_audit.py) |
| [scripts/compat/vio_sdk16_world_covariance.hpp](vio_sdk16_world_covariance.hpp) | [scripts/simulation/vio/sdk/vio_sdk16_world_covariance.hpp](../simulation/vio/sdk/vio_sdk16_world_covariance.hpp) |
| [scripts/compat/vio_sdk_covariance_replay.cpp](vio_sdk_covariance_replay.cpp) | [scripts/simulation/vio/sdk/vio_sdk_covariance_replay.cpp](../simulation/vio/sdk/vio_sdk_covariance_replay.cpp) |
| [scripts/compat/vio_sdk_parameters.py](vio_sdk_parameters.py) | [scripts/simulation/vio/sdk/vio_sdk_parameters.py](../simulation/vio/sdk/vio_sdk_parameters.py) |
| [scripts/compat/vio_sdk_profile.py](vio_sdk_profile.py) | [scripts/simulation/vio/sdk/vio_sdk_profile.py](../simulation/vio/sdk/vio_sdk_profile.py) |
| [scripts/compat/vio_sdk_runtime.py](vio_sdk_runtime.py) | [scripts/simulation/vio/sdk/vio_sdk_runtime.py](../simulation/vio/sdk/vio_sdk_runtime.py) |
| [scripts/compat/vio_sdk_selection.py](vio_sdk_selection.py) | [scripts/simulation/vio/sdk/vio_sdk_selection.py](../simulation/vio/sdk/vio_sdk_selection.py) |
| [scripts/compat/vio_sdk_target.py](vio_sdk_target.py) | [scripts/simulation/vio/sdk/vio_sdk_target.py](../simulation/vio/sdk/vio_sdk_target.py) |
| [scripts/compat/vio_sensor_quality.py](vio_sensor_quality.py) | [scripts/simulation/vio/runtime/vio_sensor_quality.py](../simulation/vio/runtime/vio_sensor_quality.py) |
| [scripts/compat/vio_sensor_transforms.py](vio_sensor_transforms.py) | [scripts/simulation/vio/runtime/vio_sensor_transforms.py](../simulation/vio/runtime/vio_sensor_transforms.py) |
| [scripts/compat/vio_sim_clock.py](vio_sim_clock.py) | [scripts/simulation/vio/runtime/vio_sim_clock.py](../simulation/vio/runtime/vio_sim_clock.py) |
| [scripts/compat/vio_stereo_ui.py](vio_stereo_ui.py) | [scripts/simulation/vio/runtime/vio_stereo_ui.py](../simulation/vio/runtime/vio_stereo_ui.py) |
| [scripts/compat/vio_ui_lifecycle.py](vio_ui_lifecycle.py) | [scripts/simulation/vio/runtime/vio_ui_lifecycle.py](../simulation/vio/runtime/vio_ui_lifecycle.py) |
| [scripts/compat/vio_warehouse_scene.py](vio_warehouse_scene.py) | [scripts/simulation/vio/runtime/vio_warehouse_scene.py](../simulation/vio/runtime/vio_warehouse_scene.py) |
| [simulation/scripts/run_gazebo_harmonic_nav.sh](../../simulation/scripts/run_gazebo_harmonic_nav.sh) | [scripts/simulation/compat.sh](../simulation/compat.sh) |
| [simulation/scripts/run_isaac_sim_45_ui.sh](../../simulation/scripts/run_isaac_sim_45_ui.sh) | [scripts/simulation/compat.sh](../simulation/compat.sh) |
| [simulation/scripts/run_uav_obstacle_course.sh](../../simulation/scripts/run_uav_obstacle_course.sh) | [scripts/simulation/compat.sh](../simulation/compat.sh) |

<!-- END COMPATIBILITY CATALOG -->
