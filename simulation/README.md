# 无人机仿真入口

所有命令从项目根目录运行。两类入口使用不同场景与启动链路，同一时间只运行一套。

## 三维建图与导航

需要创建/加载地图、查看彩色网格、在 RViz 点击目标并避障时，使用 `uav_ego_nvblox.launch.py`。
按 [仿真使用手册](../docs/SIMULATION_MANUAL.md) 或 [EGO + nvblox 仿真运行指引](../docs/EGO_NVBLOX_GAZEBO.md) 完成：

1. 运行 `./scripts/build_algorithm_sim.sh` 构建算法与显示插件。
2. 首次使用时，以 `mode:=mapping` 启动扩展场景，运行 `survey_gazebo_map.py --layout expanded` 扫描并保存地图。
3. 停止建图进程，以 `mode:=localization` 和相同 world / map_extent 重新启动，再用 `map_bundle load` 加载地图。
4. 等待有效地图，在 RViz 使用 **2D Goal Pose** 选点；通过 `/uav/cancel` 取消目标。

[项目首页](../README.md#快速运行导航仿真) 提供加载已有地图的双终端命令。
当前链路使用 Gazebo 真值定位、nvblox 三维地图、EGO 规划和简化速度执行器，不包含 PX4 SITL。

## 场景与传感器查看

准备 Gazebo Harmonic / ros_gz，并构建 `uav_bringup`（可使用 `./scripts/build_uav_stack.sh`）后，选择一个入口运行：

```bash
# 障碍场地：立柱、门框、平台、起终点标记
./simulation/scripts/run_uav_obstacle_course.sh
```

默认演示场地使用另一个命令，先退出上面的场景再运行：

```bash
./simulation/scripts/run_gazebo_harmonic_nav.sh
```

这些入口只启动 Gazebo、传感器桥、静态 TF 和里程计速度估计器，不启动 nvblox、EGO 或 RViz 导航。
其中 `uav_obstacle_course.sdf` 不适用导航指引中 lab / expanded 的扫描布局和地图包。

参数与话题见 [Gazebo 场景说明](../docs/GAZEBO_HARMONIC_SIMULATION.md)，场地布局见 [无人机障碍场地](../docs/UAV_OBSTACLE_COURSE.md)。
在启动终端按 `Ctrl+C` 退出，等待 Gazebo 结束后再启动另一场景。原终端已关闭或仍有残留进程时，按 [停止正在运行的进程](../docs/EGO_NVBLOX_GAZEBO.md#停止正在运行的进程) 查找 PID 并逐级退出。

## Isaac Sim UI

独立入口：`./simulation/scripts/run_isaac_sim_45_ui.sh`，需已有对应安装。
参见 [Isaac 内容说明](../docs/ISAAC_ROS_COMPONENTS.md)。

## 全链路仿真开发

按 [开发计划](../docs/SIMULATION_DEVELOPMENT_PLAN.md) 推进，当前已完成 S0 基础链路并开始 S1 首批状态聚合（S0 感知样例仍待补齐）。已实现 `scripts/sim.sh doctor` 离线依赖审计和主机回归运行器，使用方法见 [验收工具说明](acceptance/README.md)，实测证据见 [S0 报告](../docs/validation/simulation/2026-10-07-s0/REPORT.md)。当前全部在本机进行仿真开发，Jetson 验证延期为可选阶段；QGC 文件版本/hash 已核验，PX4 SITL v1.16.2、Agent v2.4.3 和对应消息包已构建并通过未解锁 x500 基础链路冒烟，使用方法见 [本机 SITL](px4/README.md)，尚未进入飞行验收。

S1 的只读 AircraftState、独立构建与故障验证入口见 [状态聚合器说明](../src/uav_mission/README.md)，实测边界见 [S1 首批报告](../docs/validation/simulation/2026-10-07-s1-aircraft-state/REPORT.md)。
