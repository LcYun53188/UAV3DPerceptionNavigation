# 无人机导航文档索引

- [仿真启动与控制脚本](SIMULATION_CONTROL.md)：推荐日常入口，统一环境、进程管理、在线探索、地图保存加载、状态与日志排错。

- [项目说明与四种启动流程](../README.md#选择运行方式)：传感器场景、在线探索、离线扫描保存、加载地图导航。
- [项目评估与待办](UAV_PROJECT_ASSESSMENT.md)
- [无人机仿真使用手册](SIMULATION_MANUAL.md)：系统架构、环境准备、建图与地图包、RViz 交互导航、自动化回归及接口排错全流程。
- [EGO + nvblox 仿真运行指引](EGO_NVBLOX_GAZEBO.md)：模式选择与切换、在线探索、地图保存和加载、RViz 选点、停止及排错。
- [EGO + nvblox 开发手册](EGO_NVBLOX_DEVELOPMENT_GUIDE.md)
- [BehaviorTree 与 PX4 协同开发计划](BT_PX4_DEVELOPMENT_PLAN.md)：任务树、Action 接口、控制权、PX4 执行后端、阶段排期与验收门槛。
- [UAV 全链路仿真开发计划](SIMULATION_DEVELOPMENT_PLAN.md)：算法回归、PX4 动力学、室内 VIO、本机稳定性、场景矩阵和故障验收；Jetson 联调延期。
- [本机 PX4/BT 运行说明](../src/uav_bt/README.md)：真实飞行任务树、进展租约、取消清理与 Runner 故障注入。
- [PX4/BT 验证记录](validation/simulation/2026-10-08-bt-px4/REPORT.md)：S2 首批真实后端集成及适用边界。
- [算法 MissionServer 与两航点 BT](ALGORITHM_MISSIONS.md)：父会话、进展租约、暂停检查点与恢复。
- [算法任务验证记录](validation/simulation/2026-10-08-algorithm-mission/REPORT.md)：Gazebo 暂停恢复、取消竞态及 Runner 停滞。
- [算法导航 Action](NAVIGATION_ACTION.md)：请求契约、UUID 取消、停稳确认与普通客户端回归。
- [导航 Action 验证记录](validation/simulation/2026-10-08-navigation-action/REPORT.md)：Gazebo 两航点与运动中取消。
- [Isaac 系列保留范围](ISAAC_ROS_COMPONENTS.md)
- [安装与构建](INSTALLATION.md)
- [CUDA 构建环境](CUDA_TOOLKIT_13_2_INSTALLATION.md)
- [OAK-D / cuVSLAM 定位验证](OAKD_VISUAL_SLAM_RVIZ.md)
- [Gazebo 无人机场景](GAZEBO_HARMONIC_SIMULATION.md)
- [无人机障碍场地](UAV_OBSTACLE_COURSE.md)
- [PX4 串口通信](PX4_MICRO_XRCE_DDS_SERIAL_SETUP.md)
- [PX4 导航策略](PX4_NAVIGATION_STRATEGY.md)
- [PX4 状态机设计](PX4_STATE_MACHINE_DESIGN.md)

策略与状态机设计文档记录设计意图；当前实现的限制以项目评估为准。

- [达妙 USB IMU 与视觉里程计融合](DAMIAO_IMU_USB.md)：可选 USB 驱动、三维 EKF、单位与外参验证。
