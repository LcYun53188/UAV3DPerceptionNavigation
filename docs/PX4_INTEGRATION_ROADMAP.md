# PX4 官方链路三阶段集成

按用户指定顺序推进，使用本机 PX4 官方 SITL + Gazebo Harmonic + QGroundControl；
当前固定 PX4 v1.16.2、ROS 2 Jazzy。Jetson 延期，不依赖实机或接收机供电。
每阶段单独配置、证据和 Git 提交；前阶段通过不替代后阶段验收。

## 阶段一：行为树与真实飞行控制

入口：`./scripts/sim.sh px4-flight --bt --ui`。当前树由 recipe 生成
StartFlight → FlightStep[0..6] → AwaitFlightResult。根 Action 只提交一次，
各叶通过带根 UUID、控制会话/世代、步骤类型和索引的服务授权；不是独立步骤 Actions。
真实飞机运动和 FMU 写入由唯一 FlightServer 负责，树本身不直接发布控制。

冻结 W0 已知空旷区域、x500 动力学、PX4 本地估计及原 GNSS/惯性配置。
真值只用于独立审计，不作为 PX4 控制定位。本阶段不启动相机定位或未知障碍避障。

| 必需用例 | 验收 |
| --- | --- |
| 完整任务 | 起飞、两航点、30 s 悬停、返航、短悬停、原生降落；步骤 0–6 各授权/完成一次；根和树成功；实际着地未解锁 |
| 暂停恢复 | NAVIGATE 中暂停，确认停稳后保持约 3 s，再恢复同一叶的新物理子 UUID；步骤不重跑、不跳步 |
| 运动中取消 | Runner halt，根 CANCELED / STOPPED_AND_HOLDING；没有后续步骤授权；有界保持后实际降落未解锁 |
| Runner 停滞 | 实际 SIGSTOP，网关 ABORTED / BT_PROGRESS_TIMEOUT；没有后续步骤授权；有界保持后实际降落未解锁 |

物理验收沿用航点误差 ≤0.3 m、悬停/暂停/停止保持漂移 ≤0.15 m。
暂停须有至少 2.5 s 的 PAUSED 真值覆盖，不能只看服务 ACK 或报告中的 passed。
停止保持约 30 s；Runner 进展期限 0.5 s，独立汇总接受最后进展至制动 ≤0.75 s
（含调度余量，不改变控制器期限）。原生降落不被取消或 Runner 消失中断。

契约回归覆盖拒绝、重复请求、跳步、旧身份/世代、halt、提前根成功与终态消费。
四轮实飞使用相同实现和 recipe；`scripts/assess_px4_bt_suite.py` 独立检查根/树终态、
步骤、物理停止证据及实现 hash。仅有 QGC 连接、topic 或 XML 不算通过。
结果见 [本轮 BT 验证](validation/simulation/2026-10-08-bt-stage1/REPORT.md)。

## 阶段二：VIO 定点悬停

首个闭环定义为 **BT 起飞 → VIO 辅助定点悬停 → 原生降落**。
使用 PX4 位置 Offboard 控制，继续由唯一 FlightServer 输出。原生 Position 模式的
手动悬停可后续单独验证；Stabilized 模式本身不提供自动位置保持。

本机 VIO 节点已通过构建、CUDA 加载，以及真实 Gazebo 双目／IMU静止跟踪前置检查，见
[传感器跟踪报告](validation/simulation/2026-10-08-vio-sensors/REPORT.md)。
原始 cuVSLAM Odometry 的滑窗协方差不作为 EKF 观测不确定性；当前禁止直接适配。
SDK 位姿协方差归一化及 reset 代理已交付，短时静止与实际 reset 撤销验证通过；
90 s 检查因 SDK 协方差超限失败，旧源按设计锁存失效。尚无标准速度观测，当时也未交付 PX4
初始化对齐；尚未进行 VIO 飞行验收，见
[位姿与 reset 记录](validation/simulation/2026-10-08-vio-pose/REPORT.md)。
独立力驱动载台已完成实际 VIO 三轴/转向运动验证，多深度场景通过，平面场景
仍因协方差超限失效；不代表 PX4 闭环或 Pro W 标定，见
[运动对照报告](validation/simulation/2026-10-09-vio-motion/REPORT.md)。
固定仿真锚点的初始化对齐、保守协方差传播和显式仅位姿门控已实现，未解锁的
合成仅位姿输入已通过实际 EKF 融合/停更审计，见
[位姿融合报告](validation/simulation/2026-10-09-vio-pose-fusion/REPORT.md)。

实际同机 SDK 位姿现已接入未解锁的受管融合入口：0.8 倍目标速度、120 s 墙钟检查
通过三类实际融合、PX4 速度健康及停止源后的锁存失效；默认无 EV 观察入口回归通过。
两轮实时长时检查因源超时失败，故实时性能、同机运动、VIO 飞行与故障落地仍待验收，见
[实际融合记录](validation/simulation/2026-10-09-real-vio-fusion/REPORT.md)。

1. 固定模拟双目和 IMU、CameraInfo、内外参、安装 TF、采样率与共同仿真时钟。
   Pro W 是目标硬件；参考模型必须标明与真实广角/基线/IMU 的差异。
2. 运行本机真实 VIO 算法，将核验后的标准 odom 进入现有适配器；补齐 reset 服务代理、
   源会话撤销、轴向/速度/协方差和时间检查。不能用真值或旧位姿重发代替算法输出。
3. 使用独立 VIO PX4 构建导出的 selector 和四类 EV aid；实际 fused、创新、
   最后融合时间和源健康连续 2 s 才准入，源年龄 ≤0.2 s。
   flags/selector 约 1 Hz，接收/源年龄 ≤1.5 s；其余融合遥测 ≤0.5 s。
   若选择仅融合 SDK 位姿，显式审计配置已完成同机未解锁融合，飞行准入前仍须核验位置/
   高度/航向融合与 PX4 估计速度；保留当前四类门控的默认要求，不伪造速度观测。
4. 冻结 GNSS、磁、光流等辅助的启用状态，并在实际 EKF 中核对。气压高度若保留，
   明确报告；不能将 GNSS 悬停称为 VIO 悬停。
5. 静止、三轴移动、旋转和 reset 先未解锁验收，再执行 BT 起飞/30 s 悬停/降落。
   初步采用现有悬停漂移 ≤0.15 m；另按计划验收 VIO 位置 RMSE ≤0.15 m、
   最大误差 ≤0.30 m，仅初始化对齐，不允许逐段拟合掩盖漂移。
6. 注入输入停更、跟踪丢失、reset、时钟异常：撤销旧授权/轨迹，不自动恢复旧任务；
   实际验证失去定位后的 PX4 处置和落地状态，不能以静态门控测试代替飞行验收。

已有独立合成 EV→实际 EKF 审计通过，仅作为通信/融合前置证据；不算本阶段通过。
详见 [VIO 实现及边界](VIO_HOVER.md)。当前静止合成输入禁止用于飞行。

## 阶段三：导航与避障

在阶段二的真实定位与飞行闭环上接入深度感知、nvblox、EGO 和任务编排：

1. 验证深度/CameraInfo/TF/定位时间一致及可观测起点安全体积，再建立地图会话。
   不能将算法仿真的真值定位或零重力 cmd_vel 路径直接接入 PX4。
2. 复用 PlanningContext：绑定 map_id/epoch/version、localization_session、
   reset 计数、alignment_id/generation；坐标、会话或地图失效撤销旧曲线。
3. EGO 曲线经独立扫掠碰撞、未知空间和速度/加速度/jerk 校验后，交给唯一 PX4
   网关跟踪。先停止状态派发，再验证运动中重规划/交接；影子规划不等于实飞避障。
4. BT 调度起飞、导航、避障/重规划、悬停、返航和原生降落；保持暂停/取消/
   Runner 失联语义。导航失败不能无限重试或绕过未知区域。
5. 在独立障碍场景验收可达航点、绕障、不可达、地图过期、定位退化、规划超时、
   暂停/取消、返航通路变化和最终落地。记录实际位移、独立真值净空及任务终态。

现有 EGO 影子规划保留为前置测试，输出不控制 PX4。
详见 [规划上下文](PLANNING_CONTEXT.md) 与 [完整仿真计划](SIMULATION_DEVELOPMENT_PLAN.md)。
