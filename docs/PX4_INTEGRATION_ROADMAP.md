# PX4 官方链路三阶段集成

2026-10-09 开发顺序调整（用户指定）：**优先实现完整任务与导航／避障链路，再细化 VIO 稳定性；相机固定下偏 5°**。相机参考 profile 与默认生成器已使用 5°，Camera pose / optical TF 同步生成。0°／15°／30° 仅保留历史复现入口，不再作为当前调参方向。5° 尚未通过 VIO 飞行源验收，历史角度的通过证据不迁移到新配置。

功能开发先使用官方 PX4 SITL 的 GNSS／惯性 EKF 定位，复用唯一 FlightServer 和现有 BT；VIO 作为独立集成／验收工作。所有运行显式记录定位来源，定位源不得运行中静默切换。S6 的 VIO 验收继续保持未完成。

当前基础任务已实现起飞、航点、悬停、返航、原生降落及暂停／取消／Runner 故障语义。后续优先补齐：同机 5° 深度／CameraInfo／TF 与 nvblox 地图会话；EGO 曲线到 PX4 local 的采样跟踪与控制权绑定；暂停／取消／重规划交接；绕障、不可达、地图过期与最终落地独立验收。W0 已知区域航点不能作为 EGO 避障验收。开发不再等待 VIO 稳定性，但地图／未知空间／碰撞／控制新鲜度门限继续执行。

本轮官方 EKF 无界面完整 BT 任务回归通过（起飞／两航点／悬停／返航／降落），带 UI 轮在解锁前触发时间／状态保护。见 [任务优先基线回归](validation/simulation/2026-10-09-task-first-baseline/REPORT.md)。

以下较早的阶段记录保留为历史证据。


640×400 仓库 VIO 同低负载配置 120 s 运动验证已通过（RMSE 2.60 cm，最大 3.78 cm），显式 0.8×／NVIDIA 无界面／图像深度 1／EKF 最大延迟 160 ms 下同机 120 秒融合连续两轮通过（READY 约 114.02 s）。默认／UI 配置仍有失败。仓库 0.8 m BT VIO 起飞／15 s 悬停／原生降落连续两轮通过，1.5 m 仍因对齐协方差失败，导航尚未执行。见 [长时融合报告](validation/simulation/2026-10-09-warehouse-queue-latency/REPORT.md)。

低负载运动与空中持续流策略的验证边界见 [运动报告](validation/simulation/2026-10-09-warehouse-low-load-motion/REPORT.md)。空中持续 EV、独立场景摘要准入、唯一 FlightServer/BT 已接入；低高度真实飞行已通过，原定高度仍待验收。见 [最新飞行报告](validation/simulation/2026-10-09-warehouse-bt-flight/REPORT.md)。

新增仓库场景与 staged 验收入口；初始六轮 VIO 前置实测均失败，仓库悬停／导航未执行。
见 [仓库说明](WAREHOUSE_SIMULATION.md) 与 [原始证据](validation/simulation/2026-10-09-warehouse-vio/REPORT.md)。

最新优化增加 480×300 仿真双目与显式 `bounded_gap` 协方差坏样本拒绝策略，
保留 200 ms 原有效样本期限。最终 0.8×、120 s 未解锁融合一轮通过（连续 READY
114.2 s），同配置仍有跟踪新鲜度失败；120 s 独立运动通过。FlightServer 已接入
显式 `aligned_pose_v1` 准入门控，持续 EV 飞行会话与真实悬停仍待验收。见
[负载优化与准入接入报告](validation/simulation/2026-10-09-vio-optimization/REPORT.md)。

SDK 同步器毫秒/纳秒换算与位姿审计源时钟已修复；SDK 重建及 222 项测试通过。
实际 reset、120 s 独立载台运动（位置 RMSE 1.36 cm）和 50 s 未解锁融合通过；
1.0/0.8 倍的 120 s 融合仍因新鲜度/协方差失败，S6/VIO 悬停仍未验收。见
[同步器与审计时钟修复报告](validation/simulation/2026-10-09-sync-clock-fix/REPORT.md)。

SDK 分段时序现已采集：默认实时轮存在至少约 388 ms 的 UpdatePose 回调外延迟，
缩短图像队列后有所下降，但四轮整体仍未通过。另复现同步器毫秒/纳秒阈值不匹配，
以及减速审计的墙钟计数边界；修复与重跑仍待推进。见
[SDK 时序及接口复现](validation/simulation/2026-10-09-vio-sdk-timing/REPORT.md)。

本轮继续修复新鲜度交接：接收路径检查当前样本，watchdog 保留旧样本期限与失效锁存。
新增被动 DDS/回调时序记录和 NVIDIA GLX/EGL 对照；实时延迟已定位到 SDK 位姿发布之前，
内部同步/跟踪/调度原因仍待剖析。当前 0.8 倍速 120 s 融合（2329 输入）和实际 reset
回归通过，实时长时及 S6/VIO 悬停仍未通过；平面场景协方差失败保留。见
[新鲜度修复与时序报告](validation/simulation/2026-10-09-vio-timing/REPORT.md)。

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

按当前优先级，先在官方 SITL GNSS／惯性 EKF 与已验证 FlightServer 上实现深度感知、nvblox、EGO 和任务编排；VIO 飞行稳定性不再是功能开发前提。真实 VIO 版本仍需单独通过阶段二验收：

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
