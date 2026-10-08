# 实际双目／IMU → cuVSLAM 静止跟踪（2026-10-08）

本轮交付独立参考传感器模型、纹理场景、受管运行入口和定量审计。
**实际 Gazebo 双目／IMU已驱动本机 cuVSLAM VIO-only，静止前置检查通过；
真实 VIO→PX4 EKF→BT 悬停仍未完成，S6 未通过。**

代码：`5bbf0df`（传感器、运行器与审计）、`3cd6453`（源契约拒绝）。

## 配置与输入来源

本机 PX4 v1.16.2 官方 SITL、Gazebo Harmonic、QGC、ROS 2 Jazzy。
cuVSLAM SDK `15.0.0+74f0e317-modified`，实际节点 tracking_mode=1，日志确认 IMU fusion。
每轮 domain 78、instance 7、独立随机 Gazebo partition；QGC 禁用 USB 自动连接。
运行器复用既有仿真互斥锁和所属进程组清理，不终止其他会话。

`simulation/px4/vio/sensors.json` 固定理想针孔双目 640×400、水平 FOV 1.21 rad、
基线 0.075 m、25 Hz；IMU 250 Hz，安装点 FLU (0.12,0,0.242) m。
左右光学坐标系为右/下/前，左相机位于机体 +Y；基线由 TF 提供，不在 CameraInfo
投影矩阵中重复计入。相机 RGB 图像来自 Gazebo 渲染，IMU 来自 Gazebo 物理传感器。
位姿来自实际 cuVSLAM，没有注入真值、PX4 回读位姿或固定合成观测。

模型合并上游 x500，传感器和固定随机种子的三面纹理墙生成到本次缓存，
不修改上游模型、原 W0 参数/二进制或 VIO 遥测构建。清单绑定源文件、配置、
生成资产、节点/库/SDK 与锁定 PX4/Agent 构建 hash。

这是参考模型，**不是 OAK-D Pro W 的设备标定模型**：未模拟广角畸变、曝光、
测量噪声、USB 延迟或硬件时钟。IMU 是理想输入，算法使用上游默认噪声假设，
不能解释为 Pro W 的实测噪声。静止初始化也不替代运动激励和在线标定验证。

## 实测结果

| 归档目录 / run_id | 结果 | 最近稳态窗口 |
| --- | --- | --- |
| headless / `0fec16b0-64d1-432b-960d-0eecf1996f19` | PASS | 左/右/定位 25 Hz，IMU 250 Hz；双目匹配 100%；漂移 0.08048 m |
| ui / `187826c8-67e3-4dff-8897-a6dc9469f0a2` | PASS | 左/右/定位 25 Hz，IMU 250 Hz；双目匹配 100%；漂移 0.06288 m |

每轮观察 35 s，验收最近约 5 s。UI 轮图像/定位最大采样间隔 0.040 s、IMU 0.004 s，
双目配对最大时间差 0；均满足采样率 ±10%、最大间隔两周期和双目配对 ≥95% / ≤1 ms。
检查源时间戳严格递增、年龄 ≤0.2 s、CameraInfo 尺寸/焦距/中心/畸变/校正、
图像布局与非空纹理、IMU frame 与静止重力方向、持续 vo_state=1、唯一发布者。
IMU 稳态均值约 (0,0,9.8) m/s²，角速度约零。

漂移是最后 5 s 估计位置相对窗口起点的最大距离，门限 0.15 m；
**不是对独立真值的绝对误差或位置 RMSE**，不证明动态精度或缓慢漂移可在线识别。
headless 轮早于 PPM 快照导出这一运行器变更，审计/质量/模型/算法 hash 与当前一致；
UI 轮全部输入 hash 匹配当前文件，详见 `current-input-check.json`。

两轮 QGC 均确认连接系统 8；PX4 全程 arming_state=1 且着地。
vehicle_command、trajectory_setpoint、offboard_control_mode、vehicle_visual_odometry
发布者均为零。没有 FlightServer、Offboard 心跳或解锁命令。
所属根进程/进程组清理检查通过；QGC 在 8 s 正常退出期限后被所属组 SIGKILL，
不声称所有进程优雅退出。UI 轮保存原始 RGB PPM 和无损 PNG，图像显示固定纹理墙及地面。

## 保留的失败与修复

- `initial-agent-failure/`，run `5f290193-85ac-4496-9965-fd84f08cde77`：
  Agent 缺少 libmicroxrcedds_agent.so 的库路径，PX4 状态审计失败；
  cuVSLAM 使用 best effort 图像订阅，日志多次出现 80–640 ms 输入间隔。
  修复包装器 LD_LIBRARY_PATH，并将本机大图订阅设为 reliable；不放宽时效门限。
- `observer-qos-failure/`，run `3f6c77ca-1df2-45b2-8e0d-2291bfb7c955`：
  算法定位已为 25 Hz，但审计器自身 best effort 图像订阅只有约 15.5/12.5 Hz，
  最大间隔 0.36/0.68 s，配对约 63%。审计正确判失败，修复审计器订阅 QoS。
- 早期仅看最新双目帧和输出存在性的运行不作为最终验收；现在对稳态窗口检查
  速率、间隔与匹配，并排除末端一个回调尚未送达的边界，而非放宽实际同步误差。
  其他诊断缓存保留在 `.cache/simulation/vio-sensors/`。

## 原始协方差契约问题与拒绝门控

实际上游 `visual_slam_impl.cpp` 的 Odometry pose/twist 协方差来自 PoseCache /
VelocityCache 的滑窗样本统计；静止时可能为零，也可能是很小的正值。
UI 轮 `raw_odometry_numeric_conversion` 可通过原有数值检查，这不代表观测精度可信。
SDK 位姿观测协方差另外发布在 `vo_pose_covariance`；参考点/基变换还需核验。
速度由旧窗口机体系的相对位姿差产生，不应直接冒充当前机体系瞬时速度及其误差。

适配器默认输入改为 `/uav/vio/odometry`，默认 source_contract=unverified；
未核验的契约拒绝就绪，明确禁止直接使用 `/visual_slam/tracking/odometry`。
只有核验标准 ENU/FLU 坐标、当前机体速度及观测协方差语义后，才能显式声明
standard_enu_flu_odometry_v1。该声明是受信任本机配置，不是自动标定证明；
当前没有标准化发布者，真实 VIO 不输出至 PX4，也不获准飞行。

`transport.json` 是独立 domain 92 的合成 DDS 测试：未验证契约拒绝、明确契约通过、
跟踪丢失拒绝、恢复与发布者更换锁存通过；没有 FMU 发布者。它不是相机/融合证据。

## W0 控制回归与软件检查

`w0-full/`，run `b0c71707-ada3-4dbc-bc82-3086f4e67fbb`：当前源契约代码下运行
`px4-flight --bt --ui`，真实根任务 SUCCEEDED / LANDED_AND_DISARMED，树成功，
步骤 0–6 各授权/完成一次，最大真值位移 4.18982 m。四段到点误差最大 0.08522 m；
两段悬停 32.01393 s / 0.07789 m、5.03389 s / 0.08909 m，最终着地解除武装。
这是模拟 GNSS/惯性 W0 控制回归，没有运行 VIO；本轮未重新执行另外三个 BT 故障用例。
旧四用例归档的输入 hash 随 VIO 源契约文件变更而过期，不能将其重放声称为当前四用例复验。

- 任务包构建：4 packages finished。
- 源契约/转换/连续性、采样故障、双目三角几何、外参与生成资产回归：43 passed / 0.34 s，`pytest.xml`。
- 新包装器 bash 语法与 Python 编译检查通过。
- managed vendor / EGO 检查通过；已有供应商子模块状态保留。

归档采用明确白名单，含失败与最终运行日志、参数、输入清单、原始采样/定位、
参考资产和 W0 控制证据；不包含授权私有 JSON、QGC 配置/缓存或 rootfs。
`payload-sha256.json` 绑定压缩前数据，`SHA256SUMS` 校验归档文件；PNG 由原始
PPM 无损导出，无图像合成或画面修改。

## 复现与下一验收

```bash
./scripts/build_px4_sim.sh --jobs 4
./scripts/build_px4_flight.sh
./scripts/build_vio_node.sh
./scripts/sim.sh px4-vio-sensors --ui --duration 35
```

入口只有未解锁静止验证模式，参数 duration 范围 20–120 s。
后续先实现标准位姿/速度/协方差源、reset 代理与会话撤销，完成三轴平移/旋转、
初始对齐后位置 RMSE/最大误差及退化验证，再接实际四类 EV 融合与现有 BT 的
起飞→30 s 悬停→原生降落。阶段二通过后才接导航与避障。
