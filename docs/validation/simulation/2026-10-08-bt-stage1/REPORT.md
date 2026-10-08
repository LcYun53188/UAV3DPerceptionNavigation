# 第一阶段：当前 BT/PX4 官方链路复验（2026-10-08）

用户确定继续 PX4 官方 SITL＋Gazebo Harmonic＋QGroundControl，并按
**BT 验证 → VIO 悬停 → 导航与避障** 顺序推进。本轮完成第一阶段的四项必需实飞验证，
独立汇总 **PASS**；不是 VIO 或感知避障验收。阶段边界见
[集成路线](../../../PX4_INTEGRATION_ROADMAP.md)。

## 配置与任务

本机 PX4 v1.16.2、Gazebo Harmonic、x500、固定 W0、domain 78 与各轮唯一分区。
四轮均使用 `--bt --ui`，由 supervisor 管理 Gazebo/QGC；QGC 连接由日志核对，
不声称人工完成所有 GUI 功能验收。与实机无控制连接；未改变硬件参数。
本轮控制使用 PX4 的模拟 GNSS/惯性估计，Gazebo 真值只作独立审计。
没有启用 VIO，没有将 algorithm 真值里程计或 cmd_vel 接入 PX4。

默认 recipe 为起飞至 2 m、航点 (3,2,2)、悬停 30 s、航点 (-2,2,2)、返航、
悬停 3 s、原生降落。航点为起点相对 ENU 偏移，经已有 map/odom 对齐变换。
BT 根 Action 只发一次，七个步骤通过服务逐次授权，由唯一 FlightServer 输出控制。
原生降落必须实际着地解除武装后才确认完成。

## 四轮结果

| 用例 / run_id | 根结果 | 关键证据 |
| --- | --- | --- |
| full / `9b5c56a6-b6b8-41e9-9c34-c5f1ae8b62a3` | SUCCEEDED / LANDED_AND_DISARMED | 步骤 0–6 各授权/完成一次，树 SUCCESS，落地未解锁 |
| pause-resume / `8405dc6a-cb44-4ee6-9396-037e73ddbd2c` | SUCCEEDED / LANDED_AND_DISARMED | 停稳暂停约 3 s、原叶恢复为新物理子 UUID；步骤 0–6 无重复 |
| cancel / `969804f2-f736-46de-b5a8-86e7d81d8b53` | CANCELED / STOPPED_AND_HOLDING | 仅授权 0、1，仅完成 0；有界保持后原生降落未解锁 |
| runner-stall / `46bb7c4e-143f-42ea-87bb-53ef17bd8345` | ABORTED / BT_PROGRESS_TIMEOUT | 实际 SIGSTOP；仅授权 0、1；制动、保持后原生降落未解锁 |

各轮 supervisor exit 0、DDS/clock/QGC 和物理 flight audit 均 passed=true，
真实后端 mock=false、cleanup_confirmed=true。取消/故障的 PASS 表示预期停止行为通过，
不是将取消/故障改写为任务成功；Runner 停滞时进程未自行完成，终态来自网关。

完整任务最大独立真值位移 **4.17386 m**；起飞/两个航点/返航误差依次约
0.09382、0.05500、0.02689、0.01763 m，均 ≤0.3 m。
两次悬停观测段含稳定窗口，分别 32.02874 s / 0.11270 m 和
5.03913 s / 0.04766 m，达到指定时长且漂移 ≤0.15 m。

暂停恢复最大位移 **4.17173 m**，暂停漂移 **0.02480 m**，PAUSED 真值覆盖 ≥2.5 s；
两个悬停段最大漂移 0.11075、0.10144 m。旧物理子 UUID 未恢复，BT 叶没有重复授权。

取消后保持约 **29.99483 s**、1500 个真值样本，最大漂移 **0.08940 m**，随后落地未解锁。
Runner 停滞后最后进展至制动 **0.50919 s**；保持约 **29.98338 s**、1500 个真值样本，
最大漂移 **0.09550 m**，随后落地未解锁，没有授权下一步骤。

## 独立验收与测试

`scripts/assess_px4_bt_suite.py` 检查四轮不同 run_id、相同且当前匹配的实现 hash、
冻结 W0 recipe/profile、根/树终态、步骤序列、非 mock、实际落地、取消/故障后的保持。
原审计记录暂停漂移但只将 pause_resume.passed 设为 true；本轮独立汇总另外要求
漂移 ≤0.15 m、有 PAUSED 真值覆盖和新子 UUID，避免仅依赖该布尔字段。
本批未放宽任何控制新鲜度或飞行门限，也未修改飞行后端。

- 桥、任务后端、BT 生命周期和既有飞行审计回归：**267 passed / 108.26 s**，`pytest.xml`。
- 新独立汇总回归：**28 passed / 0.05 s**，`assessment-pytest.xml`。包含提前成功、
  缺清理/落地、步骤错误、暂停漂移/NaN/缺真值/覆盖不足、失联后继续步骤、制动过慢、
  无落地，以及重复 run_id、源码不一致/漂移和压缩归档重放。
- 实际缓存评估 `assessment.json` PASS；对本目录压缩证据再次评估
  `assessment-from-archive.json` PASS。保存原始 payload hash，gzip 不影响语义校验。
- managed vendor/ego 检查通过；既有脏子模块保持不变。
- `process-audit.json` 确认四轮根进程及其进程组均退出。没有广泛清理其他会话。

本轮无需修复 BT/飞行控制逻辑；新增只读独立验收，代码提交 `ed3b755`、`06dae21`。

## 复现

```bash
./scripts/sim.sh px4-flight --bt --ui
./scripts/sim.sh px4-flight --bt --ui --flight-scenario pause-resume
./scripts/sim.sh px4-flight --bt --ui --flight-scenario cancel
./scripts/sim.sh px4-flight --bt --ui --flight-scenario runner-stall

python3 scripts/assess_px4_bt_suite.py \
  --full docs/validation/simulation/2026-10-08-bt-stage1/full \
  --pause-resume docs/validation/simulation/2026-10-08-bt-stage1/pause-resume \
  --cancel docs/validation/simulation/2026-10-08-bt-stage1/cancel \
  --runner-stall docs/validation/simulation/2026-10-08-bt-stage1/runner-stall \
  --output .cache/simulation/bt-stage1-replay.json
```

重放要求当前源码/固定构建与归档 hash 一致；它不重新飞行，也不能替代其他阶段验收。
归档采用明确白名单，保留树、事件、真值/估计、状态、命令和日志；不收录授权私有
JSON、QGC 配置/缓存或 rootfs。`implementation.sha256` 绑定代码/构建，
`SHA256SUMS` 校验归档文件。

## 后续阶段

第二阶段先完成本机 VIO 节点构建和双目/IMU仿真，再进入真实 VIO→EKF→BT 悬停。
本次检查 `install_uav/isaac_ros_visual_slam` 未提供已安装节点库；此前构建在
isaac_ros_image_proc 缺少 CV-CUDA 头文件处失败，需先解决依赖并实际加载/运行节点。
现有合成 EV 实际融合审计保持独立，禁止用固定合成位姿驱动飞行。
第三阶段在定位闭环通过后接入深度建图、EGO 曲线复检与唯一 PX4 网关，验证真实绕障。
本阶段通过不代表 S2–S4 全矩阵、S6 VIO、未知场景避障或实机验收完成。
