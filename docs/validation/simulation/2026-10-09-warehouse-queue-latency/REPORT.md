# 仓库长时 VIO 融合：队列、渲染与 EKF 延迟对照

**WH-V01 已在显式低负载配置下连续两轮通过**：每轮 120 s 墙钟、连续 READY 约 114.02 s，实际位置／高度／航向融合与停源失效检查通过。配置为仓库地面纹理、相机下倾 15°、640×400、`bounded_gap`、0.8 倍速度、NVIDIA 无界面渲染、SDK 图像订阅深度 1、`EKF2_DELAY_MAX=160 ms`。默认配置和带 UI 配置仍有失败，不能宣布实时或 UI 条件下稳定。

全程同一架未解锁 x500 的真实仿真双目／IMU经 cuVSLAM、归一化、固定初始对齐进入 PX4 EKF。没有飞行控制输出、速度观测、硬件操作或旧源自动恢复。仓库 BT VIO 起飞／悬停／导航尚未运行，所有仓库验收仍输出 `flight_authorized=false`。

## 已确定的原因与改进

- 上轮失败前 SDK Track 连续约 102/159 ms，归一化回调约 0.2 ms；SDK 上游排队与跟踪占用了更新预算。系统时钟在故障之后另跳变约 499 ms，使整段 DDS 比较失去解释资格。新增显式单调时钟窗口分析，故障前稳定窗口仍可分析；完整报告保留 `host_clock_stable=false`，不掩盖时钟跳变。
- NVIDIA 渲染单独开启仍失败。图像队列深度 1 降低积压，但带 UI 轮桥接仍在旧样本 204 ms 时停止；下一条新鲜归一化位姿晚约 14.52 ms，下一帧 Track 约 139.93 ms。该轮 SDK 归一化源整体通过，不代表桥接及时。`diagnose_vio_deadline.py` 区分 SDK 撤销、旧样本到期、新位姿发出及桥接收到时刻，计划停源单列，不当作运行性能故障。
- NVIDIA 无界面、深度 1 时原始融合检查通过，但 READY 在中途被短暂的融合证据／当前源时间差打断，最长仅 65.90 s。实际 ULog 确认最大延迟配置 200 ms、EV 延迟偏移 0；遥测中融合时间域落后当前 PX4 时间约 156 ms，个别融合证据与最新源相差 204/244 ms，仓库连续 110 s 门槛仍失败。
- 新增仅限未解锁融合审计的显式 `--ekf-delay-max-ms {160,200}`。PX4 锁定源码的 `EKF2_DELAY_MAX` 定义为当前时间与延迟融合时域之间的最大延迟，不是 EV 时间戳校正。160 ms 对照的实际融合时域约落后 124 ms；两轮完整 ULog 均确认 160 ms，EV 延迟偏移仍为 0，原外部视觉采样时刻没有改写。该设置依赖当前链路的实测延迟，不是实机通用标定参数。

这些是独立运行，非同输入回放；顺序对照与重复通过支持当前显式配置，不能把渲染、调度和算法内部耗时中的某一项认定为唯一根因。第一轮 160 ms 对照的整段 Track 最大值含启动期间约 542 ms 慢帧，原数据保留；它不是已绑定 READY 阶段的截止时间证明。

## 全部原始结果

每轮 120 s 墙钟、0.8 倍目标速度、640×400、相同仓库／下倾／纹理／质量门槛。

| 目录 | 配置 | 原始整体 / WH-V01 | EV 输入 | 最长连续 READY |
| --- | --- | --- | --- | --- |
| intel-depth10 | 上轮默认渲染、UI、深度 10 | FAIL / FAIL | 805 | 38.14 s |
| nvidia-depth10 | NVIDIA、UI、深度 10 | FAIL / FAIL | 717 | 33.16 s |
| nvidia-depth1-ui | NVIDIA、UI、深度 1 | FAIL / FAIL；归一化源通过 | 1333 | 56.26 s |
| nvidia-depth1-headless | NVIDIA 无界面、深度 1、最大延迟 200 ms | PASS / FAIL | 2315 | 65.90 s |
| nvidia-depth1-headless-horizon160 | 上述配置，仅最大延迟改为 160 ms | **PASS / PASS** | 2317 | **114.0203 s** |
| nvidia-depth1-headless-horizon160-repeat | 完全同配置重复 | **PASS / PASS** | 2322 | **114.0200 s** |

最后两轮位置／高度／航向各 2314/2318 个实际融合样本，DDS 回读与输入一致；速度保持 NaN、无 EV 速度融合。实际停止归一化器后，分别 105.98/104.53 ms 返回 `VIO_SOURCE_LOST`，随后旧源与 EKF reset 锁存，drain 后融合时刻不再前进。均实际清理所属进程。

## 配置与证据核验

新增 SDK 参数服务回读，确认实际图像深度 1、图像缓冲 100、IMU 缓冲 144、VIO 模式 1、双目与两个光学 frame。回读缺失／不匹配会使整体验收失败；这是配置快照，不是 DDS 队列占用或身份认证证明。

归档保留六轮原始结果、全部传感器／SDK／归一化／融合时序、遥测、资产及完整 ULog，并逐字节核验。ULog 全文件扫描包含后续参数变化；两轮通过结果的全部配置检查通过。`nvidia-depth10` 历史 ULog 未记录两个未显式覆盖的延迟参数，参数复核保留 `passed=false`，不从默认值补造实测证据；其余参数记录可复核，原始运行本身也是失败。

25 项相关测试通过，包括时钟窗口跨跳变拒绝、早期稳定窗口、正常 SDK 回读、深度不匹配拒绝、缺少参数拒绝、旧样本与晚到新样本的诊断、计划停源分类。旧时钟修复归档的 149 个原始文件及 14 项 ULog 参数重放通过。

```bash
# 本轮两次通过的配置；无 Gazebo/QGC 可见窗口：
./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene warehouse \
  --warehouse-floor-texture --camera-pitch-deg 15 --image-resolution 640x400 \
  --quality-policy bounded_gap --real-time-factor .8 --render-device nvidia \
  --sdk-image-depth 1 --headless-rendering --ekf-delay-max-ms 160 --duration 120
python3 scripts/assess_vio_warehouse.py <run> --output /tmp/warehouse-assessment.json
python3 scripts/diagnose_vio_deadline.py <run> --output /tmp/deadline.json
python3 docs/validation/simulation/2026-10-09-warehouse-queue-latency/check_archive.py
cd docs/validation/simulation/2026-10-09-warehouse-queue-latency
sha256sum -c SHA256SUMS
```

下一步补齐同一队列／渲染配置的运动精度验证，再实现仓库飞行 profile、允许初始化后在空中持续输出 EV 的受管会话，以及唯一 FlightServer 控制的 BT 起飞→悬停→原生降落。前置通过不替代飞行中源失效、暂停／取消和实际着陆的验收；随后才做航点／返航、建图导航与避障。当前仍是通用理想双目／IMU参考模型，不是 OAK-D Pro W 实机标定。
