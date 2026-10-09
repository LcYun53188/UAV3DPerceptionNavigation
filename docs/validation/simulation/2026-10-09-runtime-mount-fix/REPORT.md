# 仓库运行时相机外参与新鲜度边界修复

640×400 仓库 VIO 已通过 120 秒独立载台三轴运动、转向、原时间戳与归一化源验收（WH-V02）：位置 RMSE **1.64 cm**、最大 **2.52 cm**、姿态最大 **0.664°**，归一化位姿 2734 条。**同机 120 秒 PX4 融合 WH-V01 仍未通过，仓库 VIO 悬停、导航与避障尚未运行。**本轮均未解锁，运动轮无 FMU 输入；融合轮只有 EV 位姿输入，已关闭所属进程和窗口。

## 两项已确认并修复的问题

1. `frames.json` 与 SDF 已使用下倾 15°，但 `SensorAudit` 发布静态 TF 时仍写死为水平相机四元数。实际 SDK 使用 TF 获取外参，因此下倾场景的观测与 SDK 标定不一致。发布器现按每个 frame 的 RPY 计算四元数；订阅 `/tf_static` 收到的实际变换写入 `result.runtime_mounts`，作为验收项。独立矩阵比较与真实 ROS 广播/接收测试同时覆盖该路径。
2. 实测日志中 `37.6-37.4` 与 `34.2-34.0` 得到 `0.20000000000000284`，恰好 200 ms 的原始样本被浮点比较误判超时。归一化器的跟踪、位姿和最后有效样本年龄检查现使用原始 ROS 整数纳秒。200 ms 年龄与 50 ms 未来界限仍包含边界；超过任一界限 1 ns 必须拒绝。协方差、墙钟接收期限、重启/reset/失效锁存均保留。

更正上一轮报告：此前“下倾 SDF 与 TF 一致”的测试只覆盖生成资产，没有覆盖实际广播器。`pitched-vio` 和 `pitched-stereo-diagnostic` 的运行时相机外参错误，不能据此归因 IMU 融合算法或比较两个算法模式的优劣。旧原始证据保持原值，旧报告已添加更正说明。

## 实测结果

各轮为独立运行，非同一录制输入的 SDK 回放；不能把不同初始化/调度的差异归为唯一因果。地面纹理与 15° 下倾均显式启用，未改默认模型。SDK、IMU 参数和质量门槛没有调低。

| 归档目录 | 配置与观察 | 结果 |
| --- | --- | --- |
| wrong-mount-sdk-dump | 修复前错误 TF，480×300，120 s SDK 输入转储 | 原始 RMSE 15.35 cm；归一化源失败；仅诊断 |
| fixed-mount-motion | 修复 TF，480×300，120 s，实时 | 原始 RMSE 2.84 cm、最大 5.36 cm、姿态 0.987°；原始运动精度通过，但协方差持续异常，整体验收 FAIL |
| fixed-mount-motion-640 | 修复 TF，640×400，120 s，实时 | **WH-V02 PASS**；原始与归一化运动检查均通过 |
| fixed-mount-fusion | 修复 TF，480×300，120 s，实时 | WH-V01 FAIL；421 EV 输入，连续 READY 14.84 s，最终 VIO_SAMPLE_STALE |
| fixed-mount-fusion-640-paced | 修复 TF，640×400，120 s，0.8×，纳秒边界修复前 | WH-V01 FAIL；648 EV 输入，连续 READY 30.30 s，融合源 VIO_RECEIVE_GAP；归一化器另外记录恰好 200 ms 的误拒绝 |
| fixed-mount-fusion-ns-clock | 同上，纳秒边界修复后 | WH-V01 FAIL；805 EV 输入，连续 READY 38.14 s，最终 VIO_SAMPLE_STALE；归一化器记录 **40.564−40.36=204 ms** 的真实超限，必须拒绝 |

实际位置、高度、航向融合均有 PX4 aiding 证据；三轮分别 419/645/803 个被 EKF 采纳的样本（各类均相同）。三轮均没有速度融合或飞行控制输出。源失效后停止 EV，随后本地估计 reset 锁存；不能通过重新绑定旧会话继续任务。READY 要求为连续至少 110 s，因此这些短时融合不能算完整通过。

480×300 修复后 SDK 位姿仍有周期性协方差超过门槛；只延长初始化等待不能消除后半段异常。640×400 本轮运动验收通过是一次实测结果，不等于已证明所有场景/负载均稳定，更不替代同机融合与悬停验收。

## SDK 实际输入核验

新增显式 `--sdk-debug-dump` 使用锁定 SDK 的调试接口记录其实际消费的双目、IMU、内外参和运行模式。`audit_vio_sdk_dump.py` 保留原始时间，检查双目配对、严格时间顺序和逐图像字节摘要；不拟合偏置、时延或尺度，也不把 Gazebo 真值传入 SDK。

修复前转储含 2998 对双目图像、29961 条 IMU：图像最大间隔 40 ms，IMU 最大间隔 4 ms，时间严格递增且双目同戳。原生外参文件的相机旋转为 identity，与下倾渲染不符；这条证据促成运行时发布器排查。`stereo.edex` 的初始 sequence/frame_end 不代表整个录制长度，实际计数来自 `frame_metadata.jsonl`。

修复后 50 s 诊断转储含 1254 对双目、12522 条 IMU，最大间隔分别 40/8 ms。SDK 原生记录的左右相机相对 IMU 的旋转角均为 15°，基线 0.075 m，IMU 旋转保持 identity，确认实际消费了修复后的外参。该轮归档为 `fixed-mount-sdk-dump`；时长不足 120 s 且有记录负载，不纳入 WH-V02 资格。元数据不记录每次 Track/RegisterImu 调用批次的先后，不能由它宣称精确调用序列回放或物理同步。

输入转储改变负载，因此入口即使内部检查通过也强制 `passed=false`，诊断结果另存 `sdk_dump_checks_passed`，仓库验收器再次拒绝诊断模式。全量图像留在缓存，不写入 Git；归档保留全部原始元数据、每张图像摘要和首对原始图像。归档重放验证元数据及首对图像，不声称从 Git 可以复放全部 SDK 图像。原生 SDK 同输入算法回放对照仍未实现。

## 验证与复现

- 仓库、生成资产、输入转储和真实 ROS TF 测试：23 项通过。
- PX4 桥接模块全部测试：123 项通过，包含浮点边界复现、整数纳秒正好边界和越界 1 ns、坏协方差不续期及失效锁存。
- vendor 两组只读检查通过；未修改 vendor 源或 gitlink。旧仓库诊断原始字节与误差重放通过；新归档按独立计算重放误差、归一化/融合与仓库门槛。

```bash
# 本轮通过的运动配置：
./scripts/sim.sh px4-vio-sensors --normalize --motion --scene warehouse \
  --warehouse-floor-texture --camera-pitch-deg 15 --image-resolution 640x400 \
  --quality-policy bounded_gap --duration 120 --ui
# 同机融合仍在排查，不能以它授权飞行：
./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene warehouse \
  --warehouse-floor-texture --camera-pitch-deg 15 --image-resolution 640x400 \
  --quality-policy bounded_gap --real-time-factor .8 --duration 120 --ui
# SDK 输入诊断，预期整体验收为 FAIL：
./scripts/sim.sh px4-vio-sensors --normalize --motion --scene warehouse \
  --warehouse-floor-texture --camera-pitch-deg 15 --image-resolution 640x400 \
  --quality-policy bounded_gap --sdk-debug-dump --duration 50 --ui
python3 scripts/audit_vio_sdk_dump.py <run>/sdk-input --output /tmp/sdk-input-receipt.json
python3 docs/validation/simulation/2026-10-09-runtime-mount-fix/check_archive.py
cd docs/validation/simulation/2026-10-09-runtime-mount-fix
sha256sum -c SHA256SUMS
```

下一步先按融合失败前的 SDK/ROS/DDS 原时间序列定位真正超过 200 ms 的调度与发布延迟，并补齐 WH-V01 长时融合；通过后再实现仓库飞行 profile、持续 EV 会话与唯一 FlightServer 控制的 BT 起飞／悬停／降落。随后做航点／返航与导航避障。仿真使用理想针孔双目/IMU参考模型，并非 OAK-D Pro W 的实机标定或飞行验证。
