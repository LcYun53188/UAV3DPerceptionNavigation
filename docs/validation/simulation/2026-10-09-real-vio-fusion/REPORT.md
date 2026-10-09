# 实际同机 VIO→PX4 未解锁融合

2026-10-09，本机 PX4 v1.16.2 / Gazebo Harmonic / QGC / cuVSLAM 15。
已接通同一架 x500_vio_ref 的双目/IMU→实际 VIO→固定初始化对齐→PX4 EKF，
0.8 倍目标仿真速度下的 120 s 墙钟检查通过，另有默认无 EV 观察入口回归通过。
两轮 1.0 倍长时检查因源超时失效，实时性能尚未通过。未解锁、未运行 BT 飞行；
本报告不代表 VIO 悬停、导航避障或 OAK-D Pro W 硬件标定验收，S6 未完成。

## 实现与准入

`AlignedPoseStream` 精确配对 `/uav/vio/pose` 的原始纳秒采样时间与
`/uav/vio/pose_status.sample_stamp`，绑定源 UUID、calibration_id、reset 与端点 GID。
使用至少 2 s 静止窗口，要求实时、唯一的 PX4 未解锁/着地遥测，冻结一次 yaw/平移。
后续原始时间不重写、不重新拟合；源、端点、时间、协方差或地面条件异常后锁存。

监督器独占 instance 7、domain 78、XRCE 8898 和独立 Gazebo partition，核对锁定
W0/Agent 构建与独立 VIO 遥测构建。只有显式 `--normalize --fuse-pose` 才生成 EV
发布者，禁止与独立运动载台或 reset 组合。默认入口不输出任何 FMU 输入。
SDK 在模型生成前启动，观测物理生成/沉降；融合门控在模型、PX4、QGC 就绪后启动。
本机 SDK 完全静止后才启动的对照没有合格位姿协方差；这不是已证实的初始化根因。

观测为实际 SDK 位姿协方差归一化后的数据，无合成位姿、差分速度或 PX4 估计回灌。
使用 `aligned_pose_v1` 和已交付的一阶保守协方差上界，传播后方差仍不得超过 0.25。
固定仿真锚点为本地起始机体原点 ENU [0,0,0]、yaw 0，不是地形绝对高度，也不是
实机测量。anchor.json 绑定生成世界、机体、标定和融合配置摘要。
速度、角速度和速度方差均为 NaN，速度 frame 为 UNKNOWN；只融合位置/高度/航向。
PX4 自身速度仅用于健康准入，默认四类融合入口保留。

本机 rclpy 7.1.11 的回调元信息没有发布者 GID，此轮模式明确记录为
`unique_graph_only`：唯一图端点 GID + 原始 UUID/reset/标定绑定。有回调 GID 时另行
比较；该绑定不是来源密码学认证，也不等价于实机控制授权。

## 成功检查

| 用例 | 原始 run_id | 结果 |
| --- | --- | --- |
| 同机实际 VIO，layered，目标速度 0.8，120 s 墙钟 | 8652d28a-1641-48ce-a29c-2c1413048c68 | PASS |
| 原默认静止观察，normalize，planar，35 s，零 FMU 输入 | 4c889b57-60f1-468c-9a6b-598354262c12 | PASS |

主要数据位于 `actual-pose-paced/`，独立复核见 `independent-assessment.json`。

- 2326 个实际 VIO 输出、2326 个逐样本一致的 DDS 回读，原始采样时间范围
  8.12–101.12 s，连续跨度 93.00 s 仿真时间；据样本跨度/墙钟跨度测得速率比 0.79974。
- EV pos/hgt/yaw 各报告 2324 个 fused 样本，EV vel 无融合。门控最长连续 READY
  114.0603 s 墙钟；源正常时无图写入冲突，所有飞行控制发布者为零。
- 输入发布时采样年龄中位 40 ms、P95 60 ms、P99 76 ms、最大 116 ms，限值仍为 200 ms。
- 停源前 local 的 xy/z/vxy/vz、heading_good_for_control 均有效，无航位推算。
  eph/epv/evh/evv 分别为 0.06051/0.04171/0.06829/0.03764，heading_var=0.00021638。
  独立检查覆盖停源前约 5 s 墙钟的持续状态，以原始 PX4 时间检验 local 发布率。
- 源进程实际 SIGINT 后 24.803 ms，融合门控返回 VIO_SOURCE_LOST；图端点消失令
  源锁存 VIO_WRITER_COUNT。终态 VIO_EKF_LOCAL_RESET，是随后 PX4 本地重置导致的
  额外锁存，旧源不会恢复。
- 保持观察 6 s 墙钟；1 s drain 后三类最后融合时间均为 101116000 us，到末尾不再推进。
  PX4 EV aid 按新观测发布，停源后没有新 aid 消息是预期行为；复核检查持有的最后
  time_last_fuse 及 drain 后任何新消息，不能要求停源后继续发布 aid 才判停止。
- 末端双目/SDK 25 Hz、IMU 250 Hz（均按仿真时间），左右图像完全配对；末端 IMU
  最大间隔 4 ms。IMU 审计端保留 100 个槽，间隔/新鲜度判据不变。
- 端到端正常化源、QGC 连接、未解锁着地和自有进程组清理均通过。

实际 ULog 全文件扫描核对 14 个参数：EV_CTRL=11、EV_NOISE_MD=0、GPS_CTRL=0、
MAG_TYPE=5、OF_CTRL=0、RNG_CTRL=0、AGP_CTRL=0、DRAG_CTRL=0、HGT_REF=3、
BARO_CTRL=1、SENS_IMU_MODE=0、MULTI_IMU=1、MULTI_MAG=0、UXRCE_DDS_SYNCT=0。
实际 flags 同时核对 GNSS/磁/光流/测距/辅助全球位置/EV 速度关闭，气压高度辅助开启。
`effective-parameters.json` 为原记录，`ulog-replay.json` 为完整归档 ULog 的独立重放。
`px4.ulg.gz` 解压后 32638423 字节，其 SHA256 与 ulog-origin.json 相符。

## 原始失败与限制

| 诊断目录 | run_id | 原始结果与证据 |
| --- | --- | --- |
| diagnostic-callback-api | 2cbff4f8-a225-4bbd-8c25-711d102db184 | FAIL，误将回调字典当对象；启动过早另触发 CLOCK_STALLED，零 EV 输出 |
| diagnostic-cold-static | 38484607-dcbe-4a4e-a468-fa5dd7e03cd9 | FAIL，回调不提供 GID；SDK 首个协方差零，随后全程对角为 1，零合格标准位姿/EV |
| diagnostic-short-imu-gap | 9812b8a2-6ee2-43d3-86ca-dc8171304e90 | 整体 FAIL；实际融合子检查 PASS：1048 输出、各 1044 fused、READY 39.74 s；IMU 审计间隔 36 ms 超限 |
| diagnostic-realtime-freshness | 859ae6ef-fec7-4019-9f03-e20ed1f0a601 | FAIL，1.0 倍目标速度、90 s；1511 输出、READY 58.34 s 后源超时 |
| diagnostic-realtime-deferred | 29e445d1-685d-46d7-98ed-0d35e6c222bf | FAIL，1.0 倍目标速度、90 s；移走回调 JSON 转换后仍超时，392 输出、READY 13.54 s |

失败均保留原始 passed=false、manifest、SDK/标准位姿、融合时序和日志；不以修改
结果字段变成成功。旧诊断的 current-source-check=false 表示本轮后续修复改变了输入文件，
它们不是当前实现验收。两个成功用例的所有输入/构建/资产指纹均匹配当前文件。

1.0 倍超时轮在失效附近，相机/IMU/跟踪的源时间间隔没有缺失，但标准化节点观察到
跟踪采样年龄约 204/208 ms 并锁存；不能仅凭这些数据确定是 SDK、队列、调度还是渲染
造成延迟。降低目标仿真速度提供更多处理余量，该轮通过不证明已修复实时根因。
相机 25 Hz、IMU 250 Hz、4 ms 物理步长、源 0.2 s 时限和协方差门槛均未放宽。

新 layered 世界和 x500_vio_ref 机体没有 W0 飞行安全配置；该入口连续要求地面状态，
不发布 vehicle_command / trajectory_setpoint / offboard_control_mode，不进行解锁。
独立载台运动数据未融合到这架飞机；尚无同机运动融合或飞行故障落地验收。

## 可执行复核

```bash
./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene layered --real-time-factor .8 --duration 120
./scripts/sim.sh px4-vio-sensors --normalize --duration 35
.venv/bin/python scripts/assess_px4_real_vio_fusion.py docs/validation/simulation/2026-10-09-real-vio-fusion/actual-pose-paced
.venv/bin/python docs/validation/simulation/2026-10-09-real-vio-fusion/check_negative_assessment.py
```

独立检查器只依赖 Python/numpy，逐样本核对 SDK→标准协方差、标准→固定对齐及
保守协方差、ENU/FLU→NED/FRD、原始时间、DDS 逐样本一致、实际融合推进、local
健康、辅助状态和停更。它不是一次新仿真运行。
8 个内存证据破坏用例（假速度、旧时间、截断协方差、错误坐标/源、隐藏 GNSS、
继续融合、无实际融合）均被拒绝，见 negative-assessment.json；原档未改动。

相关 196 项测试通过，4 个任务/接口包构建通过，vendor 和 EGO 管理补丁检查通过。
测试使用完整原生环境，隔离 PX4 wrapper 不加载 SDK interfaces，不能单独收集运动审计测试。
原生环境仍提示可选 isaac_ros_image_proc 缺少 local_setup；实际 cuVSLAM 构建/运行已验证，
本轮未修复该可选包。SHA256SUMS 覆盖归档，日志/大 JSON 使用 gzip 保留原始字节。

下一步：确认实时延迟来源并补齐同机移动/转向融合验证；冻结新机体/世界安全区域、
显式位姿 FlightServer 配置后再做 BT 起飞→30 s 悬停→原生降落与失定位故障处置。
