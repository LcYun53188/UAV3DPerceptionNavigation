# SDK 执行耗时、队列对照及同步器接口复现

2026-10-09，继续本机 PX4 v1.16.2 / Gazebo Harmonic / QGC / cuVSLAM 15。
本轮交付 SDK 内部已有执行耗时的被动采集、原始样本关联、图像队列深度对照入口，
以及原生缓冲容量范围校验。218 项针对性测试通过。四轮整体结果均为 FAIL，
最后一轮实际融合子检查 PASS、标准化审计 FAIL；不能宣布实时稳定性或 S6/VIO 悬停通过。
全程未解锁着地，无飞行控制发布者，未操作 USB 飞控。

## 新增实现

SDK 的 VisualSlamStatus 已提供 track_execution_time 和 node_callback_execution_time。
源码确认前者为 Odometry::Track 调用范围，后者为 UpdatePose 的墙钟耗时，包含 IMU
登记、数据转换、跟踪及发布前处理，不包括其上游图像订阅/双目同步/流排序。
本轮直接保存这两个字段，没有更改 SDK 计算或其锁定二进制。
assess_vio_timing.py 按原始采样时间关联左右图像、位姿及状态，输出 Track、回调总耗时、
回调中其余工作，以及「双目 DDS 发布→位姿 DDS 发布」减去完整回调耗时的非负下界。
这是回调之外耗时的下界，不能等同于精确队列等待或物理同步误差。
无效/负数/回调小于 Track 的状态不能成为分段时序证据；主机时钟跳变时不解释管线差值。
旧归档没有这两个字段时保持原输出，上一轮 205 文件归档重放仍通过。

SDK MessageStreamSequencer 的容量参数是 uint8_t；之前请求 imu_buffer_size=400
会隐式截断为 144。当前显式设置 144，保留此前实际容量，拒绝不在 1..255 内的值。
默认图像队列显式设置 DEFAULT / depth 10，保持原 SDK 默认行为；新增
--sdk-image-depth 1..10 只允许用于未解锁融合对照。参数被写入标定与 manifest，
源码/辅助模块摘要纳入运行输入；未改采样频率、200 ms 门限、协方差门限或 PX4 参数。

## 四轮原始结果

每轮 120 s 墙钟，默认 Intel Mesa 渲染、layered 场景；原始结果均保存，未改字段。

| 目录 / run_id | 目标速度 / 请求图像深度 | 整体 / 融合子检查 | 管线最大 / Track 最大 / 回调外下界最大 |
| --- | --- | --- | --- |
| realtime-default / 1c5004c6-9079-44c4-894a-8ee86cf0a775 | 1.0 / 默认 10 | FAIL / FAIL | 492.48 / 156.95 / 395.70 ms |
| realtime-depth-one / b105bddc-abd4-44f8-9a1a-af7a2801d80b | 1.0 / 1 | FAIL / FAIL | 140.89 / 128.87 / 34.96 ms |
| paced-default-failure / 8da31ba6-214c-4331-b3de-de7f85ddf5b2 | 0.8 / 10 | FAIL / FAIL | 312.41 / 190.97 / 265.82 ms |
| paced-depth-one / 7c2038b9-bbf3-4f9c-93f2-afeb6a919116 | 0.8 / 1 | FAIL / PASS | 111.71 / 109.77 / 46.16 ms |

第一轮先于容量与队列入口修改，current-source-check=false；后三轮全部运行输入匹配。
四轮主机时钟均稳定、被动记录完整，全部自有进程清理确认。每列是各自最大值，
不一定来自同一个样本；不可相减这些列来解释最慢样本。

默认实时轮最慢样本 48.20 s 的管线为 492.48 ms，同样本 Track 104.73 ms、
UpdatePose 104.80 ms，故至少约 387.67 ms 在此回调之外。配置图像深度 1 后，
单轮最大管线和回调外下界下降，但仍触发新鲜度失效；未证明完整实时资格。
这些顺序单轮对照支持积压假设，负载/调度非确定性仍可能影响结果，不是充分因果证明。

0.8 倍默认队列轮在 ROS 33.28 s 触发接收间隔失效，最长 READY 29.00 s。
它否定“此前某轮 0.8 倍成功即稳定”的推论；不能把前轮成功改写为失败，也不能只保留成功。
该轮完整 ULog 保存在 paced-default-failure/px4.ulg.gz，全文件扫描/重放的 14 个
PX4 参数均一致，未出现隐藏 EV 速度/GNSS/磁/光流等辅助；气压高度辅助按既有配置保留。

0.8 倍深度 1 轮有 2315 个实际输入及匹配 DDS 回读，位置/高度/航向各 2312 次融合，
连续 READY 114.060 s 墙钟；实际停源后 105.9 ms 返回 VIO_SOURCE_LOST，随后旧源锁存，
drain 后融合时间不再推进。融合子检查全部 PASS；独立总体复核仍返回 FAIL。
唯一标准化失败项 pose_window 要求末端 5 s 墙钟内至少 100 条位姿，该轮实测 99 条；
25 Hz 仿真采样在 0.8 倍速度约为 20 Hz 墙钟，处于判据边缘。
这揭示审计的源频率与墙钟窗口口径问题，尚未改该判据，原始 FAIL 保留。

两个 0.8 倍轮均保存只读 SDK 参数服务与订阅图记录 sdk-runtime-receipt.json，实际
image_qos_depth 分别 10 / 1，imu_buffer_size=144、image_buffer_size=100。
本机图元信息的 depth 均回报 0，不能用它确认订阅实际历史深度；参数服务回读与
已审阅的 QoS 构造代码提供配置证据，不宣称 DDS 内部队列占用已测得。

## 已复现的同步器单位缺陷

源码 VisualSlamImpl 构造器将 imu_jitter_threshold_ms_、image_jitter_threshold_ms_
直接传给 sequencer；后者比较的时间差来自 header.stamp.nanoseconds()，没有换算。
sequencer_probe.cpp 使用实际头文件、无 ROS/SDK 跟踪调用的固定序列，确认：

- 400 容量配置在原生类型转换后实际保留 144 条 IMU，和本轮显式配置一致。
- 前一帧后 40 ms 的图像到达时，最新 IMU 仅落后 4 ms。直接传 12/60 的构造器
  提前触发第二次图像处理；按 12/60 ms 转为纳秒时等待 IMU 到达当前帧时间再处理。

此处接口单位缺陷已确定，不是长时失效根因已确定。当前冻结 SDK 二进制仍使用旧逻辑；
本轮没有修改 SDK 或重新冻结 SDK 位姿契约。下一步用受管 vendor patch 修正换算、
补充同步器边界/乱序测试并重建，更新二进制契约后重跑 reset/运动/融合；同时纠正
pose_window 的时钟口径并保留本轮失败。之后才验证同机运动融合与新机体/场景飞行区域，
再进入 BT 起飞→30 s VIO 悬停→原生降落。未进行 OAK-D Pro W 硬件或 VIO 飞行验收。

## 可执行复核

```bash
./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene layered --sdk-image-depth 1 --duration 120
./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene layered --real-time-factor .8 --sdk-image-depth 1 --duration 120
.venv/bin/python docs/validation/simulation/2026-10-09-vio-sdk-timing/check_archive.py
.venv/bin/python scripts/assess_vio_timing.py docs/validation/simulation/2026-10-09-vio-sdk-timing/realtime-default
g++ -std=c++17 -I src/isaac_ros_visual_slam/isaac_ros_visual_slam/include -I src/isaac_ros_common/isaac_common/include docs/validation/simulation/2026-10-09-vio-sdk-timing/sequencer_probe.cpp -o /tmp/vio-sequencer-probe
/tmp/vio-sequencer-probe
(cd docs/validation/simulation/2026-10-09-vio-sdk-timing && sha256sum -c SHA256SUMS)
```

origin-files.json 保存每个原始文件字节数及解压 SHA256；日志/大 JSON 使用 gzip 原样保存。
check_archive.py 复核 136 个原始文件、四轮失败结果、时序/融合检查与完整 ULog 参数，
不把融合子检查 PASS 升格为整体 PASS。配套 SDK 源码证据及摘要见 source-review/。
