# VIO 新鲜度修复与时序对照

2026-10-09，本机 PX4 v1.16.2 / Gazebo Harmonic / QGC / cuVSLAM 15。
修复了接收器用旧观测年龄拒绝新观测的问题，新增被动时序记录和显式渲染对照。
当前实现的 0.8 倍目标速度、120 s 墙钟未解锁融合及默认 reset 回归通过。
1.0 倍 NVIDIA GLX/EGL 长时检查仍失败，实时稳定性及 S6/VIO 悬停尚未通过。
本轮未解锁、未发布飞行控制、未操作 USB 飞控；仿真标定不代表 OAK-D Pro W 标定。

## 修复与可解释的边界

原 `AlignedPoseStream.accept()` 先检查上一次接受样本的年龄，再检查当前输入。
因此旧样本超过 200 ms、但新样本仍在期限内时，可能锁存拒绝当前新样本。
现在接收路径按当前原始采样时间判断年龄；独立 watchdog 仍检查上次接受样本。
监督器先处理已精确配对的待接收数据，再运行 watchdog；所有源身份、reset、时钟、
接收间隔、协方差、地面状态与初始化对齐检查保留。已锁存失效的源不能被新样本恢复。
单测覆盖旧样本 205 ms、新样本 165 ms 的有效交接，以及先超时后不得恢复的负例。
修复后的 GLX 轮实际记录一次这种有效交接，但后来真正的源延迟仍导致失败。

`SourceTiming` 有界记录原始采样时间、ROS 时间、单调时间、主机时间与 DDS 发布元信息；
退出后写 JSON，不在接收回调执行文件序列化。默认不融合的入口不启用时序记录。
各融合轮主机时钟偏移稳定，记录无环形覆盖。按相同原始采样时间匹配左右图像及
SDK 位姿的 DDS 发布时间，可分别测量「双目已发布→SDK 位姿发布」与「发布→回调」。
这不是物理曝光/IMU 同步测量，也不能拆分 SDK 内部同步、跟踪、队列与 GPU 调度。

渲染选项仅作用于本轮拥有的 Gazebo 进程。`--render-device nvidia` 设置 PRIME/GLX
环境并要求新鲜 Ogre 日志确认为 NVIDIA；`--headless-rendering` 使用 Gazebo 原生 EGL
离屏入口。不改系统驱动配置。二者是诊断选项，尚未验证为延迟问题的解决方案。

## 原始用例与当前实现

每轮保存原始 result/manifest、SDK 与归一化源、融合输入/回读、EKF 时序、日志和资产。
`summary.json` 汇总原始结果；失败不重写为成功。三轮早期诊断输入已被后续修复改变，
`current-source-check=false` 明确表示它们不能作为当前版本验收。

| 目录 | 原始 run_id | 结果 / 当前输入匹配 | 双目→SDK 发布最大延迟 |
| --- | --- | --- | --- |
| realtime-default-before-fix | 5b553ec4-ca06-49fd-ab37-d264e67981ea | PASS / 否 | 108.6 ms |
| realtime-nvidia-before-fix | d94f1df8-c93c-4037-82a8-e9da18794900 | FAIL / 否 | 151.3 ms |
| realtime-nvidia-after-fix | 2bc4a9d1-fda8-4b1c-977a-bfd4be93bb97 | FAIL / 否 | 474.0 ms |
| realtime-nvidia-egl | c18bfe26-100f-4efb-92d1-b2913986fe7d | FAIL / 是 | 378.0 ms |
| paced-current | 1f99777f-7d15-4803-8d6c-5aaa6e1317c6 | PASS / 是 | 127.8 ms |
| reset-covariance-failure | cb0bb7c0-5e86-4a54-81d2-c3bbf941b558 | FAIL / 是 | 未启用时序 |
| reset-current | 2d06b8aa-ab06-44d1-9803-81606d360db5 | PASS / 是 | 未启用时序 |

前四轮均为 1.0 倍目标速度、120 s 墙钟。默认渲染早期单轮成功不构成实时资格。
该轮没有保存启动时 Ogre 日志，不将观察到的 Intel 渲染作为归档驱动确认。
其余融合轮保存 renderer-info.json 和对应原始 renderer.log；paced-current 确认为
Intel Mesa，NVIDIA 轮确认为 NVIDIA GeForce RTX 4070 Laptop GPU。

修复后 GLX 轮的 watchdog 在 ROS 90.968 s 检测最后样本 90.76 s，年龄 208 ms；
下一条 90.80 s 样本在 watchdog 后约 21.2 ms 到达，不能恢复已失效源。
EGL 轮 watchdog 在 ROS 73.688 s 检测最后样本 73.48 s，年龄 208 ms；
约 20 ms 后标准化源也锁存失效。该轮位姿 DDS 发布→标准化回调最大约 0.93 ms，
tracking 回调耗时最大约 1.81 ms，而双目发布→SDK 位姿发布最大 378 ms。
证据确认存在 SDK 发布之前的上游延迟；尚不能确定 SDK 内部具体根因。
相机 25 Hz、IMU 250 Hz、物理步长 4 ms、采样年龄 200 ms 与协方差门限均不变。

## 当前通过的回归

- paced-current：2329 条原始 VIO 输入、2329 条匹配的 DDS 回读；位置/高度/航向
  各 2326 个 fused 样本，EV 速度无融合。连续 READY 114.260 s 墙钟。
  输入采样年龄最大 116 ms；双目→SDK 发布 P50/P95/P99 为 28.38/60.56/88.56 ms。
- 实际 SIGINT 停止标准化源后 106.6 ms，门控返回 VIO_SOURCE_LOST；源端点消失
  锁存 VIO_WRITER_COUNT。随后 PX4 本地 reset 令门控终态 VIO_EKF_LOCAL_RESET。
  1 s drain 后三类最后融合时间均为 101160000 us，观察末端没有继续推进。
- 独立逐样本复核归一化、原始时间、固定对齐、保守协方差、坐标转换、回读、EKF
  三类实际融合和 local 健康均通过。全部飞行控制发布者为零，始终未解锁着地。
- 完整 ULog 解压后 32425630 字节，SHA256 见 ulog-origin.json。全文件扫描核对
  14 个参数并独立重放：EV_CTRL=11，EV 速度/GNSS/磁/光流/测距/辅助位置/拖曳
  融合关闭，气压高度辅助保留；UXRCE_DDS_SYNCT=0。记录见 effective-parameters.json。
- reset-current：默认 planar、40 s、586 条归一化位姿；SDK reset 成功，旧源撤销、
  drain 后无新标准化位姿，SDK 原始输出恢复。全部 FMU 输入及飞行控制发布者为零。
  相同配置上一轮在 reset 请求前触发 VIO_UNCERTAINTY_INVALID，因此未执行 reset，
  原始失败保留；复跑通过不证明平面场景长期协方差问题已修复。
- 7 轮自有进程清理均确认完成。四个 ROS 包构建通过（2.10 s）；203 项针对性测试
  通过，随后新增压缩归档读取测试，时序检查器 3 项测试通过（合计覆盖 204 项）。

## 复现与复核

```bash
./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene layered --real-time-factor .8 --duration 120
./scripts/sim.sh px4-vio-sensors --normalize --reset-source --duration 40
# 显式实时 EGL 对照；本轮此用例失败，不是稳定运行推荐配置
./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene layered --render-device nvidia --headless-rendering --duration 120
.venv/bin/python docs/validation/simulation/2026-10-09-vio-timing/check_archive.py
.venv/bin/python scripts/assess_vio_timing.py docs/validation/simulation/2026-10-09-vio-timing/realtime-nvidia-egl
(cd docs/validation/simulation/2026-10-09-vio-timing && sha256sum -c SHA256SUMS)
```

大文件 gzip 保存原始字节，origin-files.json 提供解压后的原始摘要与大小。
check_archive.py 复核文件字节、原始成功/失败、时序与融合检查结果及完整 ULog 参数。

下一步对 SDK 输入同步/跟踪路径做分段剖析，再验证同机运动融合。之后冻结带传感器
机体和新场景的独立飞行区域、显式位姿飞行配置，接入 BT 起飞→30 s 悬停→原生降落
及定位故障处置。现有 W0 世界/机体安全摘要不能授权新场景；当前入口仍连续要求地面
状态，尚未验证 VIO 飞行、导航避障或 OAK-D Pro W 硬件闭环。
