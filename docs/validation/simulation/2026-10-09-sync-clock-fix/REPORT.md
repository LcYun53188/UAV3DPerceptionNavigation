# SDK 同步器单位与位姿审计时钟修复

2026-10-09，本机 PX4 v1.16.2 / Gazebo Harmonic / QGC / cuVSLAM 15。
已修复同步器毫秒/纳秒换算和减速审计的源频率口径，SDK 重建与 222 项测试通过。
实际 reset、120 s 独立载台运动及 50 s 未解锁融合回归通过；两个 120 s 融合轮仍失败。
长时稳定性、同机运动融合及 S6/VIO 悬停尚未验收。全程未解锁、无飞行控制发布者，
未操作 USB 飞控；本报告不代表 OAK-D Pro W 标定或硬件闭环通过。

## 实现与契约

SDK 构造器将 imu_jitter_threshold_ms_ / image_jitter_threshold_ms_ 转换为 int64_t
纳秒后传给 MessageStreamSequencer，其比较对象为 header.stamp.nanoseconds()。
测试直接提取实际构造器参数并编译原 SDK 头文件，验证 IMU 落后 4 ms 或恰好 12 ms
时等待，超过 12 ms 时按既有超时分支处理；此前最小复现的错误单位路径会提前处理。
没有改变 12/60 ms 配置值、帧率、200 ms 新鲜度或协方差门限。

补丁保存于 patches/vendor/isaac_ros_visual_slam.patch，sources.json 保留上游
04bf49a2daf7710d2ba2390d1772435a1baeb48d，不创建本地 submodule 提交。
受管补丁、锁文件与二进制契约更新后，SDK 构建通过（15.7 s），独立 PX4 VIO 遥测
构建也重新验证；锁定的 W0 PX4/Agent 构建保留。变化只有包装节点库与构造器源码，
SDK 主程序、libcuvslam.so 和协方差转换源码的摘要未变；pose_contract.json 冻结
新的实际指纹。保留 cuVSLAM15_right_tangent_base_link_v1 位姿数学契约。

PoseAudit 原先在末端 5 s 墙钟窗口要求 100 条位姿；0.8 倍速下，25 Hz 仿真源
仅约 20 Hz 墙钟，可能以 99 条误判失败。现在使用原始采样时间/ROS 时间的 5 s
窗口，仍要求 100 条，源身份、原始时间、协方差与位姿一致性全部保留。
reset 请求时冻结 ROS 窗口终点；单调时间继续约束请求前样本、连续有效状态、
停源与 reset 后 drain。测试覆盖两种速度的相同采样证据，以及 reset 后收到的
同时间戳样本不能进入请求前窗口，过旧/未来样本不能成为频率证据。
result.normalized_pose.pose_window 显式保存窗口时钟、终点、条数与门槛。
这是审计口径修复，旧失败记录不修改，也不降低源的实时新鲜度要求。

## 实际用例

所有 case 的运行输入均匹配本轮最终代码/构建，全部自有进程清理确认。
融合轮均为 layered、默认 Intel Mesa 渲染、SDK 图像深度 1。

| 目录 | 原始 run_id | 配置 / 原始整体结果 |
| --- | --- | --- |
| realtime-depth-one | 1f8458b6-5a66-4fe4-be4a-06ea15b00696 | 1.0 倍目标速度，120 s：FAIL |
| paced-current | 301b2b34-c190-4dad-bc7d-d69391268ac2 | 0.8 倍目标速度，120 s：FAIL |
| reset-current | 5028ba23-880c-48af-895f-d5377a41e6f6 | 默认 planar，实际 reset，40 s：PASS |
| motion-current | dcb25ccf-01ca-49cb-912e-2d1799941aac | 独立载台三轴/航向运动，layered，120 s：PASS |
| short-paced | 85bde9d2-d481-45a6-8912-e68566206821 | 0.8 倍目标速度，50 s：PASS |

### 通过的回归

reset-current 有 585 条标准化位姿；reset 请求前源窗口 124 条。SDK reset 成功，
旧源撤销、drain 后无新标准化位姿，SDK 原始输出恢复。所有 FMU 输入为零。

motion-current 仅使用实际 Gazebo 动力学载台的双目/IMU，不输出 EV 或飞行控制。
2737 条标准化位姿与真值按原始时间配对，首次对齐后固定不再拟合；位置 RMSE
0.013554 m、最大 0.024954 m，最大姿态误差 0.4556°。三轴峰峰值约
1.0/0.8/0.6 m，航向约 0.8 rad；源覆盖末端、连续性、IMU 响应、协方差和正常化
均通过，末端源窗口 124 条。独立纯 Python 重放确认误差及报告一致，未记录到源撤销。
此轮是参考载台 VIO 测试，不是将载台定位融合到另一架 x500，也不是飞行闭环。

short-paced 有 929 条实际 VIO→PX4 输入和 929 条匹配 DDS 回读，位置/高度/航向
各 927 个 fused 样本，EV 速度无融合；连续 READY 44.200 s 墙钟，末端源窗口
124 条。实际 SIGINT 停源后 63.1 ms 返回 VIO_SOURCE_LOST，旧源锁存；1 s drain
后三类最后融合时间均为 45240000 us，直到观察末尾不再推进。独立逐样本检查
覆盖实际 SDK/标准化/固定对齐/保守协方差/坐标转换、回显和 EKF 融合，全部通过。
这是明确较短的功能回归，不替代两个 120 s 长时失败，不把较短时长作为资格放宽。

### 保留的失败

realtime-depth-one 最长连续 READY 42.100 s 后融合门控已失效，原始整体 FAIL。
SDK Track 最大约 243.8 ms，双目发布→位姿发布最大约 255.7 ms。
随后标准化源在 ROS 125.224 s、采样年龄 224 ms 锁存 VIO_TRACKING_INVALID；
末端原始 odometry 间隔检查也失败。单位修复没有消除跟踪耗时尖峰。

paced-current 最长连续 READY 86.080 s，1761 条输入；在 ROS 78.844 s 检测最后
样本 78.64 s，年龄 204 ms，门控锁存 VIO_SAMPLE_STALE。约 12.24 s 墙钟后，
标准化源在 ROS 88.632 s 触发 VIO_UNCERTAINTY_INVALID（采样年龄仅 72 ms）。
该轮管线最大约 158.5 ms、Track 最大约 156.1 ms。必须分别处理缺样/延迟及 SDK
协方差退化，不能把协方差失效归因于新鲜度，也不能用放宽阈值消除失败。

paced-current 的完整 ULog 解压为 32462728 字节，原始 SHA256 见 ulog-origin.json；
全文件扫描及独立重放核对 14 个 PX4 参数一致，排除偷偷启用 EV 速度、GNSS、磁、
光流、测距、辅助位置等；气压高度辅助按原配置保留。此轮失败不等同于参数检查失败。

## 验证与后续

222 项针对性测试通过，包括实际构造器编译边界测试、源窗口测试以及现有帧/时间/
身份/协方差/融合/停源负例。受管 vendor 与 EGO 检查通过，SDK 改动只来自受管补丁。
归档保存原始 result/manifest、标定/SDK 参数、资产、源与融合时序、真值、IMU、日志。
大 JSON 和原始日志 gzip 保存，origin-files.json 记录解压后字节数与 SHA256。
check_archive.py 重放原始成功/失败、时序、融合、运动误差及完整 ULog 参数；
运动误差跨 numpy 环境只允许 1e-12 的舍入差，位姿/阈值不改。

```bash
./scripts/build_vio_node.sh
./scripts/build_px4_vio.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q scripts/test_vio_sequencer.py scripts/test_vio_pose_window.py
./scripts/sim.sh px4-vio-sensors --normalize --reset-source --duration 40
./scripts/sim.sh px4-vio-sensors --normalize --motion --scene layered --duration 120
./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene layered --real-time-factor .8 --sdk-image-depth 1 --duration 50
# 保留正式长时对照；本轮两种速度均未通过
./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene layered --real-time-factor .8 --sdk-image-depth 1 --duration 120
.venv/bin/python docs/validation/simulation/2026-10-09-sync-clock-fix/check_archive.py
(cd docs/validation/simulation/2026-10-09-sync-clock-fix && sha256sum -c SHA256SUMS)
```

下一步以原始样本定位门控缺样/交接和 Track 耗时尖峰，分别复现静止协方差退化；
保留全部失败和门限，再验证长时同机融合与同机运动。带传感器机体/新场景的飞行
安全区域、显式位姿飞行配置与 BT 起飞→30 s 悬停→原生降落仍待独立实现和验收。
