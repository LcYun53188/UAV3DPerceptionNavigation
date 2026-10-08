# algorithm MissionServer / 两航点 BT 验证（2026-10-08）

实现父 ExecuteMission、异步 ExecuteWaypoints BT、跨航点/暂停控制预约、检查点与
暂停恢复，以及实际 BT tick 驱动的进展租约。控制预约提交 `9d36e95`、
BT/MissionServer 提交 `5618704`、实时回归工具提交 `66033a3`。API 与复现见 [任务说明](../../../ALGORITHM_MISSIONS.md)。

## 范围与环境

受管 `lab/mapping`、无界面，ROS_DOMAIN_ID=68、GZ_PARTITION=uav_ego_lab。
采用 Gazebo 真值、identity map/odom、零重力速度模型、nvblox 在线建图与 EGO 规划。
本批不连接 PX4，未重新执行此前 PX4 飞行验证，不证明飞机动力学或 VIO 精度。

加载最终 Python 执行器前保存了地图快照，重启受管栈后完整等待 `sim.sh init` 退出码 0，
再顺序执行下列三轮。各轮复用同一地图，未在任务期间离线移动模型，不作为全新未知地图
性能测试。初始化与回归使用同一操作锁。每轮创建并清理其拥有的 C++ MissionServer，
结束后保留 Gazebo/map。速度发布者始终只有 `gazebo_trajectory_executor`。

三轮结果的源码、BT XML 与二进制 SHA-256 已逐项核对当前构建。世界 SHA-256 为
`343d0cef432f45b0bbd854af71104a8908ee534de5c2bcb96eeef3dfc34f0424`；
C++ 二进制 SHA-256 为 `0b414b70629d8043826c311efb50075b76093dd20b5ccae1c240eab2c5d97de5`。
原始 Gazebo 位姿独立于过滤里程计，用于位置、位移与几何净空验证。

## 最终版本实时结果

| 用例 | 根 ROS / 业务终态 | 实际最大位移 | 终态后 1 s 漂移 | 最小几何净空 |
| --- | --- | --- | --- | --- |
| 两航点暂停恢复 | SUCCEEDED / SUCCEEDED | 0.8420 m | 0 m | 0.9000 m |
| 暂停中取消 | CANCELED / CANCELED | 0.3035 m | 0 m | 0.9000 m |
| Runner SIGSTOP / SIGCONT | ABORTED / ABORTED，BT_PROGRESS_TIMEOUT | 0.5360 m | 0 m | 0.9000 m |

所有根/本轮子结果均 `cleanup_confirmed=true`、`mock=false`，各服务端退出码 0。

暂停恢复：实际移动至少 0.3 m 后请求暂停；根 Action 在 PAUSED 中保持活动，保留
父会话。2 s 观察窗口位置漂移 0 m，姿态变化 0.00000604 rad。暂停期间 RViz 目标、
旧取消和自主探索入口不能抢占；重复暂停 request_id 返回原决策。两次旧轨迹重放均产生
OBSOLETE_GOAL 拒绝。恢复后使用新子 UUID 和局部 token，最终两个航点
`(-2.2,0.6,1.2)`、`(-3.2,-0.6,1.2)` 误差分别 0.000670 m 和 0.000899 m。
真实 GetResult 读取到三个本轮子终态：一个 CANCELED、两个 SUCCEEDED。

暂停中取消：暂停服务接受后立即提交根取消，取消覆盖 PAUSING，不误进入 PAUSED。
子与根都返回实际 CANCELED；取消 ACK 未视作清理确认。服务 ACK 与根反馈采样并非
一一对应，因此该轮周期反馈未捕获短暂 PAUSING，服务响应与随后根/子结果保留在 trace。

Runner 停滞：SIGSTOP 本轮 C++ 进程后，后端独立锁存 BT_PROGRESS_TIMEOUT，
从最后收到有效进展到故障状态为 0.5618 s。等到实际子 ABORTED/停止确认后 SIGCONT；
根恢复仅执行清理，返回实际 ABORTED / BT_PROGRESS_TIMEOUT，不重发导航。
该测试有恢复后的根终态，不把 SIGSTOP 当作业务暂停，也不推断 SIGKILL 时可取得根结果。

每个本轮根/子结果前 0.6 s 都有 30 条过滤里程计样本。所有窗口的最大线速度
≤0.02146 m/s、角速度 ≤0.03267 rad/s，满足 0.05 m/s、0.1 rad/s 门限。
原始 DDS 观察可能含此前已终止目标的保留状态；只按本轮根反馈关联的子 UUID
计入结果。`trace_summary.json` 明确列出本轮 UUID、样本数量及停止窗口统计。

## 构建与契约验证

增量构建接口、执行器和 BT 共三个包通过（11.7 s），不重编 CUDA 包。
Python/ROS 契约与仿真脚本测试 **288 passed**（57.36 s）。
最终版本 `colcon test --build-base build_uav --packages-select uav_bt` 的两个 CTest
通过：13 个 algorithm MissionServer 用例和 9 个既有 PX4 wrapper 用例，XML 已归档。

新增契约测试运行真实 C++ 进程和 DDS，但导航/里程计为合成输入，覆盖父会话占用、
首 tick 授权、重放/旧世代拒绝、旧服务实例根请求拒绝、暂停/恢复/取消竞态、暂停上限、
Runner 停滞、后继根任务、矛盾/mock 子结果、进程正常退出与停止释放。
这些测试不作为 Gazebo 或 PX4 飞行证据。地图/clock 契约边界已有部分测试，尚非完整故障矩阵。

managed vendor/ego 补丁检查、源码语法与 Git whitespace 检查通过。现有环境仍提示
旧 `isaac_ros_image_proc/local_setup.bash` 缺失，最终构建/测试退出码均 0，未改该安装记录。
第一次 CTest 命令漏写 `--build-base build_uav`，未找到构建目录；纠正路径后的命令通过，
该环境调用错误不计为测试结果。

## 排除的响应通道故障与修复

中间版本曾在有效 SendGoal 响应阶段超时，C++ 日志为 `Failed to send goal response`；
客户端随后 `Timed out waiting for mission state`。该轮无根反馈、控制预约状态或实际运动，
不算暂停中取消验收。保留 `excluded-discovery-response/` 的失败结果、原始 trace 与日志；
其源码/二进制 hash 与最终版本不同。

最终回归先发送必定被拒绝的 DISCOVERY_PROBE，验证双向 SendGoal 响应，再读取
只读 coordinator_instance 并发一次有效根请求；不重试有效任务。服务端对未确认接受
锁定 ACCEPTANCE_UNCONFIRMED_RESTART_REQUIRED。根请求必须绑定当前随机进程实例，
重启后的旧实例请求在预约前拒绝。

## 证据与后续范围

每个通过目录包含结果、根检查点、服务端日志、完整 `trace.jsonl.gz`；暂停目录另含
暂停检查点。`launch.log.gz` 是受管会话日志快照，`commands.txt` 为复现命令，
`SHA256SUMS.json` 校验归档文件。更早成功的中间版本只留在 `.cache/sim/`，不作最终验收。

最终仿真保留 `ready=true`、HOLD，位置约 `(-3.0082,-0.3659,1.2000)`，自主探索关闭。
本批完成 S2 algorithm 任务/暂停基线，S2 未整体验收；后续仍需扩展地图/clock 故障矩阵、
逐步骤 PX4 飞行 BT，以及 S3 非 identity 坐标/定位重置适配，再推进飞行与地图融合。
