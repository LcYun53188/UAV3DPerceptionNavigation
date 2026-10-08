# algorithm 导航 Action 验证（2026-10-08）

完成 EGO `NavigateToPose3D` 服务端及普通 Action 客户端回归。接口、边界与复现说明见
[导航 Action](../../../NAVIGATION_ACTION.md)。核心代码 `4983a9a`，构建/回归工具 `caff7f9`。

## 配置与范围

- 受管 `lab/mapping`，ROS_DOMAIN_ID=68，GZ_PARTITION=uav_ego_lab；无界面。
- Gazebo 原始真值定位、identity map/odom、零重力速度模型，nvblox 在线地图、EGO 规划。
- 初始化使用离线相机放置，回归开始前已退出并回到 `(-3,0,1.2)`；初始化不计自主运动。
  最终用例复用同一建图会话，包括此前诊断观测，不是全新未知地图性能测试。
- `/cmd_vel` 恰有一个发布者 `/gazebo_trajectory_executor`。
- 目标容差 0.2 m，停稳速度 ≤0.05 m/s、角速度 ≤0.1 rad/s，连续窗口 0.6 s。
- 运行前保存源码 hash；两个通过用例的 hash 与提交内容逐项核对一致。场景 hash 固定，
  原始 Gazebo 位姿独立于过滤里程计，用于验证实际位移、终点和几何净空。

这是普通 Action 客户端验证。algorithm 父 ExecuteMission、跨子任务授权、两航点 BT、
暂停检查点/恢复及 BT 进展租约尚待实现；S2 未整体验收。该速度模型结果不证明 PX4
动力学、VIO 或真实飞机精度。本轮未重新执行此前 PX4/BT 飞行测试。

## 最终通过用例

| 用例 | ROS / 业务终态 | 最大实际位移 | 最终目标误差 | 结果后 1 s 位置漂移 | 最小几何净空 |
| --- | --- | --- | --- | --- | --- |
| 航点 1 `(-2.2,0.6,1.2)` | SUCCEEDED / SUCCEEDED | 1.0016 m | 0.001370 m | 0 m | 两段合计 0.9000 m |
| 航点 2 `(-3.2,-0.6,1.2)` | SUCCEEDED / SUCCEEDED | 1.5644 m | 0.000911 m | 0 m | 同上 |
| 运动中按 UUID 取消 | CANCELED / CANCELED | 0.3556 m | 不作为到达用例 | 0 m | 0.9000 m |

以上结果均 `cleanup_confirmed=true`、`mock=false`。两航点使用不同 Action UUID，
共收到 125 条反馈；取消用例收到 35 条反馈。取消请求在原始位移 ≥0.35 m 后发送，
请求至实际终态为 0.6846 s，取消 ACK 没有被当作任务完成。

每个结果前 0.6 s 均有 30 条过滤里程计样本。航点 1/2 的该窗口最大线速度分别为
0.000943/0.000611 m/s；取消窗口最大线速度 0.02120 m/s、角速度 0.03267 rad/s，
满足停止门限。原始样本与统计见 `trace_summary.json` 和各用例 `trace.jsonl.gz`。

## 构建与契约测试

增量构建接口和执行器两个包通过（约 1.16 s），不重建 CUDA 感知包。
导航及仿真脚本回归共 **266 passed**；其中 15 个真实 DDS Action 用例使用合成导航/
里程计，验证成功、取消 ACK/终态分离、繁忙拒绝、旧 UUID 隔离、最终/局部目标区分、
地图/健康/规划/超时故障、冻结时间戳、终点漂移、旋转不算停稳，以及清理期间地图变更。
这些契约用例不作为飞行证据。新增 4 个 GoalManager 测试覆盖旧目标、取消和探索启停互斥。

源码编译、脚本语法、Git whitespace 检查与 managed vendor/ego 补丁检查通过。
`with_venv.sh` 加载已有安装环境时仍会提示旧 `isaac_ros_image_proc/local_setup.bash` 缺失；
本轮构建/测试退出码均为 0，未修改该既有安装记录。

## 排除的初始化重叠运行与修复

一次回归在 `sim.sh init` 尚未退出时提前启动，初始化放置与导航重叠，深度门控拒绝
恢复，导致 MapSnapshot 失效。服务端撤销目标后返回 ABORTED / MAP_INVALID /
`cleanup_confirmed=true`。该轮含离线放置位移，**不计入自主导航或成功验收**；保留在
`invalid-init-overlap/`，不能把其中位移当作执行器自主运动。

回归入口现持有与 `sim.sh init/survey` 共用的 `operation.lock`。初始化持锁时再次调用
回归，实测退出码 1，提示“已有仿真或控制操作正在运行”，未创建回归输出目录、未提交
Action。之后等待初始化退出码 0，再顺序执行上述两个最终通过用例。

## 证据与后续

`two-waypoints/`、`cancel/` 各含结果 JSON 与完整压缩 trace；`launch.log.gz` 为此次
受管会话日志快照，`SHA256SUMS.json` 校验归档数据。`commands.txt` 记录验证命令。
更早的中间版本重复运行仅保留在 `.cache/sim/`，不作为最终源码验收。

受管仿真保留运行，最终位姿约 `(-2.9740,-0.3266,1.2000)`，取消后保持，便于继续开发。
下一批实现 algorithm MissionServer、父会话持续占用、两航点 BT 和暂停检查点/恢复，
在此基础上补父任务 UUID 与剩余预算的真实回归。
