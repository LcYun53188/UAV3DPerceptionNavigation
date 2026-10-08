# algorithm 地图/时钟故障与恢复回归（2026-10-08）

在已完成的 [MissionServer/两航点 BT](../2026-10-08-algorithm-mission/REPORT.md) 上补故障验收。
新增 11 个真实 DDS/C++ 契约用例和两个 Gazebo 故障场景，再执行故障后的新根两航点回归。
本批保留既有执行器与 MissionServer 运行逻辑，扩大测试覆盖并增加可复现故障注入入口。
契约用例提交 `9348385`，实景注入工具提交 `42c918c`。

## 环境和证据边界

受管 lab/mapping、ROS_DOMAIN_ID=68、GZ_PARTITION=uav_ego_lab、无界面。
复用此前已初始化的在线地图和 Gazebo 会话 PID 82166；未重置世界、未在任务中离线移位。
算法栈仍为真值定位、identity map/odom、零重力速度模型，不连接 PX4，不证明 VIO、
PX4 飞行动力学或定位重置适配。`/cmd_vel` 始终只有原 Executor 发布。

回归脚本持有受管操作锁，逐轮拥有并清理 C++ MissionServer。时钟故障通过真实
`/world/uav_ego_lab/control` Gazebo transport 服务暂停/恢复世界，无额外 `/clock` 发布者。
地图故障通过 PAUSED 状态的真实 `/uav/map/save` 服务推进 epoch，同时保留地图包。
三个通过目录均核对脚本/执行器/BT 源码和已运行二进制 hash 与最终版本一致。

## Gazebo 结果

| 场景 | 根结果 | 实际最大位移 | 终态后 1 s 位置漂移 | 净空 |
| --- | --- | --- | --- | --- |
| clock-stall | ABORTED / ODOMETRY_OR_CLOCK_FAULT | 0.3860 m | 约 1.8e-16 m | ≥0.9000 m |
| paused-map-change | ABORTED / MAP_SESSION_CHANGED | 0.3045 m | 0 m | ≥0.9000 m |
| 故障后的新根两航点暂停恢复 | SUCCEEDED / WAYPOINTS_COMPLETE | 0.9941 m | 0 m | ≥0.9000 m |

所有根结果均 `mock=false`、`cleanup_confirmed=true`，每轮拥有的服务端正常退出。

时钟暂停持续约 1.356 s（两次 transport 响应接收时间之差）。暂停 ACK 后约 0.391 s
观察到后端故障，不能据此推断从世界实际暂停瞬间计算的检测延迟。故障后、恢复世界前
没有新的原始真值样本，观察到 30 条全零速度命令。该窗口根 Action **未结束**，没有用
冻结里程计冒充停稳确认。恢复世界和新鲜里程计后，实际子和根都返回 ABORTED，
均确认停止清理；没有恢复旧导航。子结果前 0.6 s 的 30 条里程计样本最大线速度
0.02173 m/s、角速度 0.02884 rad/s。

地图变化先等待实际子 CANCELED 和 PAUSED，再调用地图保存；map_id 保持不变，epoch
从 `1791432153284311087` 增至 `1791432153284311088`。子保留 CANCELED 终态，
父任务因 MAP_SESSION_CHANGED 返回 ABORTED，没有从检查点续跑。地图包位于本地
`.cache/sim/s2-fault-paused-map-change/map-bundle`，归档仅含 manifest/hash，未提交大体积地图。

故障处理后，另起服务实例、读取当前地图/实例并显式提交新的根 UUID，两航点和暂停恢复
通过。两个目标误差分别 0.000330 m、0.000784 m，暂停位置漂移 0 m，旧曲线重放两次
被拒绝。本轮关联三个子结果：一个 CANCELED、两个 SUCCEEDED。此实景用例证明新实例
可恢复工作；同一实例在已确认清理后接受新根/新世代的行为由下面的契约用例验证。

所有本轮根/子结果观察前 0.6 s 的样本窗均有 30 条里程计，线速度最大
0.02581 m/s、角速度最大 0.03267 rad/s，满足停止门限。原始 DDS 可能包含此前已结束
目标；统计只纳入本轮根反馈关联的子 UUID，详见 `trace_summary.json`。

## 契约覆盖和限制

新增 10 个参数化用例：在 RUNNING 和 PAUSED 两阶段分别注入地图 epoch 变化、
健康失效、源里程计时间戳冻结、接收停更、源时间戳回退。真实 C++ MissionServer
通过 DDS 调用 Python NavigationAction，但导航/里程计是合成输入。这些用例不属于飞行证据，
源时间戳回退也不等于整栈 `/clock` 回跳。

各用例验证父预约锁存故障、撤销子导航、旧会话失去授权及后继子 UUID 更新；输入恢复后根仍为
ABORTED，清理确认后只有显式的新根、新世代和当前地图才能完成后续两航点。
故障刚发生但状态尚未通过 DDS 到达 MissionServer 时，恢复服务可能依据上一条新鲜状态
返回 RESUMING ACK；ACK 不授权运动，执行器原子校验仍必须拒绝故障会话，旧目标不得派发。
测试按实际终态和派发计数验证该边界，没有把服务响应视作已恢复。

第 11 个新增用例保持里程计接收缺失直到停止确认期限：子无法证明停稳，根返回
ABORTED / CHILD_STOP_UNCONFIRMED、`cleanup_confirmed=false`，执行器保留父会话及
故障锁止，MissionServer 拒绝后继根。新鲜数据恢复不会自动解除锁止。该未确认路径只在
合成后端验证；真实 Gazebo 停更本轮在期限前恢复，不能宣称实景持续失联验收已通过。

## 构建、检查与归档

增量构建三个包通过（1.82 s），运行源码不变；新增契约用例使用单调时钟驱动合成后端，
扩大 suite 后将 algorithm CTest 总超时从 120 s 调整为 180 s，不改变任务运行预算。
全量 **299 passed**（112.50 s）；`pytest.xml` 与输出已归档。两个 CTest 全部通过（101.76 s）：24 个 algorithm 用例、9 个既有 PX4 wrapper 用例；
CTest 结果见归档 XML/日志。
managed vendor/ego 补丁检查、Git whitespace 与 Python 语法检查通过，未改第三方 gitlink。

每个场景含结果、检查点、服务端日志和完整压缩 trace；另含受管 launch 日志快照、
最终状态、命令与 SHA-256 清单。既有 `isaac_ros_image_proc/local_setup.bash` 缺失提示仍存在，
本轮构建和最终测试退出码均为 0。

最终保持 Gazebo 运行，地图有效、`ready=true`、HOLD，自主探索关闭；测试拥有的
MissionServer 均已退出。最终状态见 `final-status.txt`。

下一步仍需逐步骤 PX4 飞行 BT、S3 非 identity 坐标/定位 reset、更多地图/clock 故障矩阵。
本批不将 S2/S3/S4 或整个仿真开发计划标为全部完成。
