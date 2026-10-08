# 本机 PX4 逐步骤 BT

`./scripts/sim.sh px4-flight --bt --ui` 显示本轮拥有的 PX4 Gazebo 与 QGC 窗口，
执行完整 W0 起飞、两航点、悬停、返航和原生降落。已有 algorithm Gazebo/RViz 可继续
在 domain 68 / `uav_ego_lab` 中运行；PX4 使用独立 domain 78 和随机分区，不共享控制输出。
本轮 UI 在任务落地、解除武装并清理后关闭；不带 `--ui` 保留无界面回归入口。
启动前 UDP 14550 必须空闲；已有 QGC 占用时入口会拒绝，不接管其他进程。

## 任务与步骤的控制关系

受管 BT 客户端设置 `step_controlled=true`、`runner_progress_required=true` 和当前实例。
Runner 校验 recipe 后生成 `StartFlight → FlightStep[0..N-1] → AwaitFlightResult` Sequence，
节点按 TAKEOFF / NAVIGATE / HOVER / RETURN / LAND 和索引命名，实际 XML 保存为
`bt-flight-tree.xml`。根 ExecuteMission 只发一次，每个 FlightStep 异步授权一次，
等待真实步骤完成；完成原生降落、着陆解除武装和根结果后才报告树 SUCCESS。

每步骤采用 `/uav/px4/advance_step` 的 `AdvanceFlightStep` 服务：根 UUID、coordinator、
当前 ControlSession（UUID、generation、owner）、非零 request_id、step_index、step_type。
这是父 Action 内的步骤授权协议，**不是每步骤独立 ROS Action**。物理运动、控制流、
停稳、暂停恢复和降落仍由原唯一 FlightServer 网关执行；BT 不直接写 FMU。
不带 `--bt` 的对照客户端沿用原自动推进；旧整段 XML/不启用 step_controlled 的 Runner 保持兼容。

首次起飞授权仅在 PRESTREAM 接受，未授权不进入 Offboard/解锁；其后只能在 AWAIT_STEP
授权紧邻的下一索引，且类型必须与已校验 recipe 一致。运动/悬停达到现有实际稳定门限
后，网关撤销旧子 UUID/轨迹，保持当前位置并进入 AWAIT_STEP，不自动执行下一步。
暂停恢复重新生成同一步骤的物理子 UUID，不消耗下一步授权，也不重跑已完成步骤。

服务 ACK 只表示授权已记录。FlightStep 必须通过根 Action 的绑定反馈观察步骤完成，
LAND 必须等待实际根 SUCCEEDED、cleanup_confirmed=true、mock=false。提前或矛盾
根成功不能使树退出 0。若步骤授权被拒绝，Runner 取消父任务、等待真实清理并退出 1。

请求缓存每根最多 256 条；完全相同 ID/内容返回原决策，内容冲突拒绝，重复步骤或跳步
不能启动新运动。旧实例、旧根、旧控制世代/owner、失效健康、时钟或进展、停止/原生
降落中的新授权均拒绝。旧 ACK 可被幂等重读，但不重新授予或执行已退役步骤。

## 停止和失败

所有等待阶段仍实际 tick，并按已有根 UUID/实例/递增序号续进展。AWAIT_STEP 不提供
无限保活：根总预算和独立进展租约仍生效。Runner 停滞先制动、确认停稳和保持交接，
最多 30 s 有界保持后原生降落；已提交的原生降落不可取消，也不被 Runner 失联中断。
SIGTERM 的 halt 保留客户端等待清理，不把取消 ACK 当完成；结果后不再续进展或重起任务。
仅为树消费已确认 LAND 结果做最后一次无派发、无续租 tick，记录 `BT_TREE_TERMINAL SUCCESS`。

根请求拒绝会记录原因与各源接收/源时间年龄，不输出授权 nonce。首次拒绝不自动重试。
网关启动时尚未收到首个非零 `/clock` 属于发现等待；已观测时钟随后停滞超过 0.5 s
或回跳才锁存故障，不能靠恢复数据自动清除。首次时钟到来前仍不能通过新鲜 PX4 状态
门控，不会解锁或发布任务控制。
现有 0.1 s 本地定位源年龄门限保持不变；图形渲染负载不是放宽安全门限的理由。

## 验证范围

W0 是锁定的平面已知安全区域，导航采用 PX4 本地位置和经过校验的非零 map/odom 对齐；
Gazebo 真值只作独立审计。本接口尚未接 EGO、未知障碍地图、VIO 或跨进程检查点恢复。
因此不代表 S2–S4 全部验收完成，也不等价于实机起降。

验证记录见 [逐步骤 BT 与可视化 SITL](validation/simulation/2026-10-08-px4-step-bt/REPORT.md)。
