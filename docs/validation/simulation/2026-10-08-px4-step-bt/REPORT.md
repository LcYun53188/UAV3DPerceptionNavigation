# PX4 逐步骤 BT 与可视化 SITL（2026-10-08）

将原整段异步任务包装细化为显式起飞、航点、悬停、返航、降落叶节点。父 ExecuteMission
只发一次；叶节点通过绑定父 UUID、实例、控制世代和步骤索引的 AdvanceFlightStep 服务
逐次授权，等待实际完成，不用服务 ACK 代替任务结果。接口见 [逐步骤说明](../../../PX4_STEP_BT.md)。

## 实现与范围

- 网关在步骤完成并满足已有稳定门限后清空子引用、进入 AWAIT_STEP、保持位置，等待
  下一索引授权；初始起飞授权缺失时不进入 Offboard/解锁。仍只有原 FlightServer 写 FMU。
- 请求缓存、旧实例/根/会话/世代拒绝、重复/跳步拒绝、暂停恢复保留同一 BT 叶并更新
  物理子 UUID。停止、健康/时钟/进展失效及原生降落阶段不能取得新的步骤授权。
- 根结果校验 ROS 状态、业务码、清理与非 mock。确认 LAND 结果后由最后一次无派发/
  无续租 tick 完成 Sequence，记录 BT_TREE_TERMINAL SUCCESS；提前成功不能退出 0。
- 原直接客户端和旧整段 XML 模式保持兼容。当前步骤是服务授权协议，尚非每步骤独立
  ROS Action；EGO/nvblox、VIO、未知障碍规划和跨进程恢复仍不在本批 PX4 范围内。

W0 使用锁定 PX4 v1.16.2、x500_7、domain 78 / 随机 Gazebo partition；非零 map/odom
对齐来自既有 W0 定义，控制使用 PX4 本地估计。Gazebo 真值只作独立运动审计。
原 algorithm lab/mapping 在 domain 68 保留运行和地图，没有重置它。

## UI 与运行

已实际打开原算法 Gazebo、RViz 和 QGC，并用 X11 窗口树验证三种窗口存在。
`--ui` 另显示本轮拥有的 PX4 Gazebo/QGC：QGC 与 SITL 连通、DDS/clock 冒烟通过，
随后执行任务并在落地解除武装后清理本轮进程。QGC 的网络地图瓦片下载告警不作为
飞行链路结果；连接证据以监督器日志审计为准。原算法 GUI/RViz 不随 PX4 回归关闭。

初始独立 QGC 占用 UDP 14550 时，监督器在启动任何 SITL 前拒绝运行；关闭本轮创建的
独立 QGC 后才运行监督器拥有的 QGC。没有关闭用户已有的其他程序或改动 QGC 配置。
本轮 UI 操作日志位于本机 `.cache/sim/ui/`，受管 PX4 UI 日志随各场景归档。

最终进程与 X11 窗口复核见 ui-inventory.json、ui-windows.txt：仅有算法 Gazebo 服务端
82197 / GUI 112661 和 RViz 112724；六轮 PX4 Gazebo 均已退出，没有废弃实例。
独立 QGC 已重新打开，此时 PX4 回归会话已结束。

## 最终版本验证

`pause-resume/`、`cancel/`、`runner-stall/` 为最终源码用例；源代码与二进制 SHA-256
在每轮开始记录，并在归档时核对。最终结果、物理指标及故障处置见下表和目录中的
flight-observation.json。各轮使用同一 W0 recipe，分别创建独立、已清理的 SITL 会话。

| 用例 | 实际根终态 | 最终落地解除武装 | 步骤接受 / 完成 |
| --- | --- | --- | --- |
| 完整任务中暂停恢复 | SUCCEEDED / LANDED_AND_DISARMED，清理确认 | 是 | 0–6 各一次 / 0–6 各一次 |
| NAVIGATE 中取消 | CANCELED / STOPPED_AND_HOLDING，清理确认 | 有界保持后是 | 0、1 / 仅 0 |
| NAVIGATE 中 SIGSTOP Runner | ABORTED / BT_PROGRESS_TIMEOUT，清理确认 | 有界保持后是 | 0、1 / 仅 0 |

完整暂停恢复用例：真值最大位移 4.1597 m，完成起飞、两航点、30 s 与 3 s 目标悬停、
返航和原生降落。起飞/两航点/返航的独立真值误差分别为 0.13068、0.07539、0.07932、
0.08478 m，均低于回归 0.3 m 门限。两次悬停观察段（含稳定窗口）32.004 s、5.023 s，
最大漂移 0.09859、0.08176 m。业务暂停保持约 3 s，真值漂移 0.06061 m，恢复更新
物理子 UUID；七个 BT 步骤没有重复授权。最终日志含 BT_TREE_TERMINAL SUCCESS。

取消用例：Runner halt 后等待实际停稳/保持交接才取得 CANCELED、cleanup_confirmed=true。
实际最大位移 2.2937 m；结果后保持约 29.985 s，1500 条真值样本最大漂移 0.09095 m；
随后原生降落并解除武装。根取消终态本身只证明停止保持，最终落地由后续物理状态确认。

Runner 停滞用例：实际 SIGSTOP 后约 0.509 s 进入制动，网关返回 ABORTED /
BT_PROGRESS_TIMEOUT、cleanup_confirmed=true。最大位移 2.4917 m，结果后保持
29.996 s，1502 条真值样本最大漂移 0.13794 m；随后原生降落并解除武装。未授权
后续步骤；被冻结 Runner 没有客户端终态，结果来自网关及独立物理审计。

## 拒绝现场与时钟启动修复

`excluded-admission-unknown/`：最初可视化运行被拒绝，根未接受、飞控输出 0、仍落地解除
武装。该轮尚无详细准入诊断，不能从结果确定拒绝原因。

`excluded-initial-clock/`：增加诊断后的另一轮记录 CLOCK_BUSY_OR_HEALTH，
clock_fault_latched=true，但拒绝时各源时间已新鲜（本地位置约 0.004 s）。代码审计和
确定性测试证明：旧 watchdog 会将首次 `/clock` 尚未发现时持续的 ROS time=0 当作
已观测时钟停滞，锁存后不能恢复。现场没有记录首次锁存瞬间，因此不能排除该轮曾有
真实短暂停钟；不把静态可复现路径冒充完整现场时序证明。

现改为首次非零时钟到来前等待发现，新鲜 PX4 状态门控仍禁止无时钟解锁；已观测时钟
停滞超过 0.5 s 或回跳仍锁止，新鲜数据恢复不会清除故障。新增确定性测试覆盖发现延迟、
真实观测后的停滞、回跳及锁存。未放宽 0.1 s 本地位置源年龄或其他健康门限。

`intermediate-full/` 为修复前/树最终 tick 完善前的一轮完整飞行成功，仅保留为中间版本
诊断证据，其 source_sha256 与最终版本不同，未计入最终源码验收。所有拒绝轮都未
解锁，无自动有效任务重试；后续每轮是显式创建的新 SITL 会话和根 UUID。

## 构建和测试

隔离 PX4 四包构建、算法侧接口/BT 增量构建均通过；最终 C++ Runner 重建通过。
飞行/桥/Runner Python 与真实 DDS 契约共 **151 passed**（21.85 s），XML 已归档。
两个 CTest 通过：24 个 algorithm 用例、13 个 Runner 用例（107.47 s）。
这些契约用例使用合成输入，不作为物理飞行证据；最终 Runner 的 LAND 终态消费改动
由后续 CTest 和最终 SITL 用例验证。停止与 native landing 仍采用既有 W0 策略。

新增契约覆盖步骤索引/类型、旧根/实例/会话/世代/owner、重复 ID/冲突、停止/健康/
时钟/进展失效拒绝、步骤完成等待、租约要求、逐步骤派发、授权拒绝后的清理、halt、
提前根成功拒绝。managed vendor/ego 补丁检查、Python 语法和 Git whitespace 检查通过。

归档只包含明确列出的结果、生成树、事件、真值/估计 trace、命令、诊断与运行日志；
不含 BT 授权私有 JSON、QGC 配置/缓存或 rootfs。SHA256SUMS.json 校验完整归档。

本批只完成 W0 的服务授权式逐步骤 BT；独立步骤 Actions、EGO/PX4 感知导航与更完整的
定位/坐标/故障矩阵继续开发，S2/S3/S4 不标记为整体验收完成。

实现提交：68670bd（网关/接口）、0171164（逐步骤 BT）、68bfe89（UI 与审计）。
