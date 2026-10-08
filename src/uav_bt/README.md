# uav_bt：PX4 与 algorithm 任务树

使用 BehaviorTree.CPP 4.9 与 rclcpp_action，调用真实 PX4 ExecuteMission 后端。
`--bt` 现在生成逐步骤树，通过绑定根/控制世代的服务依次授权起飞、航点、悬停、返航和降落；
网关仍拥有唯一 PX4 输出与物理停止。接口、UI 和边界见 [逐步骤 BT](../../docs/PX4_STEP_BT.md)。
旧 `trees/px4_flight.xml` 整段异步节点保留作兼容模式，独立逐步骤 ROS Actions 尚待开发。
algorithm 另有 `algorithm_mission_server`，
使用 `algorithm_waypoints.xml` 的 ExecuteWaypoints 异步节点调用 EGO Navigate Action，
实现父会话、两航点、暂停检查点/恢复及进展租约；运行和边界见
[算法任务说明](../../docs/ALGORITHM_MISSIONS.md)。两种后端使用独立入口，不能混开控制。

```bash
./scripts/build_px4_flight.sh
./scripts/sim.sh px4-flight --bt --ui
./scripts/sim.sh px4-flight --bt --flight-scenario pause-resume
./scripts/sim.sh px4-flight --bt --flight-scenario cancel
./scripts/sim.sh px4-flight --bt --flight-scenario runner-exit
./scripts/sim.sh px4-flight --bt --flight-scenario runner-stall
```

后两个场景分别 SIGKILL / SIGSTOP 本次拥有的 Runner，验证任务进展失效后的
制动、保持和原生降落。`cancel` 在运动中向 Runner 发 SIGTERM，验证 halt 和异步清理。
原有不带 `--bt` 的直接客户端入口保留作对照。

## 生命周期

Runner 单线程持续处理 Action 通信，每约 100 ms tick 一次树。只派发一次根 Action，
只有 tick 返回 RUNNING 且根 UUID 已确认时才发布 `/uav/px4/mission_progress`。
进展包含 Action 根 UUID、coordinator instance 和严格递增的 tick_sequence；
后端拒绝旧实例、旧任务、重放计数及已失效任务的续租。

首次 Action 接受与进展握手最多允许 5 s，期间保持地面预流，未收到有效进展绝不解锁。
握手后 0.5 s 没有有效 tick 即锁存 BT_PROGRESS_TIMEOUT：状态/控制有效时制动并确认
停稳，交接有期限的 HoldController 后返回 ABORTED、cleanup_confirmed=true，
保持最多 30 s 后原生降落。状态或控制已丢失时执行既有故障策略。
已提交的原生降落不受进展失效打断；最终保持期限不靠 Runner 滑动续期。

SIGINT/SIGTERM 在可取消阶段触发树 halt，并保留 Action 客户端直到收到实际终态。
接受响应晚于 halt 时，客户端在获知 UUID 后补发取消。取消响应只是请求确认，
不能作为已停稳的结果。若原生降落已开始，退出请求继续等待着陆；取消竞态导致
请求被拒绝时也继续观察根结果。超时未收到结果只报告 CLEANUP_UNCONFIRMED。
根任务结束后停止 tick；不会因为周期运行而再次起飞。

结果必须同时匹配 ROS Action 终态、业务 result_code、cleanup_confirmed 和 mock=false。
进程退出码：0 成功，130 已确认取消，1 拒绝/失败/未确认清理。SIGKILL 没有客户端
终态日志，由后端独立结果和物理状态证明处置。

## 构建与测试

构建入口隔离在 `.deps/mission-build` / `.deps/mission-install`，仅构建接口、桥、任务和
BT 四个包，不重建 CUDA 感知栈。Runner 参数由受管客户端生成的 0600 临时 JSON 文件
提供，授权 nonce 不放入命令行或归档；结束后删除文件。

```bash
./scripts/with_px4_sim.sh bash -e -c 'source .deps/mission-install/local_setup.bash; PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q src/uav_bt/test src/uav_mission/test'
```

ROS 通信契约测试使用独立测试域和假后端，不发布 PX4 控制。真实飞行证据由上面的
SITL 场景单独产生，见 [验证记录](../../docs/validation/simulation/2026-10-08-bt-px4/REPORT.md)。
