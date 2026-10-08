# algorithm 两航点 BT 与 MissionServer

`uav_bt/algorithm_mission_server` 在现有 EGO/nvblox 执行器之上提供父任务：

- `/uav/algorithm/execute_mission`：`ExecuteMission` Action，根 UUID 直接使用 ROS goal UUID。
- `/uav/algorithm/pause_mission`、`resume_mission`：`PauseMission` / `ResumeMission`。
- 执行器的 `/uav/algorithm/control_session`：`ManageControlSession`，预约/释放父会话。
- `/uav/algorithm/control_status`、`mission_progress`：会话状态与实际 BT tick 进展。
- 子导航沿用 [`/uav/algorithm/navigate`](NAVIGATION_ACTION.md)，仍只有原 Executor 发布速度。

适用于 Gazebo lab/expanded 的真值定位、identity map/odom、零重力速度模型。
本批实时验收使用 lab；不连接 PX4，不证明飞机动力学、VIO 或定位重置适配通过。

## 根任务与 BT

请求使用 `backend=ALGORITHM`、`mission_type=WAYPOINTS`，`parameters_json` 示例：

```json
{
  "coordinator_instance": "<当前服务端实例>",
  "map_session": "<当前 MapSnapshot.map_id>:<epoch>",
  "waypoints": [[-2.2, 0.6, 1.2], [-3.2, -0.6, 1.2]],
  "navigation_timeout_s": 90.0
}
```

发送前用 `ros2 param get /algorithm_mission_server coordinator_instance` 读取当前实例，
填入请求；该参数只读，每次进程启动随机生成，旧实例请求不能预约新进程的会话。
回归脚本自动读取此参数。

支持 2–32 个有限 XYZ 航点。每航点导航预算 1–120 s，默认 90 s；根 `timeout_s`
为 20–600 s 的单调时间总预算，包含预约、导航、暂停和清理。提前保留 7 s 清理窗口，
未在总期限内确认清理则 ABORTED / `cleanup_confirmed=false`，服务端拒绝后续任务。

XML `algorithm_waypoints.xml` 使用异步 `ExecuteWaypoints` 节点顺序调用导航 Action，
每个子目标只发送一次，不跳过失败航点、不循环重发。根反馈给出父 UUID、唯一进程实例、
控制世代、子 UUID、阶段、航点索引（`ExecuteWaypoints[index]`）和剩余总预算。

根任务结束前必须收到实际子终态并释放会话，严格校验 ROS 终态、业务码、
`cleanup_confirmed` 和 `mock=false`。矛盾子结果不能伪装成成功取消；会话释放还会独立
验证新鲜里程计停稳。成功、取消、故障后都不重放旧曲线。

## 父会话与进展租约

父会话 UUID 等于根 UUID，owner 是随机生成的 MissionServer 实例，generation 每根任务递增。
预约覆盖所有航点、航点间隙和暂停期间。普通导航客户端、RViz 目标、旧取消服务及自主探索
不能在该期间接管。已释放的 owner/generation 不得重新导航，后继任务使用新授权。

预约握手最多 2 s，没有首个有效 BT tick 时不授权任何子导航。之后租约为 0.5 s，
进展必须匹配父 UUID/实例且序号严格递增。根树约 10 Hz tick 后才发送进展；暂停时仍 tick
阶段管理与健康检查，无独立保活线程。重放、旧实例、旧 UUID、过期后补发都不能续租。

租约过期、地图会话变化、定位/时钟异常会撤销子目标、清空曲线并锁存故障。
根进程恢复后只做终止清理，不能恢复旧导航。SIGTERM/SIGINT 会等待确认停止/释放，
活动根任务返回 ABORTED / SERVER_EXIT_REQUESTED；SIGKILL 时根结果可能不可得，
不能据后端停止就编造根终态。若根进程丢失，需在确认停稳后显式释放旧会话或重启执行器。
算法速度模型在无任务时保持原执行器的零平移/回正输出，不采用 PX4 的空中保持或降落策略。

当前执行器最多记录 256 个 owner 的世代，根任务最多缓存 256 个服务请求 ID。
达到上限会拒绝新预约/请求；不会清除记录以自动放行旧授权。

## 暂停、恢复与检查点

`RUNNING→PAUSING→PAUSED→RESUMING→RUNNING`：暂停请求接受后取消当前子导航，
收到已确认停稳的子终态才进入 PAUSED；父 Action 始终活动。暂停中取消优先转 STOPPING，
不得在停稳后误进入 PAUSED。恢复使用当前位姿重新发子 UUID/局部 token/轨迹，
沿用本航点剩余预算；已停稳暂停时间不消耗子导航预算，但消耗根总预算。

暂停服务请求包含根 UUID、反馈中的 coordinator_instance 以及非零 request_id。
相同 request_id/操作返回原决策快照，跨操作复用同一 ID、旧根 UUID和旧实例均拒绝。
服务 ACK 不是停稳确认，当前实际阶段以根 Action 反馈为准。

暂停默认最多 60 s；ROS 参数 `pause_timeout_s` 可设为 `(0,60]`，不得靠反复续租延长。
达到暂停上限/总预算或健康失效即终止清理。

ROS 参数 `checkpoint_file` 可保存原子更新的诊断 JSON，含任务定义及 SHA-256、根身份、
航点索引、地图会话、世代、子 UUID、剩余总/导航预算。只支持同进程内暂停恢复，
`restart_resume_supported=false`；新进程不读取文件自动续跑。当前定位契约为 Gazebo 真值
identity，不包含 VIO reset_id/非 identity 定位会话恢复，相关工作属于后续坐标/定位适配。

## 构建与验证

完整算法栈已安装后，增量构建接口、执行器和 BT，不重编 CUDA：

```bash
./scripts/build_navigation_actions.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./scripts/with_venv.sh python -m pytest -q \
  src/uav_nav_sim/test src/uav_bt/test scripts/test_sim_control.py scripts/test_sim_validation.py
```

已有仿真先查看 `sim.sh status`。新的 lab 建图会话需完成初始化并等待退出码 0：

```bash
./scripts/sim.sh start --layout lab --view none --background
./scripts/sim.sh init
ROS_DOMAIN_ID=68 GZ_PARTITION=uav_ego_lab ./scripts/with_venv.sh \
  python scripts/check_gazebo_algorithm_mission.py --scenario pause-resume \
  --output .cache/sim/mission-pause-resume
```

另外两个场景为 `--scenario cancel-pausing`、`--scenario runner-stall`，使用新的输出目录。
故障回归还支持 `--scenario clock-stall` 和 `--scenario paused-map-change`：前者通过
Gazebo 世界控制服务暂停/恢复真实时钟，验证停更期间持续零命令且不误报清理；后者在
PAUSED 时调用真实地图保存服务，保留 `map-bundle/` 并使 epoch 变化，验证父任务撤销。
时钟暂停在异常清理中也会尝试恢复。故障恢复不会自动续跑旧任务；先确认实际清理结果，
再提交绑定当前地图的新根任务。持续里程计缺失不能确认清理时，服务端/执行器锁止，
新鲜数据恢复不会自动解除该锁止。
每轮由客户端拥有并清理其 MissionServer 进程，保留 Gazebo 与地图；拒绝重复启动服务端。
操作锁避免与初始化/扫描并发。发送有效根任务前，用必定被拒绝的 DISCOVERY_PROBE
检查 SendGoal 双向响应通道；服务发现本身不视作响应就绪。接受回调长期未确认时，
MissionServer 锁定 ACCEPTANCE_UNCONFIRMED_RESTART_REQUIRED，不自动重试有效任务。

回归独立记录原始 Gazebo 位姿、过滤里程计、轨迹 token、根反馈、进展和控制状态；
通过实际 GetResult 服务读取子结果。暂停用例还主动重放旧轨迹并尝试旧入口，检验拒绝
且保持稳定。Runner 停滞采用 SIGSTOP 后等待后端停稳，再 SIGCONT 观察实际 ABORTED，
不把暂停进程当作暂停任务。

也可在正确域/分区内独立启动常驻服务端，再由自己的 Action 客户端提交任务：

```bash
ROS_DOMAIN_ID=68 GZ_PARTITION=uav_ego_lab ./scripts/with_venv.sh \
  ros2 run uav_bt algorithm_mission_server --ros-args -p use_sim_time:=true \
  -p checkpoint_file:=$PWD/.cache/sim/algorithm-checkpoint.json
```

回归脚本拒绝接管这个常驻服务端；使用回归入口前先正常退出它。
验证记录见 [MissionServer/BT 报告](validation/simulation/2026-10-08-algorithm-mission/REPORT.md)。

新增地图/时钟故障与恢复证据见 [故障回归报告](validation/simulation/2026-10-08-algorithm-faults/REPORT.md)。
