# EGO 导航 Action（algorithm）

`managed_goals=true` 的 Gazebo Executor 提供 `/uav/algorithm/navigate`
（`uav_nav_interfaces/action/NavigateToPose3D`）。Action 与 GoalManager 共用执行线程，
速度仍只由原 Executor 发布。适用于当前 identity `map→odom`、真值定位、零重力速度模型；
不连接 PX4，不提供自主起飞或飞行动力学验证。

## 请求与反馈

- `goal`：`map` 坐标系、有限 XYZ；当前只执行位置目标，不控制目标四元数。
- `position_tolerance_m`：0.15–1.0 m；底层最终到达门限仍为 0.15 m。
- `stopped_speed_mps`：0.01–0.2 m/s；角速度同时必须 ≤0.1 rad/s。
- `stable_duration_s`：0.5–10 s；连续满足停稳条件的单调时间窗口。
- `timeout_s`：1–600 s 的导航期限；到达/取消/故障后额外给 `stable_duration_s + 5 s` 清理时间。
- `control_session`：非零 UUID、generation>0、非空 owner，反馈原样回显。
- `map_session`：当前 MapSnapshot 的 `<map_id>:<epoch>`，地图版本增长不改变会话身份。

所有数值必须有限；不使用零值隐式默认值。忙、已有话题目标/自主探索、地图会话不符或
地图/里程计不新鲜时拒绝请求。已有话题目标需先取消，探索需先禁用。

10 Hz 反馈包含本 Action UUID、owner、递增事件序号、地图会话、当前位置、最终目标距离、
局部阶段和已完成分段数。局部观察点到达不会结束最终目标 Action。

当前 ControlSession 是单目标身份与反馈上下文，尚未提供跨子任务的父会话预约、
世代转移或 BT 进展租约。两个普通客户端目标之间不会自动保留父任务控制权。

## 结束与取消

成功结果需最终位置满足容差、活动/排队曲线和局部 token 已撤销，且新鲜里程计持续满足
线速度和角速度门限。停稳后再次核对终点，制动期间漂出容差返回 ABORTED。

取消使用 Action goal UUID。取消 ACK 只说明请求已接受：旧 token 在 ACK 前失效，
随后反馈 STOPPING，实际停稳后才返回 ROS CANCELED / `result_code=CANCELED` /
`cleanup_confirmed=true`。清理期间拒绝新目标，旧 UUID 的迟到取消不会作用于后继任务。

Action 占用期间，RViz `/uav/goal`、自主探索启停和旧 `/uav/cancel` Trigger 都拒绝修改任务。
地图切换/失效、定位或时钟异常、规划失败和导航超时均撤销目标、确认停止，再返回 ABORTED。
里程计接收时间、源时间和时间戳推进同时检查；时间戳停滞 ≥0.5 s 不算停稳。

清理期限耗尽返回 ABORTED、`cleanup_confirmed=false`，原因追加 `STOP_UNCONFIRMED`，
并锁定所有新 Action/旧入口。确认场景和定位恢复后需重启执行器；不自动重放旧目标。
`/uav/navigation/state` 在清理期间显示 STOPPING，最终为 REACHED/CANCELLED/STOPPED；
旧事件话题不携带 UUID，程序以 Action 的反馈和终态为准。

## 构建与回归

完整算法栈已构建后，可只增量构建接口和执行器，不重编 CUDA 感知包：

```bash
./scripts/build_navigation_actions.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./scripts/with_venv.sh \
  python -m pytest -q src/uav_nav_sim/test scripts/test_sim_control.py scripts/test_sim_validation.py
```

普通客户端的两航点/取消回归只支持受管 `lab/mapping` 场景，核对会话、域、分区和固定场景 hash，并用操作锁拒绝与初始化/扫描并发。
已有受管仿真时先查看状态，按需要退出；首次启动需要初始化观测区域：

```bash
./scripts/sim.sh start --layout lab --view none --background
./scripts/sim.sh init
ROS_DOMAIN_ID=68 GZ_PARTITION=uav_ego_lab ./scripts/with_venv.sh \
  python scripts/check_gazebo_navigation_action.py --output .cache/sim/action-two-waypoints
ROS_DOMAIN_ID=68 GZ_PARTITION=uav_ego_lab ./scripts/with_venv.sh \
  python scripts/check_gazebo_navigation_action.py --cancel \
  --waypoint -2.2 0.6 1.2 --output .cache/sim/action-cancel
```

两航点默认 `(-2.2,0.6,1.2)`、`(-3.2,-0.6,1.2)`。取消用例在原始 Gazebo 位移 ≥0.35 m 后
发送取消。输出 `result.json` 和 `trace.jsonl`，记录源文件 hash、UUID、反馈、过滤里程计和
原始 Gazebo 位姿；同时检查终态、实际位移、独立几何净空及结果后 1 s 保持漂移。

这些是普通 Action 客户端回归。algorithm MissionServer、两航点 BT、父任务暂停检查点/
恢复以及跨子任务控制权仍待实现；不能据此标记 S2 全部通过。
验证证据见 [导航 Action 报告](validation/simulation/2026-10-08-navigation-action/REPORT.md)。
