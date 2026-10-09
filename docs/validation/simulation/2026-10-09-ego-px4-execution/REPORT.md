# EGO 曲线到 PX4 网关：实现与前置验证

2026-10-09，保持相机下偏 5°、官方 SITL GNSS／惯性 EKF 开发路线。本批接入 FlightServer 的 EGO NAVIGATE／RETURN 分支和曲线执行准入，未验证实际曲线飞行、同机深度地图或感知避障。

## 已实现

默认 `DIRECT` 基础后端保留。显式 `--navigation-backend EGO` 由受管 supervisor 设置能力开关，并将选项写入根任务参数与运行 manifest。此入口不自动启动地图或规划源；缺源则根任务在解锁前拒绝。

唯一 FlightServer 在 NAVIGATE／RETURN 派发一次 map goal，保持当前参考等待最多 2 s；实际规划结果经 EgoExecution 验证后，以原始 ROS start_time 采样 cubic B-spline，逆变换 map→PX4 local ENU，再沿原网关转换为 NED position setpoint。BT 不直接写 FMU，不产生第二个控制发布者。

绑定包含 coordinator、根 UUID、ControlSession UUID／generation／owner、物理 child UUID 与步骤索引；本次 goal stamp、定位 session/reset、地图 epoch/version、对齐 ID／generation 必须一致。只允许停止起点、parent_trajectory_id=0，一目标一次结果，单节点最多 256 个已消费曲线 ID。不能重用旧结果或把同世代变换更改当作新上下文。

网关重新检查最新 ESDF 的未知／占用／地图外体积、整段扫掠、速度／加速度／jerk、静止终点及已知区域；地图正常版本更新对剩余曲线重新复检。动态地图 source_stamp 也必须新鲜，不能用新 header 重发旧观测。规划源要求唯一发布者，并绑定 GID，单个新发布者替换也不能延续旧执行。

身体／跟踪半径按 profile 配置，另检查水平制动余量包络；水平余量不虚增到竖直方向穿过地面。暂停／取消／故障／步骤结束先撤销曲线，再进入原保持／制动或故障收尾路径。恢复从已确认停稳位置重新派发新 child／goal；暂不支持运动中曲线拼接／重规划。跟踪误差超出 profile margin、规划超时、地图／会话／源失效时停止任务；故障按既有 Offboard-loss 原生 Land 策略收尾，未声称全障碍场景故障落地已验证。

## 证据

- uav_nav_sim、uav_mission 构建通过；115 项执行准入、飞行网关及 PlanningContext 回归通过。
- `planning/`：真实 C++ EGO 与 ROS domain 91；合成 ESDF 中直线通路被墙挡住，实际生成 16 控制点、11.072 s 曲线，横向绕行 1.546 m。实际输出进入执行准入／采样，最后一轮复检耗时约 5.29 ms；非 identity 坐标转换通过。全程无 FMU／cmd_vel topic，三个子进程正常结束。
- fixture 用于独立规划契约，身体半径 .3 m、运动限制 (.5,1,2)、水平制动余量 0；不替代真实 flight profile 包线。单次 5.29 ms 不能证明大地图、重复复检或完整控制周期的实时性。
- `missing-planning-sitl/`：实际 x500 / PX4 / QGC / BT、运行 UUID `adf2e5d7-db16-450b-bfc9-f477e20d97e1`；选择 EGO 而不提供地图／规划源。根任务以 `INVALID_REQUEST:NON_UNIQUE_PLANNING_SOURCE` 拒绝，飞控命令记录为空，最终 landed / disarmed。总运行报告为 FAIL 是预期拒绝结果，不是任务完成。该收据形成于本批水平制动验证补充前，保留其原始源码摘要。

原始 JSON 和日志按需 gzip 保留字节。校验：`python3 docs/validation/simulation/2026-10-09-ego-px4-execution/check_archive.py`。

## 入口与剩余实现

```bash
./scripts/build_px4_flight.sh --packages-select uav_nav_sim uav_mission
./scripts/with_venv.sh bash -e -c 'source .deps/mission-install/local_setup.bash; python scripts/run_planning_context_smoke.py --execution-admission'
# 需要外部受控地图／对齐／PlanningContext + EGO 源；单独运行应拒绝解锁：
./scripts/sim.sh px4-flight --bt --navigation-backend EGO
```

PlanningContext 的 odom 输入应重映射 `/planning/source/odometry:=/uav/px4/localized_odometry`；地图为 `/planning/source/map`，对齐为 `/planning/source/alignment`，结果为 `/planning/bound_trajectory`。对齐必须等于受管 flight profile 的 map←odom 变换并绑定网关实例与 reset，不能由真值或 topic 名推断。根任务 backend 仍为 `PX4_KNOWN_REGION`，本批只开放受管已知区域内的规划执行能力。

下一步提供受管的同机 5° 深度／CameraInfo／TF、nvblox 地图／对齐源和 EGO 生命周期，记录定位／地图会话后执行真实曲线跟踪，再实现运动中重规划与完整绕障任务。当前不能标记 S5 导航／避障验收完成。
