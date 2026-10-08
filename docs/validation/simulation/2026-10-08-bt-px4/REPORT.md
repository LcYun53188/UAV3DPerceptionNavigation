# 本机 BT → PX4 W0 首批闭环验证（2026-10-08）

本批新增 `uav_bt` C++ Runner、可执行 XML、异步 ExecuteMission 客户端，以及与根任务 UUID/coordinator instance 绑定的 BT 进展租约。真实 PX4 起飞、定点导航、悬停、返航和原生降落复用已验证 W0 后端；BT 不直接发布 setpoint。额外验证运动中暂停/恢复、Runner 正常退出请求、强制退出及 tick 停滞的处置。

本批是 **S2 首批真实 PX4 后端接入**：默认 XML 只有一个 `ExecuteFlightMission` 异步节点，整条飞行序列的阶段仍由 FlightServer 执行。尚未新增树内逐步骤飞行 Action、外层 MissionServer 编排、EGO Navigate 适配或 algorithm 两航点树，不将整个 S2 标为完成。

## 最终真实 SITL 结果

| 场景 | 根任务真实结果 | 保持/进展观测 | 验收 |
| --- | --- | --- | --- |
| full | SUCCEEDED / LANDED_AND_DISARMED | 两航点误差 0.0870/0.0564 m；落点 XY 误差 0.0723 m；进展最大间隔 0.1059 s | PASS |
| cancel | CANCELED / STOPPED_AND_HOLDING | 保持约 29.99 s，最大漂移 0.0924 m | PASS |
| runner-exit | ABORTED / BT_PROGRESS_TIMEOUT | 保持约 30.00 s，最大漂移 0.1059 m；最后进展到制动 0.5193 s | PASS |
| runner-stall | ABORTED / BT_PROGRESS_TIMEOUT | 保持约 29.98 s，最大漂移 0.0944 m；最后进展到制动 0.5145 s | PASS |

最终运行标识：

- `full/`：`147bc70e-7ef4-4e37-a76d-e2b6bb1df546`。
- `cancel/`：`e9674113-9d71-40ad-96ef-3cec9039a384`。
- `runner-exit/`：`ab188dcd-df49-464e-9321-d39c73fe59d2`。
- `runner-stall/`：`f29f0cd1-980a-4aa2-a8c6-ad4f58125ed9`。

完整任务 HOVER 阶段时间 32.040/5.040 s，最大漂移 0.1120/0.1360 m；暂停漂移 0.0618 m。有效进展 1045 次。四场景任务活动阶段控制 trace 最大间隔 0.0204 s。

表中故障场景的 PASS 指正确检测并完成预期处置，根任务实际仍为 ABORTED。保持漂移从已确认交接时的实际位置计算；取消/故障后就地降落，不将距原起点距离解释为返航误差。独立真值仅用于观测，不作为定位、参考或状态就绪输入。

完整任务使用默认 2 m 起飞、两个航点、30 s 悬停、返航、3 s 悬停及降落，额外在运动中暂停约 3 s 后恢复新 child UUID。HOVER 阶段总时间包含停稳确认窗口。所有场景最后都观察到 landed=true、arming_state=DISARMED；每次根任务只派发一次。

控制参考追踪间隔记录于 flight-trace；该指标覆盖任务活动阶段的网关 tick，不等价于端到端 DDS/飞控接收延迟，也不覆盖全部最终保持阶段。进展最大间隔来自后端单调接收时间；正常运行约 10 Hz。租约 0.5 s 与 50 Hz 检测周期共同决定过期响应，不能把阈值写成零检测延迟。

## 生命周期与冻结配置

- 进展包含根 Action UUID、coordinator instance、严格递增 tick_sequence。旧实例、旧 UUID、重放和已失效根任务的进展均被拒绝。
- 首次握手允许最多 5 s，期间只在地面预流；未收到首个有效 tick 不发送模式切换/ARM。握手后 0.5 s 进展过期触发 BT_PROGRESS_TIMEOUT。
- 健康/实际控制仍有效时退役运动参考、制动并确认停稳、交接 HoldController 后才提交 ABORTED、cleanup_confirmed=true。保持最多 30 s，再请求原生降落。状态/实际控制无效时沿用既有故障策略。
- 最终保持不依赖已结束 Runner；原生 LAND 已提交后不因租约失效打断。进展恢复不能续租已终止根任务；本轮未单独实测 SIGSTOP 后 SIGCONT 恢复或新任务重新授权。
- SIGINT/SIGTERM 在可取消阶段调用树 halt，并保留客户端直到真实根终态。晚于 halt 的接受响应会触发补发取消。原生降落期间退出请求继续观察着陆，取消竞态被拒绝也不提前销毁客户端。
- 结果同时核验 ROS Action 终态、业务 result_code、cleanup_confirmed 与 mock=false；退出码 0=成功，130=确认取消，1=拒绝/失败/清理未确认。强制退出和冻结没有正常客户端终态，由后端根结果与物理状态证明处置。

PX4 v1.16.2 commit `54f0455ffcd755534539a7cf33a09a20bf71d29d`、Agent v2.4.3、对应 px4_msgs v1.16.2、Gazebo Harmonic、ROS Jazzy；控制/区域/估计器和故障参数延续 [W0 飞行基线](../2026-10-07-px4-flight/REPORT.md)。新增 lease 数值在 [W0.json](../../../../simulation/safe_regions/W0.json)。本机 BehaviorTree.CPP 4.9.0，rclcpp_action 28.1.18，nlohmann-json3-dev 3.11.3-1，ament_cmake_pytest 2.5.6。所有运行使用独立 domain 78、实例 7/system 8 和唯一 Gazebo partition，全部在本机，Jetson 继续延期。

## 构建、测试与复现

```bash
./scripts/build_px4_flight.sh
./scripts/sim.sh px4-flight --bt --flight-scenario pause-resume
./scripts/sim.sh px4-flight --bt --flight-scenario cancel
./scripts/sim.sh px4-flight --bt --flight-scenario runner-exit
./scripts/sim.sh px4-flight --bt --flight-scenario runner-stall
```

四包独立构建通过；任务、桥、仿真工具、BT 契约及航点审计回归合计 **181 passed**：

```bash
./scripts/with_px4_sim.sh bash -e -c 'source .deps/mission-install/local_setup.bash; PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q src/uav_bt/test src/uav_mission/test src/px4_comm_bridge/test scripts/test_sim_control.py scripts/test_sim_validation.py scripts/test_px4_sim_tools.py scripts/test_px4_flight_audit.py'
```

其中 9 个 ROS/BT 通信测试使用独立测试域与假飞行后端，不发布 PX4 控制：成功、失败、拒绝、mock 结果拒绝、清理未确认、终态矛盾、halt 等待清理、接受晚于 halt、降落期间退出。该组也通过 `colcon test --packages-select uav_bt` 注册执行。其他新增回归验证进展身份/重放拒绝、首次握手上限、失效制动和原生降落保护，以及中断航段不误计到达。测试日志、构建日志、vendor 核对日志以 gzip 原样保存，JUnit 见 bt_ros_contract.xunit.xml。

## 证据与开发中发现

`full/`、`cancel/`、`runner-exit/`、`runner-stall/` 含各次 manifest、Action 结果、状态/命令/事件、BT 进展与清理记录；独立真值、控制轨迹、原始客户端日志为 gzip。精确数值见 [metrics.json](metrics.json)。flight-observation 的 source_sha256 在运行开始采集，最终归档核对源码和编译后 Runner 哈希；授权临时 JSON 使用 0600，结束后删除，不归档 nonce。

首轮地面测试 `5cbdb2d2-1950-4103-97a2-e8170474ffea` 因进展握手超时失败，证据保存在 `failed-ground-handshake/`；无 ARM/模式命令、独立真值位移为零，不计飞行通过。初版每 100 ms 只 spin_some 一次，50 Hz Action 反馈占据了事件处理，使接受响应延迟。核对 [Jazzy Action 客户端源码](https://github.com/ros2/rclcpp/blob/jazzy/rclcpp_action/src/client.cpp#L312-L358) 的反馈优先顺序后，改为单线程持续处理通信、约 10 Hz tick 树；进展仍只在树 tick 后续租，没有独立保活线程。

首批四场景的物理结果已通过，但审计曾把 LEASE_BRAKE 前未完成的导航段误计为到达。已改为仅统计进入下一个合法任务步骤的航段，增加独立真值回归，并复跑最终四场景。早期运行原始记录仍保留在 `.cache/simulation/sitl/`（full `cd9ceb62-e17b-4e65-a62d-bee75de88144`、cancel `a2b9566d-9def-4a31-822f-174f9d2839bc`、exit `e89ad265-7c16-466c-b9c9-198635a5c286`、stall `a2c808aa-f57b-44a1-96bd-6be37459a983`）；其被打断航点误差不作为到达指标。

## 剩余工作

下一批补齐 S2 的 EGO Navigate Action、外层任务服务器/分步骤 BT 和 algorithm 两航点检查点回归，再扩展 S3/S4 的 Agent/网关失联、定位 reset、模式接管、重复运行与性能矩阵。本批每个最终真实场景运行一次，未达到计划中的正常任务 10 次/关键故障 3 次正式重复验收。感知避障、地图迁移、外部 VIO、真实 RC/ELRS 和实机均不在本报告通过范围。
