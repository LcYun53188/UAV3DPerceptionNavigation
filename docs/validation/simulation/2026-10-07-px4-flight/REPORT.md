# 本机 PX4 W0 飞行任务验证（2026-10-07）

已在真实 PX4 v1.16.2 SITL + Gazebo Harmonic x500 上完成：地面解锁、2 m 起飞、两个定点导航、悬停、返回起点 XY、原生降落及触地解除武装。完整任务还验证了运动中暂停、确认停稳、恢复新子任务，以及原生降落期间拒绝取消。结果来自实际 ROS Action、PX4 状态回读和独立 Gazebo 模型真值，不使用 mock、set_pose 或真值定位注入。

## 最终验证结果

| 场景 | 实际任务结果 | 独立观测 | 验收 |
| --- | --- | --- | --- |
| 完整任务 + 暂停/恢复 | SUCCEEDED / LANDED_AND_DISARMED | 两航点误差 0.0509 / 0.0333 m；返航误差 0.0366 m；最终落点距起点 XY 0.0381 m；触地且解除武装 | 通过 |
| 运动中取消 | CANCELED / STOPPED_AND_HOLDING | 取消时速度 0.5052 m/s；3.2755 s 后确认停稳交接；最大制动位移 0.4196 m；保持约 30 s，最大漂移 0.0826 m；期限后原生降落，触地解除武装 | 通过 |
| 时钟暂停 1.1 s 后恢复 | ABORTED / STALE_OR_INVALID_AIRCRAFT_STATE | 时钟故障锁存；输出计数恢复前后均为 860；PX4 Offboard-loss 原生降落，触地解除武装 | 通过（预期故障退出） |

完整任务第一段 HOVER 总阶段时间 32.0200 s（包含 2 s 停稳确认与请求的 30 s 保持），1601 个真值样本最大漂移 0.0958 m；第二段为 5.0202 s（确认 + 3 s），最大漂移 0.0451 m。悬停期间 PX4 最大速度 0.0566 m/s。暂停期间真值最大漂移 0.0762 m，恢复使用新 child UUID。

取消后的 30 s 保持独立于已返回的 Action 结果，由 FlightSession 持有；随后发送原生 LAND，观察着陆解除武装，最终释放控制权。时钟故障时位置的 0.1 s 新鲜度门槛先触发 ABORTED，随后 0.5 s clock fault 仍锁存；恢复 clock 后不重发旧参考。故障场景网关只曾发送 Offboard/ARM 命令，降落由冻结的 PX4 丢失 Offboard 策略执行。取消和时钟故障均就地降落，其落点距原起点约 1.11 / 1.61 m，不作为返航落点误差。

## 可复现入口

从仓库根目录运行，先准备已锁定的 PX4/Agent/消息依赖（见 [运行说明](../../../../simulation/px4/README.md)）：

```bash
./scripts/build_px4_sim.sh --jobs 4
./scripts/build_px4_flight.sh
./scripts/sim.sh px4-flight
./scripts/sim.sh px4-flight --flight-scenario pause-resume
./scripts/sim.sh px4-flight --flight-scenario cancel
./scripts/sim.sh px4-flight --flight-scenario clock-fault
```

默认任务为起飞 → 航点 (3,2,2) → 悬停 30 s → 航点 (-2,2,2) → 返回起点 XY → 悬停 3 s → 降落；坐标为相对起点 ENU 偏移。每次入口完整运行地面状态预检后执行任务，再清理本次拥有的进程组。QGC 使用独立配置、关闭串口自动连接。运行入口包含仿真授权、独立 ROS domain 78、PX4 instance 7/system 8 和每次唯一 Gazebo partition。

## 冻结配置与实现

PX4 v1.16.2 commit `54f0455ffcd755534539a7cf33a09a20bf71d29d`，Agent v2.4.3，匹配 px4_msgs v1.16.2；详见各次 manifest。W0 区域绑定空旷平地场景及 x500 模型哈希，边界及机体/跟踪/制动余量在 [W0.json](../../../../simulation/safe_regions/W0.json)。固定 map→odom 对齐使用平移 (10,-4,0.3) m 和 yaw 0.35 rad，避免 identity 变换掩盖坐标错误。

转换器将 PX4 NED 姿态/位置转换到 ENU，将速度转换到 ROS child-frame FLU，并处理协方差及源采样时间。控制参考为端点停稳的五次曲线；速度、加速度、jerk 上限为 0.6 m/s、0.5 m/s²、0.6 m/s³。RETURN 为区域内返回起点 XY 的轨迹；不是 PX4 高度策略不同的 RTL 模式。LAND 使用原生 AUTO_LAND。

本次 rootfs 参数覆盖为 `COM_RC_IN_MODE=4`、`COM_OF_LOSS_T=0.5`、`COM_OBL_RC_ACT=4`、`COM_DISARM_LAND=2`、`EKF2_MAG_TYPE=6`。MAG_TYPE=6 使用初始磁航向，之后惯性/GNSS 更新；航向有效性、误差和 reset 计数检查保留。位置接收/源年龄 0.1 s，状态 0.75 s，着陆状态 1.2 s，时钟无进展 0.5 s。该配置属于 W0 模型基线，未迁移到硬件。

任务服务器以异步 Action 和 SingleThreadedExecutor 处理状态与控制更新。根任务、子任务、FlightSession、控制世代分开记录；暂停/取消先制动、实际停稳再交接，恢复从当前状态生成新参考。源状态失效、定位 reset、越界、意外模式/武装状态及控制发布者冲突会撤销旧输出。任务、暂停和保持均有单调时间上限。

## 证据与回归

最终三次运行：

- `full/`：`13c71b3f-ca94-465c-bde4-5b9bcccefffc`，场景 `pause-resume`，完整默认任务成功。
- `cancel/`：`f49a0143-1926-484f-a807-146a61e5279d`。
- `clock-fault/`：`3d912d2e-dbd1-40e5-949e-a7026d6fe4e4`。

每个目录含 manifest、地面 observation、flight-observation、事件、命令、PX4 状态历史、诊断、清理记录及客户端日志；控制轨迹和独立真值保存为 JSON gzip。汇总数值见 [metrics.json](metrics.json)。各次 flight-observation 的 `source_sha256` 在开始时记录运行源码哈希；归档时均与最终网关、几何转换、里程计转换、客户端及 supervisor 源码核对一致。manifest 保留运行当时版本锁快照；后续版本锁仅更新完成范围描述，不改依赖 pin。

三包独立构建通过。以下针对任务、桥接、仿真工具的回归合计 **161 passed**：

```bash
./scripts/with_px4_sim.sh bash -e -c 'source .deps/mission-install/local_setup.bash; PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q src/uav_mission/test src/px4_comm_bridge/test scripts/test_sim_control.py scripts/test_sim_validation.py scripts/test_px4_sim_tools.py'
```

vendor 管理补丁核对通过，原有第三方 gitlink 保持。Python 编译、shell 语法及 Git whitespace 检查通过。

## 开发中故障与适用边界

早期真值桥接丢失模型身份，已改用独立 Gazebo 原生 Pose_V 读取；不向控制器提供真值。多线程阻塞式 Action 原型出现陈旧/乱序回调，改为单线程异步执行后完成最终三场景；该观察不归因为已证明的 DDS 底层缺陷。默认磁航向 Automatic 在约 1.5 m 离地后产生 heading reset，曾触发旧任务退出；冻结 MAG_TYPE=6 后仍保留 reset 拒绝机制。早期失败原始日志保留于 `.cache/simulation/sitl/`，包括 `a38a10dd-7bde-4cd6-8671-faf3bfa6917f`；该失败不计安全着陆通过。

本报告证明本机已知 W0 区域的真实 PX4 任务闭环；没有完成感知避障、EGO/nvblox 接入、外部 VIO、BT XML 调度、全部 S3/S4 故障矩阵、人工 QGC GUI 验收或实体 RC/ELRS 接管验证。Jetson 按用户要求延期。完整仿真开发计划的其余阶段仍按计划推进，不以这三场景替代全部验收。
