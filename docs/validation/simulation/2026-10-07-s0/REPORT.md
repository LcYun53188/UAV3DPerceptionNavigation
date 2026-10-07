# S0 第一批开发与验证

日期：2026-10-07。以下为调整范围前的历史验证记录。当时 S0 状态为进行中，E/Jetson 层 BLOCKED。用户随后明确暂不使用 Jetson；当前本机范围及新结果见 [本机开发调整记录](../2026-10-07-local-s0/REPORT.md)。原始结果与快照保留。

本批交付版本观测锁、algorithm profile、验收/结果 schema、指标配置校验、离线依赖 doctor 和 S0 主机回归运行器。保留原算法入口与原有 vendor 改动。没有实现新的任务 Action、BT Runner、PX4 飞行后端，也没有将 S0 标为完成。

## 已验证

- `sim.sh doctor`：OS/架构、锁定软件包、源码 commit、配置/patch hash、受管 vendor patch 一致。在线 TF/topic/clock 不在本批 doctor 能力内。
- 主机算法/仿真控制及新工具回归：247 passed；PX4 bridge mock/转换/状态测试：24 passed。
- run_id：`ffb3dc46-e4dd-44fb-ae7b-c8b3742b3177`。总结果 INCONCLUSIVE：Jetson 必需项没有实测证据。见 [result.json](result.json)、[配置快照](config.snapshot.json)、[环境审计](environment.json) 与同目录回归日志。
- 一次 lab 真值定位/速度模型导航：从初始化位置约 `(-3, 0, 1.2)` 前往 `(-1.5, 0, 1.2)`，实际前移约 1.5 m，最终 `GOAL_REACHED`，终点误差 0.000314 m。后续状态为 `navigation=REACHED`、`executor=HOLD`，见 [algorithm-goal.json](algorithm-goal.json) 与 [终态](terminal-state.json)。
- 真值离散检查：935 个 odometry 样本，最小球形包络净空 0.587175 m。采用现有 `check_gazebo_goal.py` 的固定 lab 解析几何和半径 0.3 m 球形包络；不是完整碰撞体连续检测，也不证明复杂障碍/全场景通过。
- 本次无 PX4；无动力学起降、VIO、Jetson 或硬件控制结论。只运行一次短距离导航，不作可靠性统计结论。

机器：Ubuntu 24.04.4 x86_64，ROS Jazzy，Gazebo Sim 8.11.0，RTX 4070 Laptop 8188 MiB / 驱动 595.71.05。具体软件包和源码版本见配置快照。程序基础 HEAD 为 `593a0d2bcaae46600e6f3bfd1fa476bfd16916db`，工作树包含开发计划与本批未提交改动；测试不是 clean-HEAD 验证。

## 复现

先确认没有其他受管或手工仿真，再运行：

```bash
./scripts/sim.sh doctor
./scripts/with_venv.sh python scripts/run_sim_scenario.py simulation/scenarios/S0_regression.yaml
# 预期退出 2：ARM 样例仍缺失。
./scripts/sim.sh start --layout lab --view none --background
./scripts/sim.sh init
ROS_DOMAIN_ID=68 GZ_PARTITION=uav_ego_lab ./scripts/with_venv.sh python \
  scripts/check_gazebo_goal.py --goal -1.5 0 1.2 --timeout 90 --output /tmp/s0-lab-goal.json
./scripts/sim.sh status
./scripts/sim.sh stop
```

本次受管 launch PID=11773，使用 domain 68 / partition `uav_ego_lab`；验证后已由 `sim.sh stop` 正常退出。启动日志保留于 `.cache/sim/launch-1791376877847352812.log`，完整主机回归目录保留于 `.cache/simulation/runs/ffb3dc46-e4dd-44fb-ae7b-c8b3742b3177/`。没有重置既有其他会话。初始化使用现有 set_pose 观察流程，不是自主起飞。

## 下一阶段入口与阻塞

1. S0 尚需固定 PX4 1.16 具体 tag/commit、匹配 Client 的 Agent、QGC 版本/hash。当前锁中 pending 不能视为版本冻结完成。
2. 需要 Jetson 访问方式及实际 JetPack/L4T/ROS/Isaac/CUDA 版本，运行接口最小构建、VIO/nvblox 加载和固定数据样例，记录内存/GPU。当前只完成工作站侧依赖审计，尚未完成 ARM 移植评估；S6/S7 不能进入验收。
3. 仿真 VIO 配置、传感器契约、时钟采集器与在线 doctor 待开发。既有 OAK-D 配置保持硬件时钟设置。
4. 下一批按 S1 增加 AircraftState、UUID/暂停协议与 FlightSession 交接 mock；工作可推进，但不能因此把 S0 缺项标为通过。完整 algorithm 指标模板仍为 INVALID_CONFIG。
