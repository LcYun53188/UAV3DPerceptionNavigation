# 本机仿真范围调整与 S0 主机回归

日期：2026-10-07。用户明确暂不使用 Jetson，全部仿真开发在当前工作站进行。

两份开发计划已同步：S0 的组件样例改为本机验证，S6 去除 ARM 前置依赖，S7 改为同机 Gazebo/PX4/感知/导航集成负载、内存与控制截止时间验证。Jetson 兼容矩阵、ARM 样例及跨机联调为后续可选阶段，不阻塞当前本机 S0–S8。

版本锁将 Jetson 从 pending 移到 deferred；主机验收配置将 `S0_JETSON_SAMPLE` 改为 N/A 并记录用户调整范围的理由。后续若重新将其设为 required，缺少目标证据仍返回 INCONCLUSIVE，不会虚构 ARM 验证通过。

本次主机回归 run_id：`3dc229a4-6bb0-406f-8824-99ec3a7512cc`。算法/仿真/工具测试 247 passed，PX4 桥测试 24 passed；Jetson N/A，总结果 PASS，退出码 0。见 [结果](result.json)、[配置快照](config.snapshot.json)、[离线环境审计](environment.json) 及同目录日志。已核对 schema、配置 hash 与证据文件引用。

该 PASS 仅覆盖当前 S0 主机回归，不表示整个 S0 或 PX4/VIO 飞行验收完成。仍需补齐 PX4/Agent/QGC 版本冻结、本机组件样例及后续任务/控制/感知实现。上一批 [lab 实际导航与历史结果](../2026-10-07-s0/REPORT.md) 保留；本次没有重新启动仿真或重复导航测试。

复现当前主机回归：

```bash
./scripts/with_venv.sh python scripts/run_sim_scenario.py simulation/scenarios/S0_regression.yaml
```
