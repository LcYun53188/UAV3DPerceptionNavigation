# 仿真验收配置（S0 首批）

当前已实现离线依赖审计、配置校验、主机回归运行与结果归档；完整任务、PX4、在线 topic/TF/clock doctor 尚未实现。

安装工具依赖（独立于 CUDA 全量构建）：

```bash
uv pip install --python .venv/bin/python -r requirements/simulation-validation.txt
./scripts/sim.sh doctor
./scripts/with_venv.sh python scripts/run_sim_scenario.py simulation/scenarios/S0_regression.yaml
```

`doctor` 对比版本锁中的 OS/架构、ROS 软件包、vendor commit、配置/patch hash，并调用已有 vendor patch 检查。其 PASS **仅表示离线依赖一致**。打印的后续控制/本机组件样例 pending 不被隐藏，也不代表整个 S0 通过；topic、TF、发布者、clock、飞行状态检查尚待后续阶段接入。

`algorithm_s0.yaml` 是当前可运行的主机回归配置。运行器固定执行已知回归命令，不从 YAML 执行任意命令，不启动或控制 Gazebo。每次生成 UUID 目录，保留配置/hash、Git 状态/diff、工具源码 hash、环境审计、stdout/stderr、退出码与结果。输出默认在 `.cache/simulation/runs/`；`--output-root` 可选择持久目录，`--timeout` 指定每组回归的单调等待期限。超时清理该组进程，不发送全局信号。

2026-10-07 用户明确暂不使用 Jetson，当前 S0 主机回归将 Jetson 样例设为 N/A，并记录延期理由；本机必需回归全部通过时，该主机回归范围的总结果可为 PASS。退出码：PASS=0、FAIL=1、INCONCLUSIVE=2。此 PASS 不表示完整 S0 已完成：PX4 版本冻结、本机组件样例等仍需补齐。后续恢复 Jetson 时重新启用目标验证；若将该项改回 required，缺少目标采集器/证据仍返回 INCONCLUSIVE。历史 INCONCLUSIVE 报告保留。

`algorithm.yaml` 是后续完整算法验收矩阵模板，尚未冻结 metrics，加载它会返回 INVALID_CONFIG。T05 仅指算法适用取消阶段；T06 的 PX4 模式接管部分 N/A 不表示未来任务授权逻辑可以省略。S2/S3 开始实现任务/时钟后，应拆分逐项证据与适用理由。

`schema/acceptance.schema.json`（上级目录）定义每项指标的来源、时域、运算符、阈值、窗口、期限、处置、用例、证据、冻结阶段与 hash。每项 `config_hash` 为去掉自身字段后、键排序、紧凑 JSON 的 SHA-256；全次运行的 hash 包含展开配置和版本锁。阈值更改必须重新冻结；缺字段、重复 ID、未知用例或 hash 不一致会拒绝配置。`sim_validation.evaluate` 只接受已由采集器按窗口整理的数值，缺失/非有限数据不计 PASS；窗口/时间源采集器尚待各阶段实现。

`schema/result.schema.json` 定义四种结果。必需项为 N/A 或 INCONCLUSIVE 时，总结果不能 PASS。S0 主机回归允许 metrics 为空，以测试框架自身的断言/退出码提供证据；完整验收配置不允许空指标。

当前版本锁是工作站观测基线，记录了 cuVSLAM VIO-only 源码与配置。QGC v5.1.5 文件已锁定，PX4 SITL v1.16.2、Agent v2.4.3 与 px4_msgs v1.16.2 已构建并通过基础链路冒烟，任务/状态/控制与 VIO 样例仍待完成，Jetson 兼容组合移至后续可选阶段；S3 仍需完成对应版本冻结后才能正式验收。现有 OAK-D 配置 `use_sim_time=false` 是硬件配置，不能直接充当后续仿真 VIO profile。
