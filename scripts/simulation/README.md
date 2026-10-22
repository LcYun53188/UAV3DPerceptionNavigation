# 仿真代码目录

[仿真资源目录](../../simulation/README.md) · [脚本总目录](../README.md) · [运行说明](../../docs/simulation/LAUNCHERS.md)

统一入口是 `./scripts/sim.sh`，仅提供 `run manual`（光流纯手动）和 `run full`（完整项目）。`manual.py` 统一管理两种程序的 HID／模拟输入，`session.py` 记录运行身份，`control.py`／`control_ros.py` 提供新终端状态、地图和关闭操作；`cleanup.py` 预览或清理监督器退出后的残留进程。本目录根层维护 CLI、菜单、配置注册、后端路由、环境和策略；具体工具按职责放入子目录。

| 目录 | 职责与主要入口 |
| --- | --- |
| `algorithm/` | `sim_control.py` 管理算法会话，`sim_ros_control.py` 执行有界 ROS 操作；地图扫描、探索与导航回归同目录维护 |
| `px4/` | `run_px4_sitl_smoke.py`、`run_px4_flight_tasks.py`、BT 客户端、深度检查与地面定位选择 |
| `flow/` | `run_px4_mtf01.py`、光流与测距桥、独立 Position 运行器和融合切换审计 |
| `pocket/` | 面板、USB 输入、虚拟设备、开关映射、任务意图与持续操作会话 |
| `vio/runtime/` | `run_px4_vio_sensors.py`、传感器资产、场景生成、建图、融合与在线观测 |
| `vio/sdk/` | SDK 参数、运行回执、性能跟踪、协方差头文件与原生回放源码 |
| `vio/analysis/` | 已完成运行的分析、绘图、验收、故障诊断和证据归档 |
| `validation/` | 仿真公共校验、环境检查、S0 回归、任务协议与规划契约检查 |

配置、任务配方和插件资源位于 `simulation/`，构建器位于 `scripts/build/`，硬件采集与台架工具位于 `scripts/hardware/`。不要把新仿真工具继续放到根目录。

## 规范路径与兼容

旧脚本名和原生源码的兼容链接集中到 [scripts/compat/](../compat/README.md)，完整迁移表见 [layout.json](../compat/layout.json)。旧 `scripts/<工具>` 已从根目录移出，按旧名执行时改用 `scripts/compat/<工具>`；同一工具只有一份真实实现。VIO 资源的旧路径链接仍位于 `simulation/px4/vio/`，历史报告保留原样。

公共模块路径由 `scripts/support/paths.py` 根据迁移表配置。规范 Python 入口自行加载该支持；环境包装器也保留旧模块名的导入能力，不要求在根目录放同名文件。

Python 工具以真实文件位置定位工作区，并支持从其他目录按绝对路径执行。仅直接运行支持该方式的工具；在线工具可能启动仿真、发送目标或注入故障，不能批量执行来验证目录。

```bash
# 从项目根目录预览，不启动仿真
./scripts/sim.sh run manual --dry-run
./scripts/sim.sh run full --input-source virtual --dry-run

# 目录、链接与语法检查，不访问飞控
python3 scripts/tools/maintain_catalogs.py --check
```

运行证据的输入摘要包含源码及规范路径；代码迁移不改写历史报告或构建回执。若受管入口报告构建输入变化，使用对应 `scripts/build/` 构建器重新生成回执，不绕过校验。

专项审计、故障注入和历史算法运行器保留为内部开发工具；总菜单及 CLI 不再注册这些启动类型，旧公共启动命令会提示迁移到两个程序。
