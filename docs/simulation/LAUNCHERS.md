# 仿真启动入口与目录规划

[项目首页](../../README.md) · [脚本总目录](../../scripts/README.md) · [算法控制手册](SIMULATION_CONTROL.md)

日常只使用 `scripts/sim.sh`。无参数打开菜单，`run` 使用同一套配置启动；算法会话通过 `algorithm` 控制，专项诊断使用 `advanced` 传递运行器原生参数。

## 全局无 GNSS 策略

全部受管仿真使用无 GNSS 环境，不提供 GNSS 开关。PX4 在所有预设和环境变量合并后强制 `EKF2_GPS_CTRL=0`；Gazebo 公共服务器配置不加载 NavSat 系统，继承的 x500 卫星传感器不产生数据。运行器校验场景中不得重新加载卫星插件，manifest 记录 `localization_policy` 和实际参数摘要。合成 ROS 验证也关闭 GPS 发布，并拒绝 `enable_gps:=true`。

VIO／光流链路使用各自的局部定位。算法建图／导航仍使用 Gazebo 真值和简化动力学，不能作为无 GNSS 飞行定位验收。独立相机和基础 DDS／时钟检查允许定位尚未就绪，结果单独报告 `localization_ready`，不授予飞行许可。

旧 `px4-flight`／`px4-observe` 和独立深度 `map`／`plan` 依赖 GNSS 定位，尚未接入无 GNSS 定位，因此统一入口、兼容入口及直接 Python 运行器均拒绝启动。深度建图／EGO 请使用已接通的 VIO 仓库链路。历史 GNSS 证据与手册保留，但不能绕过此约束。

## 分层菜单

主菜单按操作目标组织，功能配置放二级，兼容场景放三级；任务点与行为树配置在选定场景后继续选择。

```text
1. Gazebo＋RViz：纯手动飞行
   ├─ 二级：VIO 开／关 × 建图开／关
   └─ 三级：与该组合兼容的场景
2. Gazebo＋RViz：自动建图
   ├─ 二级：自动初始化 → 自主探索建图（只有一项）
   └─ 三级：扩展场景／实验室
3. Gazebo＋RViz：任务点／行为树导航
   ├─ 二级：不加载地图／加载现有地图
   ├─ 三级：扩展场景／实验室
   └─ 任务配置：RViz 单点／多航点行为树／自定义行为树
4. 传感器、定位与环境检查
   └─ VIO 联合审计／深度检查／PX4 状态检查／离线环境检查
5. 辅助工具与高级配置
   └─ 虚拟 Pocket／场景查看／Isaac Sim／原始配置列表
l. 加载已保存的 JSON 配置
0. 退出
```

每层输入 `0` 返回主菜单，`Ctrl+C` 退出。三级场景只显示所选流程对应的场景，不把算法实验室场景当作已接入 PX4 飞行的场景。检查工具与原始配置保留在后两个分组，日常飞行／建图／导航不再需要从平铺的底层组件列表中挑选。

| 流程 | 当前可运行范围 | 启动后行为 |
| --- | --- | --- |
| 纯手动飞行 | VIO 开＋建图开＋仓库静态障碍场景 | 打开 Gazebo／RViz／QGC；自动优先实体 Pocket，否则使用虚拟设备；拒绝 PREPARE／RUN 等自动任务意图 |
| 自动建图 | expanded／lab 算法仿真 | 启动后自动局部初始化，成功后才开启自主探索；使用真值定位和简化速度模型，全场覆盖以探索结果为准 |
| 任务导航，不加载地图 | expanded／lab，RViz 单点导航 | 自动初始化起点，随后在 RViz 用 2D Goal Pose 选点 |
| 任务导航，加载地图 | expanded／lab，RViz 单点导航 | 等待 HOLD、加载所选地图包并确认就绪；不执行初始化扫描 |

手动模式的无 VIO＋RViz、VIO 与建图独立开关，尚未接通统一启动链路。多航点交互选点和任意自定义行为树也尚未接入该菜单；已有固定 BT 的专项入口继续保留。对应选项显示“待接入”，选择后不会启动替代流程。旧 `vio-pocket` 类型仍保留自动任务能力；新的 `manual` 类型使用独立 `--manual-only` 策略。

```bash
# 查看菜单层级；仅显示，不启动
./scripts/sim.sh menu --list
# 自动初始化，再自主探索
./scripts/sim.sh run autonomous-mapping --layout lab
# 自动初始化，准备 RViz 选点
./scripts/sim.sh run navigation --layout lab
# 加载已经保存、与场景匹配的地图，准备 RViz 选点
./scripts/sim.sh run navigation --layout lab --load-map --map-bundle .cache/maps/lab
# 纯手动 VIO＋建图；自动选择实体或虚拟 Pocket
./scripts/sim.sh run manual
```

手动模式默认 `--input-source auto --device auto`：优先使用实体 RadioMaster Pocket（8 轴／24 按钮）及稳定的 by-id 路径；未连接实体时复用已有虚拟 Pocket（9 轴／3 按钮），没有虚拟设备则自动打开面板。创建失败会在启动 Gazebo 前终止并输出日志路径。实体存在但不可读或类型不匹配时报告错误，不在会话中自动切换控制源。退出只关闭本次创建的面板。实体及虚拟输入均需在 QGC 校准并启用，不自动解锁或起飞。专项 `vio-pocket`／`flow-pocket` 仍要求预先启动虚拟面板。

初始化或地图加载失败时不进入后续操作，并清理本轮新建的算法会话；不接管另一套运行中的仿真。地图选择保持原有 `scene_id` 校验。上面的流程可保存为原格式 JSON，菜单和命令行都执行相同校验。

## 日常使用

```bash
# 交互选择仿真和配置
./scripts/sim.sh
# 列出全部配置类型
./scripts/sim.sh list
# 查看某一类型的选项
./scripts/sim.sh run algorithm --help
# 小实验室、在线建图、仅 RViz；先预览启动命令
./scripts/sim.sh run algorithm --layout lab --view rviz --dry-run
# 实际启动
./scripts/sim.sh run algorithm --layout lab --view rviz
```

菜单按层级选择，回车使用显示的默认值。预览后选择 `1` 启动、`2` 按层级修改、`3` 保存配置、`0` 返回。主菜单输入 `l` 加载配置；`Ctrl+C` 或输入结束退出。`sim.sh list` 列出下表中的配置标识，`sim.sh menu --list` 显示菜单层级。

| 配置类型 | 场景与作用 | 常用选项 |
| --- | --- | --- |
| `manual` | 手动 VIO＋建图仓库链路，禁止自动任务 | `--input-source auto/physical/virtual`、`--device auto`、`--qgc-config`、`--session-duration`；未接通的功能组合会被拒绝 |
| `autonomous-mapping` | 自动初始化＋自主探索建图，Gazebo／RViz | `--layout` |
| `navigation` | RViz 任务点；自动初始化或加载地图，Gazebo／RViz | `--layout`、`--load-map`、`--map-bundle`、`--task-source`；未接通的 BT 选择会被拒绝 |
| `algorithm` | expanded／lab，真值定位＋简化速度执行器＋nvblox／EGO | `--layout`、`--mode`、`--view`、`--goal-height` |
| `scene` | 障碍场地、基础演示、RMUC／RMUL 2024／2025；只查看传感器 | `--arena`、`--mid360`／`--no-mid360` |
| `px4-flight` | 旧 GNSS W0 任务，已停用 | 无 GNSS 定位待接入 |
| `vio-pocket` | 无 GNSS 仓库 avoidance，SDK Schur64＋VIO／深度／EGO | `--device`、`--qgc-config`、`--session-duration` |
| `flow-pocket` | 无 GNSS 基础仓库，纯光流／测距操作，无地图避障 | `--device`、`--qgc-config`、`--duration`、`--session-duration` |
| `flow-local` | 纯光流自动起飞／悬停／降落测试 | `--control native/mission`、`--duration`、`--no-ui` |
| `vio-audit` | baseline／complex 仓库，未解锁联合融合与停源审计 | `--variant`、`--duration`、`--fault`、`--resolution`、`--rtf`、`--renderer`、`--no-ui` |
| `depth` | 无 GNSS PX4 深度相机检查；独立 map／plan 已停用 | `--level camera/map/plan`、`--duration`、`--no-ui` |
| `sitl` | PX4 基础状态、DDS、时钟、QGC 检查；不飞行 | `--duration`、`--no-ui` |
| `panel` | 独立虚拟 Pocket 面板 | `--uinput`／`--no-uinput` |
| `isaac` | 已安装的 Isaac Sim 4.5 UI | 沿用 `ISAAC_SIM_*` 环境变量 |
| `doctor` | 离线依赖检查 | 不判断在线 topic／TF／clock 就绪 |

**`flow-local` 在预检通过后会自动起飞。** Pocket 类型启动后由操作员发起任务；先在另一终端运行 `./scripts/sim.sh run panel`，再填写面板打印的实际设备路径。VIO Pocket 使用固定的 avoidance／Schur64／VIO 主定位配置；复杂仓库仅在审计类型中提供，不能随意替换飞行场景。

```bash
# 终端 A：虚拟设备保持运行
./scripts/sim.sh run panel
# 终端 B：使用实际设备路径；可选择已有会话的 QGC 校准
./scripts/sim.sh run vio-pocket --device /dev/input/js0 --session-duration 3600
```

配置启动负责选择参数；安装、构建、校准和验收条件仍按[安装指南](../setup/INSTALLATION.md)、[VIO Pocket](../flight/VIO_POCKET_AVOIDANCE.md)、[光流操作](../flight/MTF01_STANDALONE_FLIGHT.md)完成。

## 配置保存与覆盖

菜单保存到 `.cache/sim/profiles/<类型>.json`，也可输入其他路径。命令行通过 `--save` 保存最终配置；已有文件不会被覆盖。优先级为：注册表默认值 → JSON 配置 → 显式命令行选项。

```bash
# 保存配置并预览，不启动
./scripts/sim.sh run algorithm --layout lab --view rviz \
  --save .cache/sim/profiles/lab.json --dry-run
# 读取配置，临时改为 Gazebo＋RViz
./scripts/sim.sh run --config .cache/sim/profiles/lab.json --view both
# 只预览菜单选择
./scripts/sim.sh menu --dry-run
```

JSON 保持原格式：`{"schema":1,"profile":"algorithm","options":{"layout":"lab","view":"rviz"}}`。未填写选项采用默认值；未知字段、无效类型、超范围数值与配置类型冲突会被拒绝。`--config`／`--save` 文件位置按调用时的当前目录解析；配置内部的相对路径按项目根目录解析。所有运行器均从项目根目录启动。

## 会话控制与退出

算法建图在另一终端执行：

```bash
./scripts/sim.sh algorithm init
./scripts/sim.sh algorithm status
./scripts/sim.sh algorithm explore
./scripts/sim.sh algorithm cancel
./scripts/sim.sh algorithm save .cache/maps/run1
./scripts/sim.sh algorithm stop
```

`localization` 模式改用 `algorithm load <地图目录>`，地图必须与场景匹配。完整算法控制、地图和初始化行为见[控制手册](SIMULATION_CONTROL.md)。

场景查看在前台按 `Ctrl+C` 退出；PX4 自动检查／任务按原运行器结束并清理；Pocket 会话按[操作手册](../flight/VIO_POCKET_AVOIDANCE.md)在落地未解锁后关闭。`algorithm stop` 只结束受管算法会话。切换场景或定位来源前先结束原会话；总入口沿用原锁和唯一控制者检查，不自动终止其他会话。

### 查找运行器及残留进程

在另一终端列出当前用户的候选进程。首列为 PID，第二列为父 PID，第三列为进程组：

```bash
ps -u "$(id -un)" -o pid,ppid,pgid,etime,args | rg '[u]av_nav_ws|[Q]GroundControl|[g]z sim|[g]z-sim|[r]viz2|[M]icroXRCEAgent|[p]arameter_bridge'
```

这些候选可能包含其他会话，结合完整命令中的工作区、运行目录及父子关系确认。
优先找到 `scripts/simulation/manual.py`、`run_px4_vio_sensors.py`、`run_px4_mtf01.py`
或 `run_px4_sitl_smoke.py` 等顶层运行器。若运行器已退出，则逐一确认残留子进程。

### 正常关闭 Pocket／PX4 会话

先确认飞机已落地并解除武装。VIO Pocket／纯手动 VIO 会话可使用启动日志中的本轮运行目录，
创建关闭请求文件；运行器确认地面状态后清理其拥有的进程：

```bash
read -r -p '输入本轮 VIO 运行目录：' sim_run
[[ -f "$sim_run/manifest.json" ]] && touch -- "$sim_run/close-ui"
```

原启动终端仍在时也可按 `Ctrl+C`。原终端不可用时，对已经确认的运行器发送正常退出信号：

```bash
read -r -p '输入已确认的仿真运行器 PID：' sim_pid
[[ "$sim_pid" =~ ^[1-9][0-9]*$ ]] && kill -INT -- "$sim_pid"
```

等待清理完成后，重新执行上面的进程查询。只关闭 QGC／Gazebo／RViz 窗口不代表整套会话已退出。

### 处理无响应或残留进程

对正常退出后仍存活的目标，重新查询并核对 PID，再请求终止：

```bash
read -r -p '输入已确认的残留进程 PID：' sim_pid
if [[ "$sim_pid" =~ ^[1-9][0-9]*$ ]]; then
  ps -p "$sim_pid" -o pid,ppid,pgid,etime,args
  kill -TERM -- "$sim_pid"
fi
```

仅在 `TERM` 后仍无响应时，重新确认还是同一目标，再单独执行：

```bash
ps -p "$sim_pid" -o pid,ppid,pgid,etime,args
[[ "$sim_pid" =~ ^[1-9][0-9]*$ ]] && kill -KILL -- "$sim_pid"
```

`KILL` 不执行清理，也不自动结束全部子进程；执行后再次查询并逐一检查残留。
进程信号不受 ROS domain 或 Gazebo partition 隔离，按确认的 PID 处理。
算法原始 launch 的详细退出步骤见[停止正在运行的进程](EGO_NVBLOX_GAZEBO.md#停止正在运行的进程)。

## 启动代码分工

```text
scripts/sim.sh                       公共入口，只转发参数
scripts/simulation/
  cli.py                            run / list / algorithm / advanced 路由
  menu.py                           交互输入与预览
  profiles.py                       选项、默认值、校验、JSON 配置
  backends.py                       运行器、固定预设、旧命令映射
  environment.sh                    PX4 / Mission / VIO 环境适配
  policy.py                         全局无 GNSS 约束与旧模式拦截
  manual.py                         Pocket 自动选择及自建虚拟面板生命周期
  launch_scene.sh                    Gazebo 场景启动与原互斥锁
  launch_isaac.sh                    Isaac Sim UI
  compat.sh                         历史脚本共用转发器
scripts/run_*.py                     现有任务、审计和故障运行器
scripts/sim_control.py              受管算法会话控制
scripts/sim_ros_control.py          有界 ROS 操作
scripts/build/                      构建入口
scripts/setup/                      安装与依赖准备
simulation/                         场景、任务、安全区、验收配置与资产
```

菜单、命令行和保存配置共用 `profiles.py`；固定 VIO／Pocket 参数与原生运行器映射集中在 `backends.py`。环境适配通过 `exec` 把信号和退出码交给原运行器。新 PX4／MTF／VIO 运行的 manifest 记录 `launcher_sha256`，覆盖中央启动代码及共享环境包装器。

Python 任务运行器、共享分析模块及其导入路径保留在 `scripts/`，其中部分路径用于运行证据摘要和子进程调用。它们属于执行实现，日常从总入口选择；不为整理目录而复制运行器或修改历史证据。`run_*smoke.py`／`check_*` 协议、定位、故障回归继续由验收工具调用。硬件相机／串口脚本不归入仿真入口；构建和安装继续使用现有目录。

新增日常启动类型时只注册配置和运行器，不再新增平行的 Shell 预设。新配置需要修改的边界为：选项／校验放 `profiles.py`，固定参数／环境选择放 `backends.py`；场景内容放 `simulation/` 或 `uav_bringup/gazebo/`。

## 专项运行器与兼容入口

专项手册的复杂参数可通过高级入口传递，不受菜单选项集合限制；运行器继续执行原有参数组合和飞行准入检查。

```bash
./scripts/sim.sh advanced vio --dry-run --normalize --fuse-pose --scene layered --duration 120
./scripts/sim.sh advanced scene --dry-run arena:=rmuc_2025 launch_mid360:=true
# 将 --help 传给运行器，而非总入口；需对应运行环境已构建
./scripts/sim.sh advanced vio -- --help
```

| 旧命令／脚本 | 统一入口 |
| --- | --- |
| `sim.sh start/init/status/...` | `sim.sh algorithm start/init/status/...` |
| `sim.sh px4-flight`、`run_px4_flight.sh` | `sim.sh advanced flight` |
| `sim.sh px4-observe` | `sim.sh advanced observe` |
| `sim.sh px4-vio-sensors`、`run_px4_vio_sensors.sh` | `sim.sh advanced vio` |
| `sim.sh px4-vio-mtf01` | `sim.sh advanced vio-joint` |
| `sim.sh px4-vio-pocket` | `sim.sh advanced vio-pocket` |
| `sim.sh px4-mtf01` | `sim.sh advanced flow` |
| `sim.sh px4-flow-local`、`px4-flow-pocket` | `sim.sh advanced flow-local/flow-pocket` |
| `sim.sh px4-depth`、`px4-depth-map`、`px4-depth-plan` | `sim.sh advanced depth-camera/depth-map/depth-plan` |
| `sim.sh px4-vision-audit` | `sim.sh advanced vision-audit` |
| `sim.sh pocket-panel` | `sim.sh advanced panel` |
| `simulation/scripts/run_gazebo_harmonic_nav.sh` | `sim.sh advanced scene` |
| `simulation/scripts/run_uav_obstacle_course.sh` | `sim.sh advanced scene arena:=uav_obstacle_course` |
| `simulation/scripts/run_isaac_sim_45_ui.sh` | `sim.sh advanced isaac` |

兼容入口继续校验全局无 GNSS 策略；被停用的旧模式会说明原因。其余旧命令逐项透传原参数，旧 Shell 路径是指向同一 `compat.sh` 的相对符号链接；不再各自维护预设或环境加载。`scripts/sim_menu.py` 同样保留为菜单兼容链接。复制工程时需保留符号链接，目录与链接检查方法见[维护约定](../MAINTENANCE.md)。
