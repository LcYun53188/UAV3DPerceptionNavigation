# EGO + nvblox 仿真运行指引

本入口先验证算法，最后再接飞控。链路为：Gazebo 深度 + 真值 TF → nvblox 三维 TSDF/ESDF → 不可变地图快照 → 官方 EGO A* / rebound B 样条优化 → 独立曲线验收 → Gazebo 速度模型。没有 PX4、硬件相机、VINS 或 cuVSLAM 进程。

日常操作推荐使用 `./scripts/sim.sh start`、`init`、`status`、`stop` 等统一命令，详见 [仿真控制脚本手册](SIMULATION_CONTROL.md)。脚本自动复用启动环境；本文保留原始 launch 和 ROS 命令，手动执行时仍需在各终端设置相同环境，不能与脚本重复启动。

## 构建

现有 ROS Jazzy、Gazebo Harmonic、CUDA 13.2、Isaac/NITROS 依赖沿用工作区环境。当前构建脚本 CUDA 架构为本机 RTX 4070 的 89，其他显卡须调整。源码依赖使用固定版本和 `patches/vendor`，新 EGO 源码只存于 `.deps/ego_planner_src`。

在已配置好的工作区直接构建；首次获取仓库时先按 [安装指南](INSTALLATION.md) 准备 `.venv` 与系统依赖，并恢复子模块补丁：

```bash
git submodule update --init --recursive
./scripts/apply_vendor_patches.sh
```

`CUDA_HOME` 可指定本机 CUDA 路径；GPU 架构目前写在 `scripts/build_algorithm_sim.sh` 的 `-DCMAKE_CUDA_ARCHITECTURES=89`，换显卡时应按实际架构修改。
确认 `gz sim --help` 可运行，且 ROS 环境能找到 `ros_gz_bridge`、`ros_gz_image`、`rviz2`，再执行：

```bash
./scripts/build_algorithm_sim.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./scripts/with_venv.sh python -m pytest -q src/uav_nav_sim/test
```

该脚本只构建算法依赖，bringup 作为资源包单独安装，不要求构建可选硬件定位后端。已有 vendor 补丁的恢复说明见 [补丁清单](../patches/vendor/README.md)。构建不启动仿真、不解锁、不连接飞控。

## 运行前约定

所有命令在项目根目录执行。每个参与仿真的终端都设置：

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
```

同一时间只运行一套仿真；切换模式或场景前，在原 launch 终端按 `Ctrl+C`，等待 Gazebo 及节点退出。
`simulation/scripts/run_uav_obstacle_course.sh` 只启动另一套障碍场地与传感器，不启动本导航链路；不要与本入口同时运行。

下面以约 20 × 20 m 的 `uav_ego_expanded.sdf` 为主流程。它包含中央柱、6 根新增立柱、4 个箱体、2 段隔墙和2 组门框，围墙高 4 m。
`map_extent:=10.5` 表示查询范围为 X/Y 各 -10.5 至 10.5 m、Z 为 0–4 m；该范围参与地图兼容性指纹。

## 启动方式速查

| 目的 | 启动选择 | 启动后的操作 |
| --- | --- | --- |
| 查看场地与传感器 | `./simulation/scripts/run_uav_obstacle_course.sh` | 不包含本导航链路，详见 [障碍场地说明](UAV_OBSTACLE_COURSE.md) |
| 在线建图并探索 | `mode:=mapping` | `survey_gazebo_map.py --layout expanded --local-only`，完成后发送目标 |
| 完整扫描并保存 | `mode:=mapping` | `survey_gazebo_map.py --layout expanded --output <新目录>`，扫描期间不发送目标 |
| 复用地图导航 | `mode:=localization` | `map_bundle load <兼容地图目录>`，等待 `valid=true` 后发送目标 |

导航 launch 只接受 `mapping` 和 `localization` 两个模式；扫描与探索是建图模式下的两种使用流程。完整可复制命令见 [README 启动步骤](../README.md#选择运行方式)。首次局部初始化与完整扫描任选其一，完整扫描不依赖局部初始化。

由建图切换到加载导航时，先保存需要保留的地图，再退出原 launch，使用 `mode:=localization` 重启后加载；从加载模式切回 `mapping` 会开启新的建图会话，当前入口不支持加载旧地图后继续增量建图。只退出 RViz 或调用 `/uav/cancel` 不会切换模式。

## 首次建图（扩展场景）

**终端 A：** 启动建图模式。`gui:=true` 同时打开 Gazebo UI、RViz 和定高选点节点。

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py \
  mode:=mapping gui:=true map_extent:=10.5 \
  world:="$PWD/src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf"
```

**终端 B：** 等待场景和节点启动后扫描地图。输出目录须尚不存在；已有地图可直接跳到下一节加载，或换一个新目录扫描。

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
./scripts/with_venv.sh python scripts/survey_gazebo_map.py \
  --layout expanded --output .cache/maps/uav_ego_expanded
```

扫描覆盖 16 个水平观察点、2 个高度和4 个航向。脚本通过 Gazebo `set_pose` 放置相机，在位姿切换期间关闭深度入口，稳定后再积分渲染深度；扫描时不要发送导航目标。
**这是离线传感器扫描，不是无人机自主探索或飞行轨迹。** 自由空间来自深度积分，不直接从场景几何生成。

看到 `Saved map bundle ...` 表示保存完成。脚本最后回到 `(-7.5,-7.5,1.2)`；地图目录包含 `static_map.nvblx` 和 `manifest.json`，位于 Git 忽略的 `.cache/`，不随仓库分发。
保存清单记录地图 ID、场景/配置哈希、构建版本指纹、坐标约定、体素大小和文件 SHA-256。

也可在已完成建图且执行器处于 `HOLD` 时手动另存：

```bash
./scripts/with_venv.sh ros2 run uav_nav_sim map_bundle save .cache/maps/uav_ego_expanded_copy
```

保存目录必须不存在，保存失败不会覆盖已有地图。

## 在线建图与定向探索

`mode:=mapping` 保持深度积分开启，同时启用目标管理。RViz 或 `/uav/goal` 设置最终目标；执行器内的 `GoalManager` 选择 `/uav/local_goal`，EGO 只规划这一段。目标已经安全可达时直接导航，否则在与起点连通的已知空闲区域选择靠近未知边界的观测位置。临时目标之间要求停稳，不进行运动中的轨迹拼接。

### 从局部地图开始

空地图中的机体附近体积可能处于前向相机盲区。系统不会把这些未知体素当作空闲，也不会盲飞以获得地图。

**终端 A：** 按运行前约定设置环境变量后启动（若同场景建图 launch 已在运行，不要重复启动）：

```bash
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py \
  mode:=mapping gui:=true map_extent:=10.5 goal_height:=1.2 \
  world:="$PWD/src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf"
```

**终端 B：** 使用相同环境变量，等待节点启动后执行局部初始化：

```bash
# 与 launch 保持相同 ROS_DOMAIN_ID；小场景使用 --layout lab
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
./scripts/with_venv.sh python scripts/survey_gazebo_map.py --layout expanded --local-only
```

局部初始化在起点四周 0.8 m 的位置、1.2/1.7 m 两个高度朝向起点采集深度，然后返回起点并检查 0.3 m 查询半径内的完整观测体积。它只适用于仓库中的两个固定 Gazebo 场景，使用 `set_pose` 放置相机，**不属于自主飞行验收**。不传 `--output` 时不保存地图、不结束在线建图。需要归档时可另加新的 `--output` 目录。

初始化成功后发送 RViz 目标即可。目标可以处于未知区域，但轨迹和临时目标必须处于已观测安全空间。每段到达后仅调整偏航，使前向相机朝向目标所在的水平方向；没有候选点时限时转向其他方向扫描。观察、等待规划、平移和任务终止后的 HOLD 都以横滚/俯仰为零为目标，不再通过倾斜整个机身补偿 OAK-D 的 5° 下倾安装角度。回正依赖新鲜、有效的里程计，丢失定位时输出零指令。相机按 5° 下倾安装角度观测，视野不足时可能报告无安全候选点。该控制适用于当前无重力 Gazebo 速度模型，未模拟旋翼推力与真实飞行姿态。固定 ESDF 查询边界之外不作为可飞行空间，需要更大范围时重新设置 `map_extent`。

深度图与 TF 通过独立话题到达。建图输入最多缓存 8 帧，并在图像时间戳后 0.5 秒内等待对应历史 TF；超时帧仍丢弃，关闭输入门控时清空缓存。若报告 `MAP_INVALID` 或 `STALE_MAP_OR_ODOMETRY`，检查建图日志和传感器更新，恢复后重新发送目标。

### 状态与停止条件

```bash
./scripts/with_venv.sh ros2 topic echo /uav/navigation/state \
  --qos-durability transient_local
./scripts/with_venv.sh ros2 service call /uav/cancel std_srvs/srv/Trigger '{}'
```

| 任务状态 | 含义 |
| --- | --- |
| `OBSERVING` | 原地观察或等待安全观测点，可能执行偏航转向 |
| `PLANNING` | 等待当前临时目标的 EGO 结果 |
| `EXPLORING` | 执行通往观测位置的一段轨迹 |
| `NAVIGATING` | 执行通往最终目标的轨迹 |
| `REACHED` | 最终目标到达 |
| `BLOCKED:<原因>` | 无候选点、连续失败或探索预算耗尽；保持停止 |
| `STOPPED:<原因>` | 定位、地图、时钟或跟踪异常；保持停止 |
| `CANCELLED:<原因>` | 用户取消、目标替换或地图会话改变 |

`BLOCKED`、`STOPPED` 和 `CANCELLED` 不会因后续地图更新自动恢复，需要重新发送目标。任务状态以 `/uav/navigation/state` 为准；`/uav/planner/state` 只表示单段规划结果。局部到达事件为 `LOCAL_GOAL_REACHED`，不会清除最终目标。

最终目标已经被观测为实体障碍时，直接进入 `BLOCKED:KNOWN_GOAL_OCCUPIED`，不再围绕该点继续探索。新地图使剩余轨迹不安全时，执行器立即停止，再从停稳状态进行有限重规划。机体安全体积、整条样条和动态限制仍使用原来的保守验证。每段目标的 `header.stamp` 作为关联标识，由 `PlannerStatus` 和 `TimedTrajectory.goal_stamp` 回传；取消、替换和规划超时会使旧标识失效，延迟轨迹无法重新启动飞行。目标管理与速度执行在同一个单线程节点内串行处理。

默认候选点距离上限为 2 m（欧氏距离，绕行轨迹长度可能更长），允许比历史最近目标距离多绕 2 m；排除 0.45 m 范围内已访问或失败的观测点。无候选点持续 15 s、连续 4 次规划/接管失败、60 s 未接近目标至少 0.2 m，或完成 40 个探索段，会结束任务。单段规划等待上限 8 s；这些时间预算采用单调墙钟，在停稳后的任务决策阶段检查；执行中的轨迹仍由地图与跟踪安全检查控制。它们限制本地搜索，不是全局不可达证明。

启动参数 `exploration_step`、`blocked_timeout` 分别控制候选距离与无候选点等待。其他配置在执行器的 `exploration.*` 参数中声明，启动时读取；修改 YAML/launch 后重启节点生效。`mode:=localization` 关闭未知区域探索和观察转向，仍经过同一个取消与目标关联机制。

### 小场景探索验收

仅适用于默认 `uav_ego_lab` 的真实障碍几何，先完成 `--layout lab --local-only` 初始化：

```bash
./scripts/with_venv.sh python scripts/check_gazebo_exploration.py \
  --goal 3 0 1.2 --expect REACHED --min-segments 2 \
  --output /tmp/uav_exploration_reached.json
# 不可达目标位于中央实体柱内；会实际发送新目标
./scripts/with_venv.sh python scripts/check_gazebo_exploration.py \
  --goal 0 0 1.2 --expect BLOCKED --min-segments 0 \
  --output /tmp/uav_exploration_blocked.json
```

本次小场景验收记录见 [gazebo_exploration.json](validation/gazebo_exploration.json)：未知目标经 3 段到达，实际轨迹最小真实障碍余量约 0.206 m；已知柱内目标保持停止。该记录不代表实机或扩展场景验收。

2026-09-30 的 [扩展场景深度同步验证](validation/gazebo_depth_sync.json) 已观察到分段位移且未出现地图源过期，但最终目标因 `EXPLORATION_BUDGET` 停止。扩展场景目标可靠到达仍未完成验收。

检查器记录初始目标观测状态、已观测体素变化、接受段数、任务状态和独立场景几何间距；终止后继续观察 3 s，检查未重新出发。若目标区域已因之前的测试被观测，不能把这次运行视作未知目标探索覆盖，应重启建图并局部初始化。脚本超时或退出时会取消任务。

## 加载地图并导航（扩展场景）

扫描保存完成后，先停止终端 A 中的建图 launch，等待退出，再启动加载模式。
已创建兼容地图时，每次运行从这里开始即可。

**终端 A：**

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py \
  mode:=localization gui:=true map_extent:=10.5 goal_height:=1.2 \
  world:="$PWD/src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf"
```

**终端 B：** 下面的 `.cache/maps/uav_ego_expanded` 是上一节扫描创建的目录。先用 `ls .cache/maps` 核对；若复用本机已有地图，应将加载路径改为 `.cache/maps/uav_ego_expanded_20260926`。

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
./scripts/with_venv.sh ros2 run uav_nav_sim map_bundle load .cache/maps/uav_ego_expanded
./scripts/with_venv.sh ros2 topic echo /uav/map/snapshot --once --field valid \
  --qos-durability transient_local
```

看到 `load: ...` 说明地图加载服务成功；等待地图快照 `valid` 为 `true` 后再发目标。若首次输出为 `false`，稍后重新检查。
确认 RViz 的 Fixed Frame 为 `map`，可看到网格后按下文选点。模型初始位姿为 `(-7.5,-7.5,1.2)`。

地图路径可替换为自己的保存目录。本机已有 `.cache/maps/uav_ego_expanded_20260926`，这是本地产物，其他机器需先扫描。
加载时必须使用与建图相同的 world、`map_extent` 及兼容配置；不匹配会拒绝加载。

`localization` 模式不积分新深度，使用 Gazebo 真值定位在同一个静态世界内复用地图，不包含现实环境重定位。
每次加载递增 epoch 并使旧目标失效；有效新 ESDF 到达前不接受轨迹。重新加载地图前先取消目标并等待执行器回到 `HOLD`。

### 启动参数

| 参数 | 默认值 | 用途 |
| --- | --- | --- |
| `mode` | `mapping` | `mapping` 积分深度；`localization` 等待加载地图 |
| `world` | `uav_ego_lab.sdf` | 扩展场景必须显式传入 `uav_ego_expanded.sdf` |
| `map_extent` | `5.0` | 地图 X/Y 查询半宽，扩展场景用 `10.5` |
| `gui` | `false` | 是否打开 Gazebo 图形界面 |
| `launch_rviz` | 跟随 `gui` | 是否启动 RViz 和定高选点节点 |
| `goal_height` | `1.2` | RViz 选点的目标 Z，高度单位为米 |
| `exploration_step` | `2.0` | 探索候选点距当前点的最大欧氏距离（m） |
| `blocked_timeout` | `15.0` | 没有安全候选点时允许观察的时间（s） |

只打开 RViz、隐藏 Gazebo 界面时使用 `gui:=false launch_rviz:=true`；两者均隐藏时用 `gui:=false launch_rviz:=false`。
无界面仿真仍需要 GPU 渲染深度图像。

### RViz 地图着色

导航配置默认使用 `3D nvblox Map → Mesh Color: Height`，按地图坐标系 Z 高度在
0–4 m 之间从蓝色经过青、绿、黄渐变到红色。可通过 `Min Height` / `Max Height`
调整范围（上限必须大于下限）；超出范围使用端点颜色。这是高度显示，不表示障碍风险。

`Normals` 按表面朝向着色，便于区分地面与不同方向的墙面；`RGB` 使用地图自带颜色。
当前深度建图设置 `use_color: false`，已有地图没有融合相机颜色，因此 RGB 模式仍可能灰色。
高度/朝向着色不需要重新建图，不修改 TSDF、ESDF 或规划结果；切换模式会重绘已缓存的地图。
`Cut Ceiling` 和 `Ceiling Height` 仍可隐藏高处网格（按网格块裁切）。

更新插件后需要重启 RViz 才能载入新的动态库；仅重新加载配置不足以更新插件。
单独构建显示插件：

```bash
./scripts/with_venv.sh colcon build --build-base build_uav --install-base install_uav \
  --packages-select nvblox_rviz_plugin --symlink-install --cmake-args -DBUILD_TESTING=OFF
```

### RViz 定高选点

仿真 `gui:=true` 时默认同时启动 RViz 和选点转换节点；可用 `launch_rviz:=false` 关闭，或用 `goal_height:=1.2` 设置初始目标高度。
已有仿真以 `launch_rviz:=false` 启动时，可在同域终端单独启动 UI（同一 ROS_DOMAIN_ID 下只运行一个选点转换节点）：

```bash
./scripts/with_venv.sh ros2 launch uav_bringup rviz_navigation.launch.py goal_height:=1.2
```

点击顶部 **2D Goal Pose**（快捷键 G），在地图上按下鼠标并拖动方向，松开后立即发送导航目标。
RViz Fixed Frame 须为 `map`。工具发送 `/uav/rviz/goal_2d`，转换节点设置目标 Z 后发布到 `/uav/goal`；橙色箭头表示已发送目标，Planned Path 显示规划路径。
该高度相对 map 的 Z=0，不是地形上方的实时离地高度；只固定目标高度，三维规划路径仍可能升降。目前 EGO 使用位置目标，拖动的朝向不约束最终机头方向。

运行中修改后续目标高度（不会重新发送已有目标）：

```bash
./scripts/with_venv.sh ros2 param set /rviz_fixed_height_goal height 1.8
```

选点后执行器可开始观察或飞行。`mapping` 模式允许未知目标，并通过已观测安全区域内的临时目标逐段接近；`localization` 模式只在已有地图内导航。已知障碍目标会受阻停止，任何轨迹都不能穿越未知区域或地图边界。

执行中更换目标会停止当前轨迹，使旧目标失效，再从停稳状态处理新目标。有效新地图使剩余曲线不安全时，可进行有限重规划；地图失效、定位失联等进入 `STOPPED`，不会在数据恢复后自动重启。`BLOCKED`、`STOPPED`、地图会话改变或用户取消后，都需要重新提交目标。停止运动使用下文的 `/uav/cancel` 服务。

## 发送目标、检查状态与停止

除了 RViz，也可直接向 `/uav/goal` 发布三维位置；下面是扩展场景的一个已验证目标，命令会启动仿真运动：

```bash
./scripts/with_venv.sh ros2 topic pub --once /uav/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 7.99, y: 4.25, z: 1.2}, orientation: {w: 1.0}}}'
```

在同域终端查看状态（每条 `echo` 持续输出，用 `Ctrl+C` 结束查看）：

```bash
./scripts/with_venv.sh ros2 topic echo /uav/navigation/state --qos-durability transient_local
./scripts/with_venv.sh ros2 topic echo /uav/planner/state
./scripts/with_venv.sh ros2 topic echo /uav/executor/event
./scripts/with_venv.sh ros2 topic echo /uav/executor/state
```

整体结果以 `/uav/navigation/state` 为准。`ACCEPTED` 表示执行器接受一段轨迹，`LOCAL_GOAL_REACHED` 只表示临时目标到达，`GOAL_REACHED` 表示最终段到达；`SAFE_SEED_FALLBACK` 表示使用通过碰撞验收的种子回退路线，可能在航点停顿。
`BLOCKED_START_OR_GOAL` 表示起点或目标无法通过通行检查，应检查障碍物、地图边界与观测覆盖。

取消当前目标，等待停止：

```bash
./scripts/with_venv.sh ros2 service call /uav/cancel std_srvs/srv/Trigger '{}'
./scripts/with_venv.sh ros2 topic echo /uav/executor/state --once
```

确认 `HOLD` 后可以重新加载地图或运行离线扫描。退出整套仿真时，在 launch 终端按 `Ctrl+C`；单独启动的 RViz launch 也需退出。

## 停止正在运行的进程

### 正常退出

`/uav/cancel` 只让执行器停止当前目标并回到 `HOLD`，不会退出任何进程。
若扫描脚本或自动目标检查还在运行，先在它们的终端按 `Ctrl+C`，再取消目标，避免继续移动扫描相机或发送目标。

在启动 `ros2 launch` 或 `run_uav_obstacle_course.sh` 的终端按 **Ctrl+C**，让 launch 通知并清理子进程。
等待终端返回 shell 提示符；单独运行的 `rviz_navigation.launch.py`、`rviz2` 和 `ros2 topic echo` 在各自终端退出。
关闭 Gazebo / RViz 窗口不能作为整套节点已退出的判断，应检查 launch 终端与进程列表。

如果曾按 `Ctrl+Z`，进程只是暂停，并未退出。在原终端执行 `jobs -l` 查看任务，再用 `fg %1` 恢复对应任务后按 `Ctrl+C`（`%1` 替换为实际任务号）。

### 原终端已关闭或后台运行

先查找本项目的 launch 进程。输出首列是 PID，其后是完整命令行：

```bash
pgrep -af '[r]os2 launch uav_bringup (uav_ego_nvblox|gazebo_harmonic_nav|rviz_navigation)\.launch\.py'
```

确认场景、配置路径属于要停止的那次仿真。多条结果应逐一核对；系统进程信号不受 `ROS_DOMAIN_ID` 或 `GZ_PARTITION` 隔离。
用实际 PID 替换 `12345`，查看进程及其直接子进程：

```bash
SIM_PID=12345  # 替换为待停止的 launch PID，不要直接使用示例数字
ps -p "$SIM_PID" -o pid,ppid,stat,args
pgrep -a -P "$SIM_PID"
```

确认后先向 launch 发送 `SIGINT`，相当于请求正常退出，并给它时间清理子进程：

```bash
kill -INT -- "$SIM_PID"
```

等待数秒后检查：

```bash
ps -p "$SIM_PID" -o pid,ppid,stat,args
```

没有对应进程行说明该 PID 已退出；再按下一节检查是否有残留。若提示 `No such process`，说明进程已先行退出，无需重复发送信号。

### 检查残留与处理无响应进程

下面只列出候选进程，不会停止它们。Gazebo 子进程可能显示为 `gz sim`、`gz-sim-server` 或 `gz-sim-gui`：

```bash
ps -eo pid,ppid,stat,args | rg '[g]z sim|[g]z-sim|[r]viz2|[n]vblox_node|[e]go_nvblox_planner|[m]ap_session|[g]azebo_executor|[r]viz_goal|[p]arameter_bridge|[i]mage_bridge|[s]tatic_transform_publisher|[g]azebo_odometry_velocity|[s]urvey_gazebo_map|[c]heck_gazebo_'
```

这些名称也可能属于其他项目，须结合完整命令行、工作区路径和上一步记录的父子进程关系确认。
对于原 launch 已退出但仍存活的仿真子进程，重新设置 `SIM_PID` 为该子进程 PID，先尝试 `kill -INT`。

如果已等待正常退出但进程仍无响应，重新核对 PID 后再逐级处理；下面两组命令按需分别执行，不要连续直接粘贴：

```bash
# 第一步：请求终止，随后等待并检查是否退出
ps -p "$SIM_PID" -o pid,ppid,stat,args
kill -TERM -- "$SIM_PID"
```

```bash
# 仅在 TERM 后仍无响应，且再次确认仍是同一个目标进程时执行
ps -p "$SIM_PID" -o pid,ppid,stat,args
kill -KILL -- "$SIM_PID"
```

`SIGKILL` 无法执行正常清理或保存未保存地图，也不会自动清理该进程的所有子进程；之后仍需检查残留。
按 PID 定点处理，不使用 `killall python3`、`pkill -f ros` 或 `killall gz` 等可能终止其他任务的宽泛命令。

确认本次仿真的 launch、Gazebo 及相关节点均已退出后再启动下一次实验。
若场景脚本仍提示 `A UAV Gazebo simulation is already running`，检查是否还有进程持有仿真锁；锁文件本身留在 `/tmp` 是正常的，无需删除它或绕过单实例检查。

## 小场景与自动验证

小场景使用默认 world 和 `map_extent:=5.0`，与扩展场景二选一运行。各终端仍需设置上文的域和分区。

```bash
# 终端 A：建图
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py mode:=mapping gui:=true
# 终端 B：默认 lab 扫描布局
./scripts/with_venv.sh python scripts/survey_gazebo_map.py --layout lab --output .cache/maps/room_a
```

扫描覆盖 8 个水平观察点、2 个高度和4 个航向，完成后回到 `(-3,0,1.2)`。
停止建图 launch，再分别执行：

```bash
# 终端 A：加载模式
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py mode:=localization gui:=true
# 终端 B：加载后进行往返验证；每条检查都会发送目标并等待结果
./scripts/with_venv.sh ros2 run uav_nav_sim map_bundle load .cache/maps/room_a
./scripts/with_venv.sh python scripts/check_gazebo_goal.py --goal 3 0 1.2 --output /tmp/ego_forward.json
./scripts/with_venv.sh python scripts/check_gazebo_goal.py --goal -3 0 1.2 --output /tmp/ego_return.json
```

扩展场景使用专用验证脚本。在已加载扩展地图、模型位于初始位置且定高选点节点高度为 1.2 m 时运行：

```bash
./scripts/with_venv.sh python scripts/check_gazebo_expanded_goal.py \
  --goal 7.99 4.25 --timeout 240 --output /tmp/expanded_navigation.json
```

该脚本通过 RViz 同一输入话题发送目标，检查轨迹、到达误差和场景几何间距，因此必须启动定高选点节点。
小场景、扩展场景的扫描和验证脚本不能混用；自定义世界需另行定义扫描位姿与验收几何。

## 常见问题

| 现象 | 检查与处理 |
| --- | --- |
| 运行障碍场地脚本后没有规划路径 | 该脚本只启动场景与传感器；导航使用本文 `uav_ego_nvblox.launch.py` 入口 |
| 地图保存提示目录已存在 | 使用新的输出目录；不要覆盖需要复用的地图 |
| 加载报文件不存在或 `Map operation failed or timed out` | 先用 `ls .cache/maps` 检查路径，并确认目录内有 `manifest.json` 和 `static_map.nvblx`；本机已有扩展地图目录带 `_20260926` 后缀。目录存在仍失败时，查看启动终端的 `Load rejected:` 日志 |
| 地图加载被拒绝 | 确认 `mode:=localization`、执行器为 `HOLD`，world、map_extent、模型/建图配置和构建指纹兼容 |
| RViz 重开后地图为空 | 先核对 ROS_DOMAIN_ID、Fixed Frame=`map`、话题 `/nvblox_node/mesh`；若加载模式下未重发网格，取消目标、等待 HOLD 后重新加载原地图 |
| 地图仍为灰色，或没有 Mesh Color 选项 | 构建新版 `nvblox_rviz_plugin` 并重启 RViz，加载 `uav_navigation.rviz`，选择 Height 或 Normals；旧地图 RGB 未融合颜色 |
| 地图上方被截断 | 检查 `Cut Ceiling` / `Ceiling Height`，默认 3.8 m；裁切以网格块为单位 |
| RViz 选点后不运动 | 检查地图 `valid`、`/uav/navigation/state`、起点安全体积是否已观测，以及 `/rviz_fixed_height_goal` 是否运行；同一域仅保留一个定高选点节点 |
| TF 抖动或模型跳变 | 确认旧 Gazebo、旧 launch 已退出，所有终端域与分区一致 |
| `gui:=false` 时看不到 RViz | 默认 launch_rviz 跟随 gui，显式加 `launch_rviz:=true` |

若只重开 RViz 窗口且原定高选点节点仍在运行，使用以下命令，避免重复启动转换节点：

```bash
./scripts/with_venv.sh rviz2 -d "$PWD/src/uav_bringup/rviz/uav_navigation.rviz" \
  --ros-args -p use_sim_time:=true
```

## 数据与实现边界

| 接口 | 作用 |
| --- | --- |
| `/uav/localization/odometry` | Gazebo 真值 odom/FLU 机体速度 |
| `/uav/map/snapshot` | map 坐标、epoch/version、来源时间、距离与 observed 数组 |
| `/uav/trajectory` | odom 坐标下的均匀三次 B 样条控制点、时间间隔、绝对起点和动态限制 |
| `/uav/goal`、`/uav/local_goal` | 最终目标与内部临时目标，均为 map 坐标 |
| `/uav/navigation/state` | 整体任务状态，含锁定的受阻、异常与取消结果 |
| `/uav/planner/result` | 带局部目标关联标识的单段规划结果 |
| `/uav/planned_path` | 仅可视化 |
| `/uav/planner/state` | 目标阻塞、无路、时效失败或轨迹发布状态 |
| `/uav/executor/state`、`event` | HOLD/EXECUTING，以及拒绝、取消、失效和到达事件 |
| `/uav/map/save`、`load` | `nvblox_msgs/FilePath` 地图包服务 |
| `/uav/map/input_enabled` | 离线扫描使用的深度输入门控，仅在 HOLD 的建图模式允许 |

EGO 的传感器积分、上游 FSM 和控制输出均不启用。所有实际构建的 EGO 地图查询由同一冻结快照提供。碰撞判定保留未知、地图外和距离无效状态，机体已观测体积检查使用三维前缀和加速；A* 连线也检查体积。

完整样条独立使用 SciPy 验证。速度、加速度和 jerk 用导数控制点的凸包界约束；沿曲线采样时根据全局速度界增加覆盖半径，避免只检查控制点或稀疏点。起终点三个控制点重合，确保静止接管和终端停止。执行器按 ROS 绝对时间采样，以单调时间检查消息失联；同会话旧轨迹 ID、旧 epoch、错误坐标、过晚起点均拒绝。

默认体素 0.1 m、机体查询半径 0.3 m、速度 0.5 m/s、加速度 1 m/s²、jerk 2 m/s³。ESDF/消息阈值是当前算法实验参数，不是实机误差预算。observed 来自本版本 ESDF，有效不等于逐体素近期观测；静态地图验证不声称解决动态障碍历史时效。

Gazebo 使用无重力的直接速度模型，输出转换到机体系；坐标语义核对依据为 [Gazebo LinearVelocityCmd 源码](https://github.com/gazebosim/gz-sim/blob/gz-sim8/include/gz/sim/components/LinearVelocityCmd.hh)。零速度停止依赖该仿真模型，不能作为真实无人机制动能力。控制进程异常退出后的物理制动、运动中连续轨迹接管、真实定位和飞控接入均在后续阶段完成。

`check_gazebo_goal.py` 的独立几何判据只适用于默认小场景；扩展场景使用 `check_gazebo_expanded_goal.py`。更换 `world` 后必须同时替换验收几何，不能沿用其他场景的脚本声称通过。

## 本机验证记录

2026-09-26 的跨进程地图加载、重复加载及绕柱结果见 [验证报告](validation/EGO_NVBLOX_GAZEBO_20260926.md)，其中保存小型 JSON 结果和复现命令。当前已创建的本地地图为 `.cache/maps/uav_ego_lab_verified`；地图不随 Git 分发，其他机器应按上文重新扫描创建。

2026-09-27 的扩展场景目标、回退路线与障碍物目标拒绝结果见 [扩展场景验证报告](validation/EGO_EXPANDED_NAVIGATION_20260927.md)。这些是已有验证记录，修改地图、场景或算法后应重新执行对应检查。
