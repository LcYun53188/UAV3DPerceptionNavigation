# EGO + nvblox Gazebo 算法链路

本入口先验证算法，最后再接飞控。链路为：Gazebo 深度 + 真值 TF → nvblox 三维 TSDF/ESDF → 不可变地图快照 → 官方 EGO A* / rebound B 样条优化 → 独立曲线验收 → Gazebo 速度模型。没有 PX4、硬件相机、VINS 或 cuVSLAM 进程。

## 构建

现有 ROS Jazzy、Gazebo Harmonic、CUDA 13.2、Isaac/NITROS 依赖沿用工作区环境。当前构建脚本 CUDA 架构为本机 RTX 4070 的 89，其他显卡须调整。源码依赖使用固定版本和 `patches/vendor`，新 EGO 源码只存于 `.deps/ego_planner_src`。

```bash
./scripts/build_algorithm_sim.sh
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./scripts/with_venv.sh python -m pytest -q src/uav_nav_sim/test
```

该脚本只构建算法依赖，bringup 作为资源包单独安装，不要求构建可选硬件定位后端。已有 vendor 补丁的恢复说明见 [补丁清单](../patches/vendor/README.md)。构建不启动仿真、不解锁、不连接飞控。

## 创建离线地图

每个参与同一仿真的终端使用相同的域和分区；开始新实验前停止旧实验。

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py mode:=mapping gui:=false
```

另一个终端设置相同环境变量，然后创建地图：

```bash
./scripts/with_venv.sh python scripts/survey_gazebo_map.py --output .cache/maps/room_a
```

扫描脚本仅适用于默认 `uav_ego_lab` 测试场景。它在 8 个水平观察点、2 个高度、4 个航向获取深度，通过 Gazebo `set_pose` 放置相机；位姿切换期间关闭深度入口，等待稳定后恢复。**这是离线传感器扫描，不是无人机自主探索或飞行轨迹。** 地图来自真实 Gazebo 渲染深度经 nvblox 积分，不从场景几何直接生成自由空间。

脚本最后回到 `(-3,0,1.2)` 并保存地图包。也可在已有地图且执行器处于 HOLD 时手动保存：

```bash
./scripts/with_venv.sh ros2 run uav_nav_sim map_bundle save .cache/maps/room_b
```

目录须尚不存在；保存失败不覆盖已有地图。文件为 `static_map.nvblx` 和 `manifest.json`，包含地图 ID、场景/配置哈希、构建版本指纹、坐标约定、体素大小和文件 SHA-256。地图与大文件位于 Git 忽略的 `.cache/`。

## 加载地图并避障

### 扩大场景与障碍物

`uav_ego_expanded.sdf` 提供约 20 × 20 m、围墙高 4 m 的场地：中央柱、6 根新增立柱、4 个不同高度箱体、2 段隔墙和2 组门框。原小场景保留。
新场景必须重新扫描，使用 21 × 21 × 4 m ESDF 查询范围；范围也参与地图兼容性指纹。

```bash
export ROS_DOMAIN_ID=68
export GZ_PARTITION=uav_ego_lab
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py \
  mode:=mapping gui:=true map_extent:=10.5 \
  world:="$PWD/src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf"
# 另一个同域终端，输出目录须不存在：
./scripts/with_venv.sh python scripts/survey_gazebo_map.py \
  --layout expanded --output .cache/maps/uav_ego_expanded
```

扫描覆盖 16 个水平观察点、2 个高度和4 个航向，结束后回到 `(-7.5,-7.5,1.2)`。
加载该地图时仍须指定同一 world 和 `map_extent:=10.5`。
下文原场景的绕柱几何验收脚本不适用于这个扩展场景。

### 原场景加载验证

停止建图 launch，等待 Gazebo 进程退出，再启动：

```bash
./scripts/with_venv.sh ros2 launch uav_bringup uav_ego_nvblox.launch.py mode:=localization gui:=false
```

另一个终端：

```bash
./scripts/with_venv.sh ros2 run uav_nav_sim map_bundle load .cache/maps/room_a
./scripts/with_venv.sh python scripts/check_gazebo_goal.py --goal 3 0 1.2 --output /tmp/ego_forward.json
./scripts/with_venv.sh python scripts/check_gazebo_goal.py --goal -3 0 1.2 --output /tmp/ego_return.json
```

加载模式不积分新的深度。它只用于相同静态 Gazebo 世界中的离线地图验证，不代表现实环境重定位或动态变化已经处理。地图、场景、标定配置或构建指纹不一致会拒绝加载。每次加载递增 epoch，清空版本和旧目标；有效新 ESDF 到达前不接受轨迹。

普通目标接口为 `/uav/goal`，类型 `geometry_msgs/PoseStamped`，`frame_id=map`。例如：

```bash
./scripts/with_venv.sh ros2 topic pub --once /uav/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 3.0, y: 0.0, z: 1.2}, orientation: {w: 1.0}}}'
./scripts/with_venv.sh ros2 service call /uav/cancel std_srvs/srv/Trigger '{}'
```

### RViz 定高选点

仿真 `gui:=true` 时默认同时启动 RViz 和选点转换节点；可用 `launch_rviz:=false` 关闭，或用 `goal_height:=1.2` 设置初始目标高度。
已有仿真运行时，可单独启动 UI（同一 ROS_DOMAIN_ID 下只运行一个选点转换节点）：

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

选点后执行器可立即开始运动。地图外、障碍物内或未观测的目标仍由规划器拒绝；停止运动使用上面的 `/uav/cancel` 服务。

建图模式也可发送目标，但必须先观测起点机体体积和路线；未知区不会自动清空。执行中更换目标会等待当前轨迹结束或取消后停止，再从静止状态规划。地图失效/剩余曲线复检失败会停止仿真运动，在同一地图会话恢复后从停止状态重新规划；跨会话必须重新提交目标。

## 数据与实现边界

| 接口 | 作用 |
| --- | --- |
| `/uav/localization/odometry` | Gazebo 真值 odom/FLU 机体速度 |
| `/uav/map/snapshot` | map 坐标、epoch/version、来源时间、距离与 observed 数组 |
| `/uav/trajectory` | odom 坐标下的均匀三次 B 样条控制点、时间间隔、绝对起点和动态限制 |
| `/uav/planned_path` | 仅可视化 |
| `/uav/planner/state` | 目标阻塞、无路、时效失败或轨迹发布状态 |
| `/uav/executor/state`、`event` | HOLD/EXECUTING，以及拒绝、取消、失效和到达事件 |
| `/uav/map/save`、`load` | `nvblox_msgs/FilePath` 地图包服务 |
| `/uav/map/input_enabled` | 离线扫描使用的深度输入门控，仅在 HOLD 的建图模式允许 |

EGO 的传感器积分、上游 FSM 和控制输出均不启用。所有实际构建的 EGO 地图查询由同一冻结快照提供。碰撞判定保留未知、地图外和距离无效状态，机体已观测体积检查使用三维前缀和加速；A* 连线也检查体积。

完整样条独立使用 SciPy 验证。速度、加速度和 jerk 用导数控制点的凸包界约束；沿曲线采样时根据全局速度界增加覆盖半径，避免只检查控制点或稀疏点。起终点三个控制点重合，确保静止接管和终端停止。执行器按 ROS 绝对时间采样，以单调时间检查消息失联；同会话旧轨迹 ID、旧 epoch、错误坐标、过晚起点均拒绝。

默认体素 0.1 m、机体查询半径 0.3 m、速度 0.5 m/s、加速度 1 m/s²、jerk 2 m/s³。ESDF/消息阈值是当前算法实验参数，不是实机误差预算。observed 来自本版本 ESDF，有效不等于逐体素近期观测；静态地图验证不声称解决动态障碍历史时效。

Gazebo 使用无重力的直接速度模型，输出转换到机体系；坐标语义核对依据为 [Gazebo LinearVelocityCmd 源码](https://github.com/gazebosim/gz-sim/blob/gz-sim8/include/gz/sim/components/LinearVelocityCmd.hh)。零速度停止依赖该仿真模型，不能作为真实无人机制动能力。控制进程异常退出后的物理制动、运动中连续轨迹接管、真实定位和飞控接入均在后续阶段完成。

验证脚本的独立几何判据只适用于默认场景，使用中央柱、围墙和地面真实几何检查机体包络间距；更换 `world` 后必须同时替换验收几何，不能继续使用该脚本声称新场景通过。

## 本机验证记录

2026-09-26 的跨进程地图加载、重复加载及绕柱结果见 [验证报告](validation/EGO_NVBLOX_GAZEBO_20260926.md)，其中保存小型 JSON 结果和复现命令。当前已创建的本地地图为 `.cache/maps/uav_ego_lab_verified`；地图不随 Git 分发，其他机器应按上文重新扫描创建。
