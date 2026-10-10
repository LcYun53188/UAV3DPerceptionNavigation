# 带坐标与会话绑定的 EGO 影子规划

当前新增链路把非 identity PX4 局部 ENU 里程计变换到 map，再调用实际 EGO 规划器。
默认输出仍用于影子规划，PlanningContext 节点不发布 FMU 或 cmd_vel。
FlightServer 已接入显式 EGO 执行准入／采样分支（导航和返航），需要受管根任务授权
及实时地图／对齐／唯一规划源；真实曲线飞行与同机感知导航尚未验收。见
[执行接口与前置验证](validation/simulation/2026-10-09-ego-px4-execution/REPORT.md)。

## 接口与坐标

| 输入/输出 | 类型 | 作用 |
| --- | --- | --- |
| `/planning/source/map` | MapSnapshot | 有效 map 网格和 map_id/epoch/version |
| `/planning/source/odometry` | LocalizedOdometry | 原始 odom/base_link 位姿、定位 session、六个 reset 计数 |
| `/planning/source/alignment` | LocalizationAlignment | 外部已验证的 map ← odom 刚体变换及其世代、地图/定位绑定 |
| `/planning/source/goal` | PoseStamped | map 中的静止起点目标，时间戳不可重用 |
| `/planning/context` | PlanningContext | 当前会话、版本、有效性及拒绝原因 |
| `/planning/bound_trajectory` | ContextTrajectory | 绑定 context 的、再次通过独立校验的 map 轨迹 |

PX4 网关新增 `/uav/px4/localized_odometry`，session 使用本次网关实例 UUID，reset 计数
顺序为 xy/z/vxy/vz/heading/VehicleOdometry。包裹消息与内部 Odometry 的 header 必须
一致。既有 `/uav/px4/odometry` 与 TF 保留；定位重置锁存后所有这些动态位姿发布停止。

对齐必须由明确的外部来源提供，不从 Gazebo 真值或 topic 名称推断。对齐消息的 map_id/
map_epoch、localization_session、reset 计数必须与地图及每条里程计一致。更改变换、定位
session、reset 或 alignment_id 必须增加 generation；同世代更改锁存冲突，只有明确的新
世代可以重新建立上下文。观测到里程计 session/reset 不匹配也会使当前对齐世代
失效，即使后续字段恢复一致仍须新世代重新确认。较旧世代、地图 epoch/版本回退和重放不能刷新接收活性。

变换同时作用于位置、姿态和完整 6×6 pose 协方差，包括交叉项；ROS Odometry 的 twist
仍在 child FLU，线/角速度及 twist 协方差不随世界坐标系改变。拒绝未知帧、非有限值、零
四元数和非对称/非半正定协方差。这里的对齐在每个世代固定，不支持运动中的连续 TF 校正。

EGO 新参数 `coordinate_frame:=map` 要求 map/base_link 里程计，并输出 map 样条；该模式
不能只改输入 frame_id。默认 `coordinate_frame:=odom` 保留原 algorithm identity 模式，
明确要求 map=odom。规划器、网格和目标在同一数值坐标系进行碰撞检查。

## 门控与失效

地图接收/源时间门限 2 s，里程计与对齐 0.5 s，未来容忍 0.05 s；已观测 ROS 时钟停滞
超过 0.5 s 或回跳锁存，需要新节点实例恢复。三个输入各要求唯一发布者。下游不能仅凭
一次 valid=true 永久相信一个 context，必须继续监视上下文的新鲜度和失效事件。

目标只允许静止起点（速度 ≤0.05 m/s）；本批不实现移动 handover。一次目标绑定 context
及准入时地图版本，原始轨迹必须匹配该目标时间戳、map_id/epoch，版本不得早于准入版本
或晚于当前版本，规划结果预算 2 s，计划起点须在未来 0.05–2 s。每目标只绑定一个结果。

独立使用现有 Python 曲线校验器复检最新网格的未知空间、占用和整个扫掠包络，以及
速度/加速度/jerk、终端静止、起终点一致性；不能用 EGO 发布成功代替校验。地图正常版本
更新不改变 context_id，但结果仍对最新地图复检。地图 epoch、定位/对齐世代变化、数据
失效或多个输入发布者会撤销当前目标，迟到结果不能重新绑定。context_id 是内容摘要，
不是安全授权或网络认证凭证。

## 构建与运行

```bash
./scripts/with_venv.sh colcon build --build-base build_uav --install-base install_uav \
  --symlink-install --base-paths src/uav_nav_interfaces src/uav_nav_sim src/uav_ego_adapter \
  --packages-select uav_nav_interfaces uav_nav_sim uav_ego_adapter --parallel-workers 1
./scripts/with_venv.sh python scripts/run_planning_context_smoke.py
```

回归脚本使用隔离 ROS domain 91、系统时间、合成 ESDF 与显式对齐，启动实际 C++ EGO
及 Python bridge，检查绕障轨迹、会话故障与原默认 identity 模式，结束后只清理自己的
进程。不连接实机或重置原 domain 68 的地图/场景。使用合成网格不代表完成 nvblox/PX4
深度传感器链路验收。证据见 [本批报告](validation/simulation/2026-10-08-planning-context/REPORT.md)。

真正 EGO/PX4 执行还需要：S5 深度/地图输入、已验证的地图对齐来源、地图包 schema 2、
父任务/ControlSession 授权绑定与 map 样条到 PX4 local 采样执行器已实现，
仍需完整受管源生命周期、真实跟踪和碰撞/制动包线验收。默认影子输出不自动获得飞行权限。


## 受管同机在线源

`sim.sh px4-depth-plan --duration 40` 使用实际深度／nvblox 地图和 PX4 EKF
localized odometry，启动 `planning_sources`、PlanningContext 和 EGO shadow 图。
`planning_sources` 的 session 与 alignment_id 必须显式配置；变换由参数
`map_translation`／`map_yaw_rad` 给定，必须与积分深度的 TF 完全一致。
在线地图原样转发，alignment 心跳不改变地图或 odometry 的时间戳。
首次有效绑定 generation=1，此前为 0；重置、地图 session 变化、版本／源时间回退、
源 GID 变化、重复或丢失会退役绑定，不能在旧地图上自动重绑新定位源。
数据过期使 alignment/context 无效；恢复新鲜度不能恢复旧目标。

初始化阶段等待全部有效绑定源，之后启用原时钟 watchdog。
地图只随实际版本／时间戳变化送入 EGO，位姿持续送入。
本轮地图定义为 EKF odom，故采用显式 identity 变换；这不能与 W0 现有非 identity
飞行对齐混用。飞行时只能由唯一 FlightServer 提供 TF／localized odometry／session，
不能并行启动未解锁参考 observer。入口不提交导航目标或 FMU 输入。
实测源图通过、地面机体／制动观测包络未通过，见
[源绑定验证](validation/simulation/2026-10-10-depth-planning-sources/REPORT.md)。
