# RGB、检测与 AR 路线监控界面

2026-10-30，首版桌面界面。主区域显示 RGB 图像、人体检测框和规划参考路线，右侧顶部左右并排显示左目和右目，下方显示定位／地图上下文和操作按钮。复用已有 Tk/OpenCV 环境，入口仍为 `vio_stereo_ui.py`，公共 `run full` 的原 stereo monitor 自动使用新界面。

## 使用

现有完整仿真启动入口：

```bash
./scripts/sim.sh run full
```

完整仿真的人体检测需启用；`--no-people` 会使 RGB 检测输入不可用。赛道及其他没有人员 RGB 桥的配置可能只有双目画面，此时显示明确的等待状态。启动准入和飞行命令仍由原任务控制系统处理。

先看无需 ROS 或仿真的合成界面：

```bash
./scripts/with_venv.sh python scripts/simulation/vio/runtime/vio_stereo_ui.py --demo
```

演示有明确的 DEMO 标记，框和路线均为合成数据。`--close-after 5` 可在五秒后关闭窗口。

也可对已运行的 ROS 图单独打开观察窗口：

```bash
ROS_DOMAIN_ID=<当前会话通信域> ./scripts/with_venv.sh python \
  scripts/simulation/vio/runtime/vio_stereo_ui.py /绝对路径/当前运行目录
```

运行目录可省略，省略时截图写入 `.cache/vision-monitor/vision-snapshots/`，不记录启动文件和双目诊断证据。指定目录时，截图写入该目录的 `vision-snapshots/`，沿用 `flight-images/` 的双目证据格式，每秒最多一对、最多 240 对，同时间戳的帧不重复保存。真实完整仿真由现有启动器传入运行目录、通信域和 PX4 消息环境，不需要额外启动第二个窗口。

## 显示与按钮

- 检测框只叠加到源时间戳及 frame ID 完全匹配的 RGB 图像。界面缓存至多 12 帧，用于匹配延迟到达的推理结果；没有匹配帧时保持无框。RGB 和双目超过 1 秒源年龄或接收年龄后移除。
- “显示人体检测框”“显示 AR 规划路线”控制本地叠加；“保存截图”保存当前新鲜画面和 JSON 说明；“清除选择”清除本地选择。
- 在 RGB 区点击独立人体检测框可选择图像跟踪目标，选框为黄色，显示 T 编号；后续检测通过保守的框重叠关联保留选择，检测数组顺序变化不会直接换人。编号仅代表本次选择，不是真实身份。
- 点击重叠框会要求选择独立目标；目标消失、观测超过 1 秒、出现多个候选或人员交叉导致歧义、图像坐标系／时钟／地图上下文改变时显示目标丢失并要求重新选择，不自动重关联。框关联不能保证跨遮挡身份，尚未接入飞行跟随。
- 右键、Esc 或“清除选择”取消本地选择；点击背景不会误选目标。截图 JSON 包含选择状态、编号及源帧。
- 2026-10-31 已接入三维目标预演后端。选择目标、后端绑定成功后，“开始预演”可用；开始后可暂停、继续或取消。按钮状态由新鲜且匹配当前选择的后端状态决定；没有后端、回复超时或会话变化时给出明确提示。
- 自动飞行跟随尚未放行：动态安全监视和 PX4 行为树跟随步骤仍待实现。预演的候选点未经过路径规划和动态净空检查，预演暂停／取消不是飞机停稳证明。详见[人体跟随方案](../development/PERSON_FOLLOWING_PLAN.md)。
- 关闭窗口只关闭观察器。完整仿真会话继续由现有 `scripts/sim.sh` 控制；关闭观察窗口不作为停止飞行命令。

## AR 投影条件

投影使用 RGB **采集时刻**的 `camera <- map` 历史 TF，以及与图像同尺寸、同相机坐标系的针孔内参。首版仅支持无畸变或已提供正确标定的无畸变图像；非零畸变、错误 frame、不可用历史 TF、异常内参均隐藏路线。

同时检查新鲜有效的 PlanningContext，以及同发布时间的 Path 和 TimedTrajectory。轨迹须属于当前地图 ID／epoch，版本不得晚于当前地图，时间不得早于最近上下文切换。路线超过 2 秒、上下文超过 1 秒、上下文改变或时钟回退都会隐藏旧叠加。重复源时间戳不能刷新输入有效期；无新鲜输入时显示等待或隐藏原因。

蓝绿色线仅标识**规划参考路线**，不表示执行器已接受、正在执行或动态净空满足。路线会裁剪至相机前方及画面边界；无可见片段时说明路线在视野外。首版未实现遮挡感知，投影线可能画在前景物体上。

## 固定输入话题

| 内容 | 话题 |
| --- | --- |
| RGB | `/px4_rgb/image` |
| 左右目 | `/vio/left/image`、`/vio/right/image` |
| 注册相机标定 | `/px4_depth/camera_info` |
| 检测框 | `/uav/people/detections` |
| 规划路线与身份 | `/planning/ego/path`、`/planning/ego/trajectory` |
| 地图／定位上下文 | `/planning/context` |
| 历史变换与时钟 | `/tf`、`/tf_static`、`/clock` |

监控节点使用仿真时间。硬件话题、畸变图像及实机时钟适配需单独实现与验证。界面仅发布目标选择，后端仅发布状态和独立预演候选点；二者均不发布飞行命令、规划器目标或控制 setpoint。

## 验证边界

几何和图像解码测试覆盖针孔投影、平移／旋转、相机后方、近平面／图像裁剪、异常几何、图像行 padding 及源／接收时间有效性。隔离 ROS/Tk 集成测试通过真实 DDS 输入验证图像、检测、TF、路线身份、延迟检测匹配、截图、上下文切换、过期和时钟回退，并检查界面不发布应用控制话题。

这些是显示与消息链路验证，未启动完整 Gazebo/PX4 飞行，也不构成人体跟随或实机资格。


## 三维目标预演后端

公共 `run full` 在开启 UI 和人体检测时，随现有 supervisor 自动启动 `person_follow_preview`，进程与会话一起清理。演示模式仍不连接 ROS 后端。

若正在运行旧版本仿真，可在同一 ROS 通信域单独启动后端（需现有相机、检测、规划上下文与定位消息）：

```bash
ROS_DOMAIN_ID=<当前会话通信域> ./scripts/with_venv.sh python \
  scripts/simulation/vio/runtime/person_follow_backend.py
```

不要同时启动两个后端。UI 与服务端均检查状态发布者唯一性；重复实例时禁用操作并拒绝命令。

| 接口 | 内容 |
| --- | --- |
| `/uav/people/follow/selection` | String JSON：UI 客户端 UUID、选择编号、递增序号、源帧、框、上下文及有效性 |
| `/uav/people/follow/set_intent` | SetMissionIntent：START / PAUSE / RESUME / CANCEL 控制预演；EXECUTE 拒绝飞行执行 |
| `/uav/people/follow/status` | String JSON：后端实例／会话、选择身份、状态、三维 track ID、运动估计、距离、候选点和执行阻断 |
| `/uav/people/follow/preview_goal` | PoseStamped：独立的可视化候选点，不是导航目标；必须结合有效 PREVIEW 状态和时间解释 |

后端额外消费 `/uav/people/observations` 与 `/planning/ego/odometry`。要求图像框、检测结果和三维观测的源采集时间一致；三维点投影须落在选中框内，地图／定位／对齐身份须匹配当前上下文，最新检测和观测均须为单人。不会用“最近三维人”代替选定目标。

绑定前缺少匹配观测时显示 ACQUIRE 和实际原因；绑定后短暂等候独立到达的下一组检测／深度观测，最多 0.35 秒，此期间撤下旧候选点。UI 选择心跳超过 0.75 秒、里程计超过 0.5 秒、感知／上下文超过 1 秒、最新感知无效、目标 ID 改变或出现歧义都会进入 LOST；重新选择后才可重新开始。重复源时间戳不能延长输入有效期。仿真时钟停滞超过 0.5 秒或回退后锁定本后端，需重启。

命令使用后端会话 UUID、实例 UUID 和独立请求 UUID；旧会话的迟到命令被拒绝。预演不持有现有飞行控制租约，不修改行为树航点任务；因此状态中的 `cleanup_confirmed` 为 null，不伪造物理停稳确认。


## 任务中的自动人物记录模式

右侧新增“开启人物跟踪模式（预演）”。开启后不需要再点击选人：后端在真实、非 mock 且拥有 TASK 控制权的正常任务阶段，自动确认单人运动并记录轨迹。初值为运动确认 1.5 秒／至少 5 帧、活动时限 10 秒、静止确认 2 秒、继续观察 3 秒、冷却 15 秒；这是未标定的影子策略。

界面显示 SHADOW_FOLLOW／SHADOW_OBSERVE 等阶段，当前原任务未被暂停。实际暂停、飞行跟随、停稳和恢复仍受未完成的动态安全／PX4 行为树接入阻断。模式开关不触发起飞或解锁。

完整仿真自动在会话专属 RViz 配置中添加 Person Motion Trajectories 图层，并在运行目录保存 person-motion-trajectories.json 和已完成段的 JSONL。独立后端可使用 `--trajectory-log /绝对路径/新文件.json` 指定日志；不指定时只发布轨迹，不保存磁盘记录。旧配置的 RViz 可手动添加 `/uav/people/motion_trajectories` MarkerArray，选择 Reliable / Transient Local。

详细要求与实际边界见[任务内临时人物跟随](../development/MISSION_PERSON_FOLLOWING.md)。
