# 仿真启动与控制脚本

`scripts/sim.sh` 管理 Gazebo + EGO + nvblox 导航仿真。它负责选择场景、统一两个终端的环境、记录启动进程与日志，以及调用初始化和地图服务。底层算法、碰撞检查和探索预算不变。

## 1. 准备与适用范围

先按 [安装指南](INSTALLATION.md) 安装依赖，再在工作区执行一次：

```bash
./scripts/build_algorithm_sim.sh
./scripts/sim.sh --help
```

脚本通过 `with_venv.sh` 加载 `.venv`、ROS 和 `install_uav`。脚本不自动安装依赖或构建。建议从未加载其他 ROS 工作区的新终端运行。

默认是扩展场景、在线建图、Gazebo + RViz 界面，ROS 域为 68、Gazebo 分区为 `uav_ego_lab`。`start` 的设置保存到 `.cache/sim/session.json`，随后在任意终端调用同一工作区的脚本都会复用这些设置，**无需再次 export**。这也避免了扫描终端处于域 0、启动终端处于域 68 而找不到服务的问题。

脚本只管理自己启动的一套导航仿真，不接管手动启动的 launch。已有手动仿真应先在原终端按 `Ctrl+C`。仅查看障碍场地与传感器，仍使用 `simulation/scripts/run_uav_obstacle_course.sh`，不要同时启动。

## 2. 在线建图并探索

**终端 A：**

```bash
./scripts/sim.sh start
```

终端保持运行，启动输出保存到日志。Gazebo 和 RViz 打开后，**终端 B：**

```bash
./scripts/sim.sh status
./scripts/sim.sh init
```

`start` 返回启动信息仅说明进程已创建，不表示地图已经就绪。`init` 会等待位姿和建图服务，取消已有目标，通过离线相机放置观测起点周围，然后回到起点。局部扫描使用 0.8、1.2、1.7 m 三个高度，覆盖 5° 下倾相机下起点机身体积的上下部分；在 1.2 m 高度补充四个斜向观察位置，并检查规划器种子路径所需的额外余量。等待出现：

```text
Local launch area observed; online mapping remains enabled.
```

导航时机身以水平姿态为目标，观察阶段只转偏航，不再为扩大相机视野主动大幅俯仰。取消目标或受阻后仍会在定位有效时回正姿态。该仿真直接控制速度，尚未模拟真实多旋翼依靠倾斜产生水平加速度的动力学。

这一步不是自主起飞，也不是自主探索飞行。初始化期间模型瞬移是预期行为，不要同时在 RViz 发送目标。每次重新启动建图会话都需重新初始化，或执行完整扫描。

然后在 RViz 中：

1. 确认 Fixed Frame 为 `map`。
2. 选择顶部 **2D Goal Pose**，在地图上按下并拖动方向后松开。
3. 默认目标高度为 1.2 m；中间轨迹可以升降。
4. 用 `status` 查看任务状态，不能仅凭模型朝向判断是否开始飞行。

也可直接发送 XYZ 目标，坐标属于 `map`，单位是米：

```bash
./scripts/sim.sh goal -6.5 -7.5 1.2
./scripts/sim.sh status
```

这是坐标语法示例，不是扩展场景必达目标。命令会等待有效地图和里程计再发布；发布成功不等于目标已被规划或到达。新目标会替换旧目标。未知区域可以作为最终目标，但无人机只沿已观测、安全的路径分段移动。

在线建图时，若当前位置到最终目标的直线走廊已完全观测，并通过机身范围的保守碰撞检查，可跳过原地转向与固定观察等待，直接申请规划；轨迹仍需规划器和执行器分别校验。未知区域、受阻走廊及正在进行的扫描仍保留观察流程。

平滑优化失败时，兜底轨迹在直线内部连续通过，仅在保留的路径拐点停车。局部探索段之间目前仍采用静止接管，尚不支持飞行中的轨迹替换。

未知目标的探索选点与规划器使用一致的种子路径安全余量；A* 会检查真实端点到邻近网格点的连接，粗网格失败后再以地图分辨率搜索。探索选点对偏离目标高度增加软惩罚，不强制等高飞行。

选点先比较整个已知连通区域的探索边界，以已知通路长度衡量代价，并降低已访问区域附近的优先级，再沿通路分段前进。观测点与连接通路均加入障碍间距代价，优先保留较宽敞的位置；窄通路仍需通过原有安全检查。默认探索步长为 3 m，按实际通路长度截断，不按加权代价缩短；它是选点上限，不是强制飞行距离，也不允许穿越未知体积。单独增大步长并不能解决下倾相机下的连续下降。

连续两段未缩短目标距离时，先按相邻方向扫描四个朝向，再选择下一段。到达局部点后，新增至少 0.5 m³ 的已观测地图体积也计为探索进展，因此绕障过程中暂时远离目标不会仅因直线距离停滞而过早退出。60 秒无进展超时、40 段上限和健康检查仍有效；原地扫描本身不会刷新进展计时。`OBSERVING` 表示仍在任务中，`BLOCKED` / `STOPPED` 才是终止状态。

### 无需目标点的自主探索

完成 `init` 后运行：

```bash
./scripts/sim.sh explore
./scripts/sim.sh status
# 随时停止自主探索并悬停
./scripts/sim.sh cancel
```

`explore` 启动在线自主边界探索：首次在当前位置扫描四个方向，再从已知、可达且有机身安全余量的探索边界选择观测点。扫描从当前朝向开始，避免先转回固定方向。到达后若途中新增至少 0.5 m³ 的地图体积，可只沿到达朝向短暂观察；连续两次短扫描后、距上次完整扫描位置超过 4 m、地图收益不足或规划受阻时，恢复四向扫描。短扫描没有候选点时，也会先补完整扫描再判定耗尽。选点只使用实时地图；附近未知体积作为潜在观测收益估计，不代表一定可见。飞行仍由原有 EGO 规划器及执行器检查，不使用离线瞬移扫描代替巡航。

`status` 的 `exploration` 显示 `SCANNING`、`SCANNING:BRIEF` 或 `EXPLORING`。手动发布 RViz/命令行目标会退出自主模式；`cancel`、地图会话改变和健康故障均停止自主模式，数据恢复后不会自行续飞。重复执行 `explore` 不会重置正在运行的任务。

自主任务最多运行 30 分钟或完成 120 个观测点，180 秒没有至少 0.5 m³ 的累计新增已观测体积时停止；这些限制覆盖飞行和扫描阶段。连续 8 个目标受阻也会停止。`LIMIT_REACHED` 表示预算终止，`FRONTIERS_EXHAUSTED` 表示当前可达且未访问的候选点已耗尽，**两者都不等于全图已覆盖**。封闭区域、视野盲区和不可达未知空间可能保留。

## 3. 完整扫描与地图保存

完整扫描用于准备可重复使用的静态地图，与在线探索共用 `mapping` 模式。如果仿真尚未启动：

```bash
./scripts/sim.sh start
```

在另一个终端扫描并保存：

```bash
./scripts/sim.sh survey .cache/maps/expanded_run1
```

完整扫描不需要先执行 `init`。扩展场景扫描 16 个水平位置、2 个高度、4 个朝向，耗时通常明显长于局部初始化。每个 `Surveyed` 输出表示一个观察位置完成，等待 `Saved map bundle ...` 后才算完成。扫描期间不发送目标。

若要保存在线探索积累的当前地图，使用：

```bash
./scripts/sim.sh save .cache/maps/explored_run1
```

`save` 自动取消当前目标、等待执行器报告 `HOLD`，然后调用地图会话的保存服务。输出目录必须不存在，脚本不会覆盖已有地图。地图包包含 `static_map.nvblx` 和 `manifest.json`；整个目录应一起保留。

保存后仍处于在线建图模式，可以重新发送目标。地图只在执行保存命令后持久化，`stop` 不自动保存。

## 4. 加载地图导航

切换模式必须退出并重新启动，不要在建图模式直接加载：

```bash
./scripts/sim.sh stop
./scripts/sim.sh start --mode localization
```

另一个终端加载：

```bash
./scripts/sim.sh load .cache/maps/expanded_run1
```

脚本检查地图文件，取消已有目标并等待 `HOLD`，加载完成后继续等待有效地图和新鲜里程计。看到 `地图和里程计已就绪` 后，再使用 RViz 或 `goal`。

必须与保存时使用相同场景、地图范围和兼容配置。小场景地图需 `start --layout lab --mode localization`。文件存在不代表一定兼容，最终以地图会话服务的校验为准。

此模式不积分深度，不探索未知区域，也不需要 `init`。这里的定位来自 Gazebo 真值，不是现实环境中的地图重定位。当前不支持加载旧地图后继续增量建图。

## 5. 启动选项与场景

```bash
./scripts/sim.sh start --help
# 小场景，只有 RViz 界面
./scripts/sim.sh start --layout lab --view rviz
# 完全无界面，后台运行
./scripts/sim.sh start --view none --background
# 指定 RViz 选点高度、ROS 域和 Gazebo 分区
./scripts/sim.sh start --goal-height 1.5 --domain 70 --partition my_uav_sim
```

以上是互斥的启动示例，每次切换前先 `stop`。

| 选项 | 默认值 | 说明 |
| --- | --- | --- |
| `--layout` | `expanded` | `expanded` 或 `lab`，自动匹配 world、查询范围与扫描布局 |
| `--mode` | `mapping` | 在线积分深度；`localization` 用于加载地图 |
| `--view` | `both` | `both` 打开 Gazebo 和 RViz；`rviz` 仅 RViz；`none` 全部隐藏 |
| `--goal-height` | `1.2` | RViz 的目标 Z；命令行 `goal X Y Z` 使用显式 Z |
| `--domain` | `68` | 不读取新终端的 ROS_DOMAIN_ID；由本次 start 明确指定 |
| `--partition` | `uav_ego_lab` | Gazebo 传输分区；名称不随场景变化，也不代表 world 名 |
| `--background` | 关闭 | 后台启动；终端返回后用 `stop` 退出 |

| 场景 | 查询范围 | 初始位置 | 用途 |
| --- | --- | --- | --- |
| `lab` | X/Y ±5 m，Z 0–4 m | (-3, 0, 1.2) | 小场景回归、较快扫描 |
| `expanded` | X/Y ±10.5 m，Z 0–4 m | (-7.5, -7.5, 1.2) | 较大场地、复杂遮挡 |

隐藏窗口仍需 GPU 渲染深度。后台运行不提供开机自启，也不是系统服务。脚本一次只管理一套仿真；更换域并不会允许同一工作区同时启动两套。

## 6. 状态、日志与退出

```bash
./scripts/sim.sh status
./scripts/sim.sh logs
./scripts/sim.sh logs --follow
./scripts/sim.sh cancel
./scripts/sim.sh stop
```

`status` 显示启动参数、PID、日志路径，以及约 4 秒内采集的 ROS 状态。`map.valid` 表示地图快照有效；`ready` 同时检查地图和里程计的新鲜度，**不表示目标一定可达**。缺少某项表示采样期间没有收到相应消息，检查日志与传感器。`receive_age_seconds` 是各消息距最近接收的墙钟秒数。

| 任务状态 | 操作判断 |
| --- | --- |
| `OBSERVING` | 正在观察，可原地转向；尚未执行移动轨迹 |
| `PLANNING` | 等待当前一段的规划 |
| `EXPLORING` | 正在移动到临时观测点 |
| `NAVIGATING` | 正在执行最终目标轨迹 |
| `REACHED` | 最终目标已到达 |
| `BLOCKED:…` | 无安全候选点、目标被占据或预算耗尽 |
| `STOPPED:…` | 地图、里程计、时钟或跟踪健康检查失败 |
| `CANCELLED:…` | 目标被取消、替换，或地图会话改变 |

`cancel` 只取消导航并等待 `HOLD`，仿真与建图继续运行。`stop` 向所记录的 launch 发送正常退出信号，最多等待 30 秒；不会按名称批量杀进程，也不会强制删除地图。退出前核对进程出生时间和系统启动 ID，防止旧 PID 记录误伤其他进程。

前台 `start` 终端按 `Ctrl+C` 等同于退出本次 launch。后台模式必须执行 `stop`。`logs --follow` 中按 `Ctrl+C` 只退出日志查看。

日志保存在 `.cache/sim/launch-*.log`，停止后仍可查看最近一次日志；该目录和地图缓存均不随 Git 提交。不要在运行中删除 session.json。若停止超时，保留日志和进程记录，按 [手动停止说明](EGO_NVBLOX_GAZEBO.md#停止正在运行的进程) 检查具体 PID。

初始化、扫描、保存、加载和发布目标互斥执行，避免两个终端同时更改地图。长时间扫描中断后，应先 `status` 检查；深度门控可能仍关闭，重新完成 `init`，或停止后重新开始。RViz 等外部发布者不受脚本操作锁约束。

## 7. 常见问题与验证范围

| 现象 | 检查与处理 |
| --- | --- |
| `Gazebo pose bridge unavailable` | 受管会话先看 logs，确认 world 和 bridge 启动；手动运行扫描时须设置与 launch 相同的 ROS_DOMAIN_ID |
| 只旋转、不平移 | 先完成 init；看 navigation 是否为 OBSERVING、NO_REACHABLE_FRONTIER，不能仅靠再次点目标解决盲区 |
| `MAP_INVALID` / `STALE_MAP_OR_ODOMETRY` | 查看深度、TF、里程计和 nvblox 日志；避免暂停 Gazebo。恢复数据后重新发目标 |
| `BLOCKED:EXPLORATION_BUDGET` | 当前局部探索未取得足够进展；选择已知可达点、换目标，或完整扫描后加载地图测试。延长预算不保证可达 |
| `BLOCKED:KNOWN_GOAL_OCCUPIED` | 目标落在实体障碍中，换到空闲空间 |
| 地图加载失败 | 核对目录内两个文件、场景、范围与配置指纹；查看 map_session 日志 |
| 检测到已有 launch | 先退出旧会话，不要绕过重复启动检查 |
| 窗口出现但无地图 | start 不等于就绪；mapping 执行 init，localization 执行 load，再看 status |

深度输入已加入最多 8 帧的 TF 等待队列，仍丢弃超过 0.5 秒的图像，不以最新位姿代替历史位姿。地图和执行器的超时保护仍有效。

目前 [小场景记录](validation/gazebo_exploration.json) 包含未知目标经三段到达；[扩展场景深度同步验证](validation/gazebo_depth_sync.json) 证明恢复了分段位移、未出现地图源过期，但最终以探索预算耗尽结束，**不代表扩展场景任意目标已实现可靠到达**。仿真使用真值定位和简化速度模型，以上结果也不构成实机飞行验证。

需要原始 ROS 命令、完整启动参数或底层接口时，见 [运行指引](EGO_NVBLOX_GAZEBO.md) 和 [仿真使用手册](SIMULATION_MANUAL.md)。

## 8. 控制脚本验证

2026-09-30 在无界面 `lab` 场景实测了：后台启动、跨终端查询、重复启动拦截、局部初始化、目标发布、取消并等待 HOLD、保存地图、正常停止、切换 localization、加载并等待 `ready=true`、前台 Ctrl+C 退出。另验证了错误模式操作会被拒绝。完整扫描命令复用已有扫描脚本，本次未重新执行完整扫描或 GUI 交互验收。

控制脚本与导航测试共 76 项通过，可重新运行：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ROS_DOMAIN_ID=180 ./scripts/with_venv.sh \
  python -m pytest -q scripts/test_sim_control.py src/uav_nav_sim/test
```

2026-09-30 导航连贯性优化后，上述 Python 回归测试共 94 项通过，另新增 3 项 C++ 兜底轨迹测试，验证直线内部不停顿、转角不切出已检查线段，以及相同速度／加速度／jerk 限制下的时间改善。4.3 m 直线的兜底轨迹计算时长由 26.058 s 降至 10.616 s；这是轨迹级测试结果，不是 Gazebo 端到端飞行计时。本轮未重启现有仿真会话，新行为需重启仿真后生效。

```bash
./scripts/with_venv.sh colcon build --build-base build_uav --install-base install_uav \
  --symlink-install --packages-select uav_ego_adapter --cmake-args -DBUILD_TESTING=ON
./scripts/with_venv.sh colcon test --build-base build_uav --install-base install_uav \
  --packages-select uav_ego_adapter --event-handlers console_direct+
```

2026-09-30 后续相机建图规划回归：105 项 Python、6 项 C++、10 项 vendor 补丁管理测试通过；相机配置未修改。新增障碍间距代价同时影响观测点评分和已知区域内的路线，实际步长仍按几何长度计算。测试覆盖宽敞绕路、仅剩窄路时保留通路，以及代价不会意外缩短步长。

扩展场景目标 `(2.1182, -3.69928, 1.2)` 的 [间距代价复测记录](validation/gazebo_clearance_routes.json) **未通过到达验收**：初次尝试发生地图源超时；地图恢复后从停止位置重新发起目标，360 秒内接受 23 段轨迹，结束位置约为 `(1.550, -3.049, 0.550)`，三维目标误差 1.081 m，采样最小真值机身间距约 0.100 m。超时前任务仍在探索，测试脚本随后取消并悬停。此前 [未加入间距代价的记录](validation/gazebo_frontier_routes.json) 在 7 段后以 `NO_REACHABLE_FRONTIER` 结束；两次起点和已观测地图不同，不能视为严格对照实验。当前仍存在连续下降、低空往返和无法恢复目标高度的问题，不能声称未知目标已可靠可达。

当前 Gazebo 相机安装下倾角已从 18° 改为 5°，模型与 `base_link → oakd_camera_link` 静态 TF 同步修改。已有仿真进程需重启才能生效；上面的飞行验证记录使用修改前的 18° 配置，不能作为 5° 配置的到达验证。

2026-10-01 自主探索启动验证：121 项 Python 回归测试通过，覆盖自动选点、扫描后派发目标、到达后继续、手动接管、取消与健康故障锁止、重复启用及预算停止。[90 秒只读观察记录](validation/gazebo_autonomous_start.json) 记录到 3 段接受并到达，已观测体素从 156483 增至 310877；观察结束后自主任务继续运行。该记录从任务已开始后采集，仅验证自主衔接与地图增长，不是全图覆盖或全程避障验收。


2026-10-01 扫描衔接优化：127 项 Python 回归测试通过。完整扫描从到达朝向开始，短扫描需要满足新增地图、距离和次数条件，无候选点时退回完整扫描。[120 秒实测](validation/gazebo_adaptive_scan.json) 接受 7 段轨迹，已观测体素由 136860 增至 390512；短扫描约 1.585–1.607 秒，完整扫描约 14.24–14.32 秒。这是扫描阶段耗时，不是任意目标的到达保证。

地图快照的距离与观测标志改用连续数组填充 ROS 消息，并通过序列化字节一致性测试。1764000 体素的本机字段填充微基准（各 5 次取中位数）由约 94.94 ms 降至 1.89 ms；该数字不包含 ESDF 计算、DDS 传输及轨迹检查，不能视为端到端延迟。地图源超时仍触发停止，新增日志分别报告已积分源数据年龄、最近转发深度年龄和等待队列长度，尚未证明所有地图超时已消除。
