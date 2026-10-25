# 仿真启动规划

[项目首页](../../README.md) · [仿真控制](SIMULATION_CONTROL.md) · [代码目录](../../scripts/simulation/README.md)

## 两个仿真程序

公共入口只提供两种完整程序，不再按场景、定位组合、自动测试或算法模型分出其他启动类型。

| 程序 | 启动 | 运行内容 |
| --- | --- | --- |
| 光流纯手动 | `./scripts/sim.sh run manual` | Gazebo、PX4、QGC、下视光流和测距；没有 VIO、外部视觉融合、nvblox、EGO 或自动任务 |
| 完整项目 | `./scripts/sim.sh run full` | Gazebo、PX4、QGC、RViz、双目／IMU VIO、光流／测距、深度、nvblox、EGO 和遥控器任务控制 |

`./scripts/sim.sh` 直接打开两项菜单；`list` 只列出这两个程序。两者使用本机 PX4 SITL 动力学，保持未解锁，不自动起飞。完整项目的任务由遥控器触发；纯手动程序拒绝 PREPARE／RUN／暂停恢复等自动任务意图。

## 全局无 GNSS 策略

两个程序都关闭 Gazebo NavSat 数据源和 PX4 GNSS 融合，固定 `EKF2_GPS_CTRL=0`。手动程序采用 `FLOW_LOCAL`，关闭全部 EV 融合；完整项目采用固定的 VIO 主定位及光流／测距辅助配置。切换程序需先落地上锁并关闭当前会话，不能在空中切换定位来源。

DDS、QGC、时钟和界面正常不等于定位就绪；光流程序仍需地面融合检查，完整项目仍需 VIO、传感器和地图准入。完整项目保留 SDK Schur64 实验配置及其协方差资格边界，详见 [VIO 操作](../flight/VIO_POCKET_AVOIDANCE.md)。

完整项目启动期间关闭深度融合，RViz 地图应保持为空。既有 SDK 预热及源检查通过，并且未解锁着地状态下 VIO、PX4 连续 10 秒位置跨度均不超过 3 cm，VIO 姿态与 PX4 航向变化不超过 1°，才清空原生地图并开放建图；消息队列里的启动旧帧也会丢弃。终端显示“定位稳定，已清空原生地图并开放深度建图”后开始生成地图。启动检查失败时不会保留漂移阶段的重建。

## 构建准备

按[安装指南](../setup/INSTALLATION.md)准备 `.venv`、ROS 2、Gazebo 和相关依赖。两个程序共享锁定的 PX4、Agent、消息与扩展 EKF 遥测构建；光流程序使用这个 PX4 二进制，但不启动任何 VIO 进程或相机。

```bash
./scripts/build/build_px4_sim.sh --jobs 4
./scripts/build/build_px4_flight.sh
./scripts/build/build_px4_vio.sh --jobs 4
```

完整项目还需要感知、建图／规划和隔离 SDK：

```bash
./scripts/build/build_algorithm_sim.sh
./scripts/with_venv.sh python scripts/build/build_vio_sdk16_schur64.py --jobs 3
```

已完成的构建无需每次重复。`./scripts/build/build_px4_vio.sh --check` 检查 PX4 遥测构建。若提示 `VIO build inputs changed`，按错误给出的命令显式重建；代码或资源迁移也会导致旧凭据失效，不跳过摘要检查。完整项目构建和 SDK 前置条件见 [VIO 文档](../flight/VIO_POCKET_AVOIDANCE.md#启动)。

## 遥控器与配置

默认 `--input-source physical --device auto`，自动识别实体 RadioMaster Pocket HID（EdgeTX／OpenTX，8 轴／24 按钮），使用 `pocket-rc-v2`。没有设备或设备身份不匹配时明确失败；不会自动切换成模拟输入。

```bash
./scripts/sim.sh run manual
./scripts/sim.sh run full
# 多个设备时指定实际 HID 路径，可继承 QGC 校准
./scripts/sim.sh run full --device /dev/input/by-id/usb-OpenTX_Radiomaster_Pocket_Joystick_00000000001B-joystick \
  --qgc-config /绝对路径/QGroundControl.ini
```

显式使用模拟遥控器：

```bash
./scripts/sim.sh run full --input-source virtual
./scripts/sim.sh run manual --input-source virtual
```

程序复用现有 `Pocket Virtual UAV AETR`（9 轴／3 按钮），或自动创建并管理面板。创建需要 `/dev/uinput` 权限。QGC 使用每会话隔离配置，但已完成的校准会按设备名与输入配置保存在 `.cache/simulation/qgc-calibration/`，后续默认自动复用；实体与模拟设备分别保存。首次或更换设备时才需校准，在 Fly 页面发送操纵。校准文件参数不会改写原文件。实体输入与模拟输入分别使用对应映射，启动不改变开关档位。控件见[面板与开关说明](../flight/VIRTUAL_POCKET_PANEL.md#最新控件定义)。

两个程序共用 `--device`、`--qgc-config`、`--session-duration`（60–14400 秒）。到期后仍需确认落地上锁才正常关闭。

full 可显式增加 `--people`，运行 RGB 人员检测、配准深度定位和地图标注；默认关闭，不增加菜单程序或操作步骤。先构建可选模块并准备固定模型，见[人员地图定位](../../src/uav_target_perception/README.md)。`--person-model` 可指定模型路径；纯光流 manual 不提供该功能。

准备固定版本的 CC0 人体资产后，用 `run full --people --person-scene standing` 加入一个静态站立人体；默认场景不含该模型。资产准备和独立 RGB-D 几何测试见[人体仿真模型](../../src/uav_target_perception/README.md#人体仿真模型与定位测试)。

`--person-scene postures --person-rotations` 可组合站立、坐姿和仰卧模型，并启用额外方向推理（增加 CPU 开销）；需先准备坐姿资产。确认过的人员位置默认保留在 RViz 的橙色 Last Observed People 图层，绿色实时点仍按有效期更新，地图/定位坐标变化会清空历史。详见[持续标记说明](../../src/uav_target_perception/README.md#rviz-持续显示最后观测位置)。

着色地图使用 `./scripts/sim.sh run full --color-map`，可与 `--people` 同时开启。RGB 与深度共享地图准入/输入门控，并以有界频率融合。扫描结束并落地上锁后，用 `sim.sh export-mesh 新文件.ply` 保存实际顶点 RGB；此选项不增加菜单步骤，也不改变默认的深度建图配置。

## RViz 地图着色

使用 `run full --map-color 模式`，或在 full 菜单中选择“RViz 地图着色”。默认 `height`。选项可保存进 JSON 的 `options.map_color`；运行器生成本次会话的 `map-view.rviz`，不改写仓库 RViz 模板。着色只改变显示，不改写 ESDF、规划判据或导出模型的真实 RGB。

| `--map-color` | 实际显示的数据 | 可调参数 |
| --- | --- | --- |
| `height` | 网格顶点高度，低蓝高红 | `--map-height-min 0 --map-height-max 4`（m） |
| `rgb` | 相机融合进地图的真实 RGB；自动开启 `--color-map` | 可额外开启 `--people` |
| `normals` | 表面法线方向；无顶点法线时从三角面恢复 | — |
| `esdf` | 已观测 ESDF 水平切片；近障碍红、远处蓝 | `--map-slice-height 1 --map-distance-max 3`（m） |
| `semantic` | 人员可见躯干表面点附近为紫色，其余灰色未分类；自动开启 `--people` | `--map-semantic-radius 0.4`（m） |
| `weight` | nvblox 原生 TSDF 融合权重，低蓝高红；固定权重上限 5 | — |
| `age` | 网格块最近一次原生发布距今的时间，近蓝远红 | `--map-age-max 30`（s） |
| `traversability` | 已观测 ESDF 水平切片中的球形机体净空，红色不足、黄色余量较小、绿色余量充足 | `--map-slice-height 1 --map-clearance 0.8 --map-caution-margin 0.3`（m） |

```bash
./scripts/sim.sh run full --map-color rgb
./scripts/sim.sh run full --map-color normals
./scripts/sim.sh run full --map-color esdf --map-slice-height 1.2 --map-distance-max 3
./scripts/sim.sh run full --map-color semantic
./scripts/sim.sh run full --map-color weight
./scripts/sim.sh run full --map-color age --map-age-max 60
./scripts/sim.sh run full --map-color traversability --map-slice-height 1.2 \
  --map-clearance 0.8 --map-caution-margin 0.3
# 先预览最终命令，不启动仿真
./scripts/sim.sh run full --map-color height --map-height-min 0 --map-height-max 3 --dry-run
```

ESDF 与可通行性显示所选高度所在体素层的中心平面；高度采用 `map` 坐标。未知、非有限距离及切片之外的体素不显示，不能解读为可通行。ESDF 上限只是颜色饱和值，不能扩大 nvblox 当前 3 m 的距离计算范围。可通行性从距离中减去球形机体半径以及半个体素对角线，剩余距离小于等于 0 为红色，达到黄色余量阈值为绿色；该图只说明几何净空，不判断路径连通、制动、动力学或任务可执行性。默认半径 0.8 m 与 full 规划上下文一致，调整显示参数不会调整规划器。

`semantic` 复用当前仅识别人、以配准深度和历史 TF 定位的模块，需先按[人员地图定位](../../src/uav_target_perception/README.md)构建并准备模型。紫色只是人员表面点指定半径内的空间关联，可能包含邻近背景；没有墙、地面、设备等类别的分割。人员失效、身份上下文不匹配或采集时间超过 2 s 后恢复未分类，移动目标不会永久染色。

`weight` 使用 `/nvblox_node/tsdf_layer` 中实际 `clamp(weight/5, 0, 1)`，显示距离小于等于 0 且权重大于 nvblox 可视化阈值的体素；它是融合权重，不是概率或检测置信度。`age` 是**网格块发布年龄**，不是逐体素观测时间；完整地图重发会重置对应年龄，加载旧地图也无法恢复原始采集时间。

派生显示以 0.5 Hz 更新，附带英文颜色图例和 `/uav/map/color_status` 状态。地图未准入、失效、数据过期或时钟回退时清空显示；网格和权重缓存最多 4096 块／100 万顶点或体素，超过上限清空并报告 `DISPLAY_CACHE_LIMIT`。这些模式额外消耗序列化与显示资源，未做整机性能或飞行验收。`manual` 无建图，不接受这些配置。

新增显示节点后，只需增量构建 Python 包；已有环境无需重编 nvblox：

```bash
./scripts/with_venv.sh colcon build --base-paths src/uav_nav_sim \
  --build-base build_uav --install-base install_uav --packages-select uav_nav_sim --symlink-install
```

## 配置保存与覆盖

```bash
./scripts/sim.sh run full --input-source virtual --dry-run \
  --save .cache/sim/profiles/full.json
./scripts/sim.sh run --config .cache/sim/profiles/full.json
./scripts/sim.sh menu --dry-run
```

默认值、JSON、命令行依次覆盖。格式为 `{"schema":1,"profile":"full","options":{"input_source":"virtual"}}`；未知配置、旧 profile 和旧选项会拒绝，不做隐式替换。`run --save` 保存启动配置，顶层 `save` 保存运行中的地图。保存配置不会覆盖已有文件。

## 会话控制与退出

启动终端保持程序运行；新终端从同一工作区执行，无需手工设置通信域：

```bash
./scripts/sim.sh status
./scripts/sim.sh logs
./scripts/sim.sh logs --follow
# 仅完整项目，飞机落地上锁，输出路径须不存在
./scripts/sim.sh save .cache/maps/run1
./scripts/sim.sh export-mesh .cache/maps/run1.ply
# 请求正常关闭；若在空中，等待飞手降落上锁
./scripts/sim.sh stop
```

`.cache/simulation/operator/current.json` 绑定程序、运行器 PID／出生标识、ROS 域、Gazebo 分区和运行目录，防止连接旧会话或复用后的 PID。`status` 显示实际预检、遥控器和最终结果，不能把进程存活当作可飞。地图服务端独立检查新鲜唯一的 PX4 落地上锁状态，随后关闭深度输入、保存并恢复建图；不伪造旧算法执行器的 HOLD 状态，不向飞控发送地图控制命令。

`save` 输出含 `manifest.json` 与 `static_map.nvblx` 的地图包；`export-mesh` 输出 PLY。当前完整项目持续在线建图，暂不提供跨会话静态地图加载。保存的地图与本轮场景、分辨率和地图身份绑定。操作超时后先看文件及日志，避免重复请求。

`stop` 只写入本轮运行器的关闭请求，不自动降落或结束其他会话。正常操作先降落上锁再关闭。Ctrl+C 作为中断路径交给原监督器处理；完整项目和光流程序均保留各自的故障清理记录。

运行器异常退出后，使用 `./scripts/sim.sh cleanup` 预览残留，再用 `./scripts/sim.sh cleanup --apply` 清理。该命令保留有存活监督器的会话，按 INT／TERM／KILL 分阶段结束本工作区的残留进程，并重新检查结果。

## 开发与历史入口

原 `algorithm`、`autonomous-mapping`、`navigation`、`vio-pocket`、`flow-pocket`、`flow-local`、`scene`、`advanced` 等不再是公共启动类型；旧命令明确提示使用 `run manual` 或 `run full`。专项运行器和回归资产保留在分类目录内，供开发与历史证据复核，见[代码目录](../../scripts/simulation/README.md)。历史报告中的命令描述当时的运行，不代表当前公共接口。
