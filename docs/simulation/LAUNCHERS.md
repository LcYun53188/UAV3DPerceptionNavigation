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
