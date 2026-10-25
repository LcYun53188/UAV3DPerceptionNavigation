# 无人机仿真入口

所有命令从项目根目录运行，同一时间只运行一套仿真。运行 `./scripts/sim.sh` 打开总菜单，或用 `./scripts/sim.sh list` 查看配置类型。启动目录、配置和兼容关系见[仿真启动规划](../docs/simulation/LAUNCHERS.md)。

## 文件归属与常用入口

仿真资源和运行代码分开维护：本目录存放可版本化配置、补丁与原生插件；[scripts/simulation/](../scripts/simulation/README.md) 存放分类后的启动、操作、诊断和验收代码。

| 目录 | 内容 |
| --- | --- |
| `missions/` | 起飞、航点、悬停、返航、降落的任务配方 |
| `scenes/` | 仓库场景几何与验收布局参数 |
| `safe_regions/` | 控制边界、机体包络、制动余量及飞行限制 |
| `profiles/` | 算法仿真基线配置 |
| `scenarios/`、`acceptance/`、`schema/` | 回归场景、验收条件及数据格式 |
| `px4/` | PX4 版本锁、无 GNSS 服务器配置及专项资源 |
| `px4/mtf01/` | 光流与测距传感器、融合及独立飞行配置 |
| `px4/vio/profiles/` | VIO 传感器、位姿契约、融合及 SDK 候选配置 |
| `px4/vio/patches/` | VIO／PX4 源码补丁 |
| `px4/vio/telemetry/` | DDS 发布列表和飞控日志主题 |
| `px4/vio/plugins/` | 运动载台及同机运动插件源码 |
| `px4/vio/openvins/` | 保留的 OpenVINS 候选构建配置 |
| `assets/` | 外部仿真资产说明 |
| `scripts/` | 历史 Shell 入口兼容链接 |

`px4/vio/` 根目录的旧文件名是兼容链接，真实资源在上述分类目录中维护。Gazebo 世界和模型继续随 `uav_bringup/gazebo/` 维护，任务树随 `uav_bt/trees/` 维护。

## 两个仿真程序

```bash
./scripts/sim.sh run manual  # 光流／测距纯手动，无 VIO
./scripts/sim.sh run full    # VIO／建图／导航完整项目
```

绕柱、矩形框、圆环和树木四区赛道使用 `./scripts/sim.sh run full --course all`；单区选择、地图准备、尺寸和通过判据见[四区赛道说明](../docs/simulation/OBSTACLE_COURSE.md)。

两者默认实体 HID；显式添加 `--input-source virtual` 使用电脑模拟遥控器。启动保持未解锁，不自动起飞。光流程序没有 VIO、地图或自动任务；完整项目运行 PX4 动力学、VIO、nvblox、EGO 和遥控器任务控制。

## 在另一终端操作

```bash
./scripts/sim.sh status
./scripts/sim.sh logs --follow
# 完整项目：落地上锁后保存地图包和三维模型，输出路径须不存在
./scripts/sim.sh save .cache/maps/run1
./scripts/sim.sh export-mesh .cache/maps/run1.ply
./scripts/sim.sh stop
```

控制命令自动读取当前程序的 ROS 域、Gazebo 分区与运行目录。`stop` 请求在落地上锁后关闭，不发送飞行指令。构建、校准、退出与能力边界见[启动说明](../docs/simulation/LAUNCHERS.md)和[控制手册](../docs/simulation/SIMULATION_CONTROL.md)。

## 开发工具

专项传感器检查、自动飞行回归和历史算法验证保留在 [scripts/simulation/](../scripts/simulation/README.md) 中供开发诊断，不再注册为公共仿真程序。仿真资源、硬件定位和实机飞行按各自范围解释；验证条件见[全链路开发计划](../docs/development/SIMULATION_DEVELOPMENT_PLAN.md)，结果见[验证目录](../docs/README.md#验证记录)。
