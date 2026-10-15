# Pocket 无 GNSS VIO 与静态避障仿真

该入口在本机运行 Gazebo、PX4、cuVSLAM SDK16、光流／测距、实际前向深度、nvblox 与 EGO。无需 Jetson 或实体相机。启动后保持未解锁；任务由虚拟 Pocket 面板发起。

当前是显式实验配置：SDK16 的全局协方差资格仍待验证，仅用于 Gazebo/PX4 仿真。链路就绪、任务成功、真实绕障分别记录，不互相替代；尚不代表动态障碍或实机飞行验收。

## 启动

先启动虚拟面板，并确认打印出的实际设备路径：

```bash
./scripts/sim.sh pocket-panel --uinput
```

构建隔离的数值修正 SDK（只需首次或源码变化后执行，不替换默认库）：

```bash
./scripts/with_venv.sh python scripts/build/build_vio_sdk16_schur64.py --jobs 3
```

再显式选择该 SDK 和 VIO 主定位配置，启动完整可视化链路：

```bash
./scripts/sim.sh px4-vio-pocket --sdk-schur64 --vio-primary \
  --device /dev/input/js0 --session-duration 3600
```

设备必须是 `Pocket Virtual UAV AETR`、9 轴／3 按钮。已有 QGC 校准可通过 `--qgc-config-from /绝对路径/QGroundControl.ini` 导入。QGC 必须启用该摇杆且处于 Fly 页面；摇杆配置页面仅显示原始输入，不发送飞控控制。首次校准请按[虚拟 Pocket 文档](VIRTUAL_POCKET_PANEL.md)操作。

Pocket 入口使用 480×300 双目及重新生成的标定，前向深度保持 320×240；光流 OpenCV 与 BLAS／OMP 使用单线程预算，静止面板仅在状态变化时重绘。启动预热和源稳定性检查需要数分钟；收到 `POCKET_SESSION <目录>` 后才完成启动前检查。证据位于 `.cache/simulation/vio-sensors/<UUID>/`，包括 `manifest.json`、`sensor-preflight.json`、`flight-admission.json`、`live-heartbeat.json` 和各进程日志。校准、资产、会话、控制实例均绑定；启动检查未完成时 ARM/PREPARE/RUN 请求会被拒绝。Pocket 源预热阶段若检查失败，会保留可视化诊断现场，记录 `POCKET_PREFLIGHT_BLOCKED`，并保持未生成飞行准入文件；不会自动重置失效源或起飞。

原先返回任务起点的配置已通过一次实际 Gazebo/PX4 静态绕障、到点悬停、返航、最终悬停、慢速降落及上锁的完整验收。独立检查确认实际地图中有障碍占据体素、直线路线确被阻断、接纳轨迹和飞机真值保持碰撞包络、源持续有效并完成清理。该结果属于自动验收运行；可视化 Pocket 会话另做地面就绪检查。轨迹图、失败诊断与历史证据见[原策略验证报告](../validation/simulation/2026-10-15-vio-fusion-diagnostics/REPORT.md)。改用 PX4 Home 的独立验证结果见[返航点策略报告](../validation/simulation/2026-10-15-px4-home-return/REPORT.md)。

PX4 Home 新策略已完成一轮完整无 GNSS 自动任务，独立几何／Home 审计 12 项通过；手动和定点 SD 原地慢降分别通过实际仿真。SD 提前结束和 SC 手动干预后继续均各完成一轮实际仿真验收；干预复测前一轮因定位真值误差超限失败，重复运行稳定性仍待验证。各轮结果独立记录，见[无 GNSS 验收报告](../validation/simulation/2026-10-15-no-gnss-return-acceptance/REPORT.md)。

SDK 原单精度 Schur 边缘化存在已复现的数值失效：正定输入可能输出不定先验。`--sdk-schur64` 使用双精度 SVD／求逆／Schur 运算，保留原 1e-6 奇异值截止和 float 状态存储，没有协方差裁剪或方差下限。EGO 跟踪使用同一条已校验 B 样条的解析速度／加速度前馈，保留原时钟、碰撞包络和 0.3 米误差限制。自动验收与 Pocket 均使用 `MPC_LAND_SPEED=0.3`、`MPC_LAND_CRWL=0.2` 米／秒；原始位置连续性保护及健康门限保留。

`--vio-primary` 明确要求该隔离 SDK 和实验绕障场景。地面准入仍要求 VIO、光流、测距成功联合融合；飞行中 VIO 位置／高度／航向始终必需，光流和测距允许因地形或纹理暂时不参与融合。原始传感器连接、新鲜度、有限值、安装方向与唯一发布者检查仍必需，融合变化如实记录为 `AUXILIARY_UNAVAILABLE`。该策略在启动时固定，拒绝跨源热切换；不会放宽 PX4 创新量检查。

## 操作

SB 顺序：−1 手动、0 定点、+1 自动。手动时 SC 顺序：−1 Acro、0 光流、+1 VIO；定点保持 VIO。此 VIO 会话固定使用启动时选择的 VIO 与辅助光流／测距配置，SC 的独立光流请求会拒绝跨配置切换；独立光流请在落地且未解锁时关闭本会话，再用 `px4-flow-pocket` 重新启动。

执行任务时先 SB=+1、SC=−1 准备，再 SA 解锁、SC=0 启动。SC=+1 手动干预：先暂停并停止，再交给原生 VIO Position；四轴（含油门）居中后切回 SC=0，重新取得 Offboard 并从当前位置规划继续。SE 短按暂停／继续，长按取消；手动干预期间短按不会恢复自动控制。交接条件见[面板说明](VIRTUAL_POCKET_PANEL.md#自动飞行中的手动干预)。初始开关状态不启动任务，准备与执行要求新的操作边沿及 QGC 新鲜控制输入。

SD 为自锁二段开关：手动和定点模式按下均原地缓慢降落，自动模式按下结束任务并经 EGO 返回 PX4 Home 后降落。松开仅复位开关，按住时切换 SB 不重复触发。

## 返航基准和条件

PX4 发布的 `HomePosition` 是实际返航基准。QGC 设置的 Home 只有被 PX4 接受并发布后才有效；Plan 页的 Planned Home 仅用于规划距离／时间估计。QGC 负责设置与显示；避障返航由本机导航链路持续执行。参见 [QGC Home 说明](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/plan_view/plan_view.html)。

| SB 模式 | SD 动作 | 定位与返航点依赖 | 路径执行 |
| --- | --- | --- | --- |
| 手动 −1 | 原地慢降 | 有效的局部位置与高度 | PX4 原生 LAND |
| 定点 0 | 原地慢降 | 有效的局部位置与高度 | PX4 原生 LAND |
| 自动 +1 | 结束任务并避障返航、降落 | 新鲜 VIO／EKF 位姿、有效的 PX4 局部 Home、当前会话地图与 EGO | 制动后规划到 Home 上方，到点再原生 LAND |

自动模式将原始 Home 的 NED 坐标转换为 ENU，再通过已绑定的定位对齐转换到地图坐标。任务启动位置继续用于起飞和任务相对坐标，返航目标单独读取 PX4 Home。返航请求接纳后冻结该点；局部坐标重置、Home 被改动、目标越界或地图／规划失败均拒绝或终止本轮返航，不能静默改走直线。已观察起点和固定目标的包络不代表任意新 Home 都已观察；实际返航轨迹仍需通过地图碰撞、未知空间、刹停和执行授权检查。

当前操作路线完全无 GNSS：手动和定点 SD 不依赖全局定位或 Home，自动 SD 只使用飞控确认的局部 Home 和已绑定的 VIO／地图坐标。不会补造 GNSS、设置虚构地理原点或回退到全局 RTL。局部定位／高度失效时，不能把原地慢降请求视为已成功执行。

## 静态绕障场景

`vio_warehouse_avoidance.json` 基于复杂纹理仓库增加 `avoidance_crate`：中心 `(0, 2.8, 1.1)`，尺寸 `(0.8, 0.4, 2.2)` 米，具有真实碰撞体。北侧货架向墙边移动 1.15 米，为目标保留刹停包络；西北纹理立柱位于 `(-2.8, 2.7, 2.5)` 米，用作真实近处视觉结构，并接受同样的碰撞检查。起飞与多视角扫描保留独立、冻结的安全区域；最后一个高位扫描点为 `(-1.9, -1.9, 3.25)` 米，覆盖返航高度包络的上部视野。完成扫描前，起点、目标和返航高度三处必须在实际地图中具有已观测的完整种子／刹停包络；未知单元会阻止准入。地图导航区域更大，必须经过 EGO 的已观测 ESDF、机体／跟踪包络与刹停余量检查。

任务为起飞 2 米、多视角扫描、到达相对起点 `(0, 3.6, 3.3)` 米、悬停、返航、降落。前往目标的直线会碰撞机体包络，存在先爬升等绕行路径。EGO 速度／加速度／加加速度上限为 `0.18 / 0.15 / 0.20`；机体半径 `0.5` 米、跟踪余量 `0.3` 米、水平刹停余量 `1.2` 米均保留，EGO 搜索与执行网关使用同一水平刹停包络。固定任务总预算为 420 秒，覆盖原任务及一次最多 60 秒的干预和交接余量，独立观察时限 470 秒；运行时不延长任务截止时间，单段运动与扫描时限不变。目标较旧方案南移 0.4 米，为北侧货架边缘保留观测／制动余量；直线仍会碰撞箱体包络，安全包络没有缩小。

自动验收入口与 Pocket 操作会话互斥。关闭操作会话后运行：

```bash
./scripts/run_px4_vio_sensors.sh \
  --experimental-chain --flight sequence --normalize --mtf01 \
  --scene warehouse --warehouse-variant avoidance --warehouse-floor-texture \
  --quality-policy bounded_gap --real-time-factor 0.8 \
  --render-device nvidia --sdk-image-depth 1 --headless-rendering \
  --sdk-executor single_thread --sdk-cpu-list 0-15 --image-resolution 480x300 --duration 120 \
  --sdk-schur64 --vio-primary
```

完成后在 Python 环境中运行 `./scripts/with_venv.sh python scripts/assess_vio_static_avoidance.py <运行目录>` 独立检查：直线确被实物障碍阻断、深度地图确有障碍占据体素、接纳的轨迹与 Gazebo 真值均保持包络间距、目标和返航完成、最终落地且未解锁。未完成任务不能判定绕障通过。

上述命令默认 `--flight-case full`。分别增加以下参数可验收其他动作：

| `--flight-case` | 实际用例 |
| --- | --- |
| `finish-mission` | 任务中请求结束，经 EGO 返回局部 Home 并降落；另检查缺失 Home 时拒绝且控制状态不变 |
| `pilot-intervention` | 暂停、原生 Position 摇杆移动、恢复 Offboard、重新规划并完成原任务 |
| `local-land-manual` | 起飞后实际切入 Acro，再执行原地慢降 |
| `local-land-position` | 起飞后实际切入 Position，再执行原地慢降 |

后三种用例还需 `--qgc-config-from /绝对路径/QGroundControl.ini`，并关闭虚拟面板以便验收进程独占创建 uinput。它们读取实际 QGC／PX4 控制消息，使用虚拟摇杆，不代表实体遥控器。自动验收命令会按选定用例起飞；日常操作使用前面的 `px4-vio-pocket` 入口。

慢降用例显式中止原起飞任务，再独立执行原生 LAND，结果字段为 `no_gnss_landing_case_passed`；不能据此认定自动任务完成。干预用例即使根任务成功，若独立真值到点误差超过 0.30 米，整体仍判失败。各轮成功与失败均保留在验收报告。

诊断 SDK 协方差时，可显式增加 `--sdk-information-log`，将原始 15×15 信息矩阵、PnP／IMU 积分状态和原始时间戳保存为 `sdk16-information.csv`。该选项仅允许 SDK16 的未解锁诊断或实验链路；额外 I/O 的运行不作为生产延迟证据，不改变协方差和新鲜度检查。

如需进一步定位边缘化失效，先用 `./scripts/with_venv.sh python scripts/build/build_vio_sdk16_schur_diagnostic.py` 构建独立诊断库，再同时添加 `--sdk-information-log --sdk-schur-diagnostic`。它保留原优化结果，在完整 15 维信息矩阵失效时将原始 30×30 Hessian、单精度结果和仅供比较的双精度参考写入 `sdk16-schur.csv`。该库不会替换默认 SDK，参考结果不会作为估计器输入；缺失或漂移的构建凭据会使启动失败。

在落地且未解锁时创建运行目录内的 `close-ui` 文件可关闭持续会话；关闭会回收其拥有的进程。诊断页面与现场证据应保留到问题定位结束。
