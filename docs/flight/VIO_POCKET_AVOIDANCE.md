# Pocket 无 GNSS VIO 与静态避障仿真

该入口在本机运行 Gazebo、PX4、cuVSLAM SDK16、光流／测距、实际前向深度、nvblox 与 EGO。无需 Jetson 或实体相机。启动后保持未解锁；任务由实体 Pocket 或虚拟面板发起。

当前是显式实验配置：SDK16 的全局协方差资格仍待验证，仅用于 Gazebo/PX4 仿真。链路就绪、任务成功、真实绕障分别记录，不互相替代；尚不代表动态障碍或实机飞行验收。

全局仿真关闭 GNSS 融合及 Gazebo NavSat 数据源，参见[统一策略](../simulation/LAUNCHERS.md#全局无-gnss-策略)。

当前只提供两个公共仿真程序：`./scripts/sim.sh run manual` 是完全无 VIO 的光流／测距纯手动飞行；`./scripts/sim.sh run full` 是本页描述的 VIO、建图、导航和行为树完整项目。两者默认实体 HID，只有显式选择 `--input-source virtual` 才使用模拟面板；新终端地图及关闭操作见[控制手册](../simulation/SIMULATION_CONTROL.md)。

## 启动

先按[构建准备](../simulation/LAUNCHERS.md#构建准备)完成 PX4 遥测、感知建图和 Schur64 SDK 构建。若提示 `VIO build inputs changed`，显式重建：

```bash
./scripts/build/build_px4_vio.sh --jobs 4
./scripts/build/build_px4_vio.sh --check
```

默认使用实体 RadioMaster Pocket HID，启动完整项目：

```bash
./scripts/sim.sh run full
# 显式使用电脑面板；程序自动复用或创建虚拟 HID
./scripts/sim.sh run full --input-source virtual
# 可继承已有的 QGC 校准
./scripts/sim.sh run full --qgc-config /绝对路径/QGroundControl.ini
```

实体设备为 EdgeTX／OpenTX Pocket（8 轴／24 按钮），模拟设备为 `Pocket Virtual UAV AETR`（9 轴／3 按钮）；程序自动应用对应映射。QGC 必须校准、启用摇杆并在 Fly 页面发送操纵。`--device /dev/input/...` 可用于多设备时明确选择；物理输入不会自动回退到模拟设备。

实体与模拟输入共用 SB／SC／SD 控件。任务要求新的开关边沿、有效输入以及完整定位／地图准入，启动保持未解锁，不会自动发起任务。纯光流手动程序禁止自动任务。VIO 会话在 PX4 生成机体前等待 SDK 参数核验及双目／IMU 订阅就绪；该检查不等于定位或飞行已经就绪。

Pocket 入口使用 480×300 双目及重新生成的标定，前向深度保持 320×240；光流 OpenCV 与 BLAS／OMP 使用单线程预算，静止面板仅在状态变化时重绘。操作启动按连续 30 秒的 SDK 有效数据及 VIO/PX4 实测稳定准入，位置跨度 ≤3 cm、姿态变化 ≤1°；不再使用固定 120 秒倒计时，独立 SDK 候选资格审计仍要求 120 秒。收到 `POCKET_SESSION <目录>` 后才完成启动前检查。证据位于 `.cache/simulation/vio-sensors/<UUID>/`，包括 `manifest.json`、`startup-progress.jsonl`、`sensor-preflight.json`、`flight-admission.json`、`live-heartbeat.json` 和各进程日志。校准、资产、会话、控制实例均绑定；启动检查未完成时 ARM/PREPARE/RUN 请求会被拒绝。Pocket 源预热阶段若检查失败，会保留可视化诊断现场，记录 `POCKET_PREFLIGHT_BLOCKED`，并保持未生成飞行准入文件；不会自动重置失效源或起飞。

原先返回任务起点的配置已通过一次实际 Gazebo/PX4 静态绕障、到点悬停、返航、最终悬停、慢速降落及上锁的完整验收。独立检查确认实际地图中有障碍占据体素、直线路线确被阻断、接纳轨迹和飞机真值保持碰撞包络、源持续有效并完成清理。该结果属于自动验收运行；可视化 Pocket 会话另做地面就绪检查。轨迹图、失败诊断与历史证据见[原策略验证报告](../validation/simulation/2026-10-15-vio-fusion-diagnostics/REPORT.md)。改用 PX4 Home 的独立验证结果见[返航点策略报告](../validation/simulation/2026-10-15-px4-home-return/REPORT.md)。

PX4 Home 新策略已完成一轮完整无 GNSS 自动任务，独立几何／Home 审计 12 项通过；手动和定点 SD 原地慢降分别通过实际仿真。SD 提前结束和 SC 手动干预后继续均各完成一轮实际仿真验收；干预复测前一轮因定位真值误差超限失败，重复运行稳定性仍待验证。各轮结果独立记录，见[无 GNSS 验收报告](../validation/simulation/2026-10-15-no-gnss-return-acceptance/REPORT.md)。

SDK 原单精度 Schur 边缘化存在已复现的数值失效：正定输入可能输出不定先验。`--sdk-schur64` 使用双精度 SVD／求逆／Schur 运算，保留原 1e-6 奇异值截止和 float 状态存储，没有协方差裁剪或方差下限。EGO 跟踪使用同一条已校验 B 样条的解析速度／加速度前馈，保留原时钟、碰撞包络和 0.3 米误差限制。自动验收与 Pocket 均使用 `MPC_LAND_SPEED=0.3`、`MPC_LAND_CRWL=0.2` 米／秒；原始位置连续性保护及健康门限保留。

`--vio-primary` 明确要求该隔离 SDK 和实验绕障场景。地面准入仍要求 VIO、光流、测距成功联合融合；飞行中 VIO 位置／高度／航向始终必需，光流和测距允许因地形或纹理暂时不参与融合。原始传感器连接、新鲜度、有限值、安装方向与唯一发布者检查仍必需，融合变化如实记录为 `AUXILIARY_UNAVAILABLE`。该策略在启动时固定，拒绝跨源热切换；不会放宽 PX4 创新量检查。

## 新终端地图与关闭操作

完整项目保持运行时，在另一终端使用 `./scripts/sim.sh status`、`logs`、`save <地图目录>`、`export-mesh <文件.ply>` 和 `stop`。地图操作要求飞机落地上锁；命令自动绑定本轮通信域，不调用旧算法仿真。操作会话正常关闭与任务结果、飞行资格分别记录，主动停止 VIO 以清理进程不代表完成自动飞行验收。详细步骤见[控制手册](../simulation/SIMULATION_CONTROL.md)。

## 操作

带界面的深度建图会话在 RViz 的 `Live Depth Map` 显示实际 nvblox 三维网格，按高度着色并隐藏 3.8 米以上的屋顶；蓝色箭头为 PX4 局部位姿，绿色线为已发布的 EGO 规划路径。网格以 0.5 Hz 更新，无界面验收继续关闭网格发布。未扫描区域不会凭空补齐，启动地图清空后的当前会话数据会逐步重新显示。

### Position 手动起飞与未起飞自动上锁

融合辅助／定点档实际使用 PX4 Position，油门用于控制升降速度，中位用于保持高度。
本仓库锁定的 PX4 在油门超过约 65% 行程（`manual_control_setpoint.throttle > 0.3`）
时请求起飞；只从最低推到中位仍不能起飞。起飞推力还有 `MPC_TKO_RAMP_T` 爬升过程，
本配置为 3 秒，短促推高后立即拉低可能只使机体短暂离地。

确认定位 READY、QGC 在 Fly 页面、SB 手动、SC 上档、SD 释放后，油门最低并拨 SA 解锁。
需要手动起飞时，平稳推过上述门槛，观察实际离地后将油门回中保持高度；整个过程由飞手操作。
`COM_DISARM_PRFLT=10` 表示解锁后未及时起飞会自动上锁，停桨本身不说明 USB 或 VIO 掉线。
遇到“推油门无效”时先在上锁状态比较 HID axis 2 与 PX4 实际油门，再看本轮 ULog 中的
解锁、油门、起飞及上锁原因；不要通过关闭自动上锁保护来掩盖输入或操作问题。

SB 顺序：−1 手动、0 悬停、+1 任务；SC 顺序为上 −1／中 0／下 +1。手动时为融合辅助／纯光流／Stabilized 纯姿态，悬停时为融合定点／纯光流定点／无 VIO 定高。此 VIO 会话固定使用启动时选择的 VIO 与辅助光流／测距配置，独立纯光流、无 VIO 定高和空中自动降级尚未接入；不满足实际来源条件的请求会拒绝。独立光流请在落地且未解锁时关闭本会话，再用 `run flow-pocket` 重新启动。任务档及 SD 的操作如下，两种输入共用这些规则。

执行任务时先 SB=+1、SC=−1 准备，再 SA 解锁、SC=0 启动。SC=+1 手动干预：先暂停并停止，再交给原生 VIO Position；四轴（含油门）居中后切回 SC=0，重新取得 Offboard 并从当前位置规划继续。SE 短按暂停／继续；长按取消原任务后，在健康的 VIO＋光流会话中先停止悬停，等待四轴（含油门）居中，再切原生 Position 交给摇杆。正常取消不再等待 30 秒自动降落，也不能用 SE／SC 恢复已取消的根任务。油门最低时保持 `CANCEL_WAIT_CENTER`，不能把最低油门交给 Position；确认实际 Position 后显示 `CANCELED_MANUAL`、owner=PILOT，并停止 Offboard 心跳／轨迹。此后即使 SB 仍在任务档，SD 也请求原地 LAND。手动干预期间短按不会恢复自动控制。交接条件见[面板说明](VIRTUAL_POCKET_PANEL.md#自动飞行中的手动干预)。初始开关状态不启动任务，准备与执行要求新的操作边沿及 QGC 新鲜控制输入。

首次验证 SC 手动介入可在统一 `vio-pocket` 配置中选择 `task: "intervention"`（对应 `--pocket-task intervention`）。该任务保留真实扫描／建图与 EGO，随后返回起点附近 2 米高度、悬停 40 秒，供介入及恢复测试；不先飞向障碍箱。此配置冻结 `MPC_XY_VEL_MAX=0.5`、`MPC_XY_CRUISE=0.5`、`MPC_XY_VEL_ALL=-10`，由 PX4 将手动速度参数约束至同一 0.5 m/s 上限；该限制也影响本轮自动水平运动。启动后必须核对原生参数，不能只看启动器配置。默认 `task: "avoidance"` 保留原扫描／避障任务和速度配置。根任务因包络等保护终止后，SB 仍在任务档时 SD 可直接请求原生 LAND；不会向已结束的根任务发送恢复或返航意图。

SD 为自锁二段开关：手动和定点模式按下均原地缓慢降落；任务导航／悬停等返航就绪阶段按下发起临时返航，保存原任务检查点，经 EGO 返回 PX4 Home，保持按下直到到点则原生降落。返航途中释放 SD，保持 SA 解锁、SB=任务、SC=执行，控制器先制动并确认悬停，再通过 `RESUME_MISSION` 恢复未完成步骤，从当前位置重新规划；不重复起飞，不延长原任务总期限。初始返航制动期间释放也能恢复；重复按下可继续尚未提交降落的临时返航。

扫描建图（`OBSERVE`）阶段在地图／EGO 上下文就绪时也可临时返航；释放后先经 EGO 回到原扫描锚点，再重新执行未完成的整个扫描步骤，已有当前会话地图继续保留。未实现逐扫描点／角度续扫。定位重置、地图 epoch／对齐或任务身份变化、控制／输入不健康时拒绝恢复。释放后的制动／恢复请求最多等待 16 秒；恢复遭拒不自动重试或启动新任务，后续保持／故障处理沿用原任务期限。

起飞、扫描地图尚未就绪及非返航的暂停／控制交接尚未完成时，SD 结束原任务并交给 PX4 原生 LAND 制动、原地下降，不依赖未完成的地图；结果单独记录为 `MISSION_ENDED_LOCAL_LANDED_AND_DISARMED`，不算返航成功。接纳前仍检查原生姿态／高度、飞控身份和唯一输出者；最终以实际降落模式、落地及上锁确认。原生降落一旦提交，释放 SD 不能撤销。接入时 SD 已按下或未由本次 SD 发起的返航，不会因释放自动恢复任务；按住时切换 SB 不重复触发。

上述 SD 释放恢复已通过代码回归和独立 ROS／BT 假后端验证；新增操作尚无 Gazebo／实体遥控飞行验收。

任务准备会用当前起点检查完整扫描和导航目标；手动飞行后的落点偏移可能使扫描点越过既定安全区域，此时准备被拒绝，任务档 SA 也不能解锁。不得通过扩大安全区域跳过检查。当前 VIO 会话仅预热一个任务 Runner；一次任务结束或被拒绝后，需落地上锁并重启完整会话才能再次准备任务，避免在定位源绑定后自动增加 DDS 参与者。

## 返航基准和条件

PX4 发布的 `HomePosition` 是实际返航基准。QGC 设置的 Home 只有被 PX4 接受并发布后才有效；Plan 页的 Planned Home 仅用于规划距离／时间估计。QGC 负责设置与显示；避障返航由本机导航链路持续执行。参见 [QGC Home 说明](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/plan_view/plan_view.html)。

| SB 模式 | SD 动作 | 定位与返航点依赖 | 路径执行 |
| --- | --- | --- | --- |
| 手动 −1 | 原地慢降 | 有效的局部位置与高度 | PX4 原生 LAND |
| 定点 0 | 原地慢降 | 有效的局部位置与高度 | PX4 原生 LAND |
| 自动 +1 | 临时避障返航；途中释放恢复任务，持续按下则返航降落 | 新鲜 VIO／EKF 位姿、有效的 PX4 局部 Home、当前会话地图与 EGO | 制动后规划到 Home 上方；释放则先停稳再恢复原任务；降落提交后不可撤销 |

自动模式将原始 Home 的 NED 坐标转换为 ENU，再通过已绑定的定位对齐转换到地图坐标。任务启动位置继续用于起飞和任务相对坐标，返航目标单独读取 PX4 Home。返航请求接纳后冻结该点；局部坐标重置、Home 被改动、目标越界或地图／规划失败均拒绝或终止本轮返航，不能静默改走直线。已观察起点和固定目标的包络不代表任意新 Home 都已观察；实际返航轨迹仍需通过地图碰撞、未知空间、刹停和执行授权检查。

当前操作路线完全无 GNSS：手动和定点 SD 不依赖全局定位或 Home，自动 SD 只使用飞控确认的局部 Home 和已绑定的 VIO／地图坐标。不会补造 GNSS、设置虚构地理原点或回退到全局 RTL。局部定位／高度失效时，不能把原地慢降请求视为已成功执行。

## 静态绕障场景

`vio_warehouse_avoidance.json` 基于复杂纹理仓库增加 `avoidance_crate`：中心 `(0, 2.8, 1.1)`，尺寸 `(0.8, 0.4, 2.2)` 米，具有真实碰撞体。北侧货架向墙边移动 1.15 米，为目标保留刹停包络；西北纹理立柱位于 `(-2.8, 2.7, 2.5)` 米，用作真实近处视觉结构，并接受同样的碰撞检查。起飞与多视角扫描保留独立、冻结的安全区域；最后一个高位扫描点为 `(-1.9, -1.9, 3.25)` 米，覆盖返航高度包络的上部视野。完成扫描前，起点、目标和返航高度三处必须在实际地图中具有已观测的完整种子／刹停包络；未知单元会阻止准入。地图导航区域更大，必须经过 EGO 的已观测 ESDF、机体／跟踪包络与刹停余量检查。

任务为起飞 2 米、多视角扫描、到达相对起点 `(0, 3.6, 3.3)` 米、悬停、返航、降落。前往目标的直线会碰撞机体包络，存在先爬升等绕行路径。EGO 速度／加速度／加加速度上限为 `0.18 / 0.15 / 0.20`；机体半径 `0.5` 米、跟踪余量 `0.3` 米、水平刹停余量 `1.2` 米均保留，EGO 搜索与执行网关使用同一水平刹停包络。固定任务总预算为 420 秒，覆盖原任务及一次最多 60 秒的干预和交接余量，独立观察时限 470 秒；运行时不延长任务截止时间，单段运动与扫描时限不变。目标较旧方案南移 0.4 米，为北侧货架边缘保留观测／制动余量；直线仍会碰撞箱体包络，安全包络没有缩小。

自动验收入口与 Pocket 操作会话互斥。关闭操作会话后运行：

```bash
./scripts/simulation/environment.sh vio python scripts/simulation/vio/runtime/run_px4_vio_sensors.py \
  --experimental-chain --flight sequence --normalize --mtf01 \
  --scene warehouse --warehouse-variant avoidance --warehouse-floor-texture \
  --quality-policy bounded_gap --real-time-factor 0.8 \
  --render-device nvidia --sdk-image-depth 1 --headless-rendering \
  --sdk-executor single_thread --sdk-cpu-list 0-15 --image-resolution 480x300 --duration 120 \
  --sdk-schur64 --vio-primary
```

完成后在 Python 环境中运行 `./scripts/with_venv.sh python scripts/simulation/vio/analysis/assess_vio_static_avoidance.py <运行目录>` 独立检查：直线确被实物障碍阻断、深度地图确有障碍占据体素、接纳的轨迹与 Gazebo 真值均保持包络间距、目标和返航完成、最终落地且未解锁。未完成任务不能判定绕障通过。

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

Pocket 的 `pocket-live.json` 与终端 `pocket-operator.json` 保存 `input_evidence`：区分 HID 事件／读取错误与网关收到的原生遥控输入间隔、无效记录和源时间间隔；另保存最近操作请求与任务接纳结果。静止时 HID 没有新事件属正常现象，不能据此判断断连；这些接收记录也不能单独判定 QGC 发送或 MAVLink 传输故障。
