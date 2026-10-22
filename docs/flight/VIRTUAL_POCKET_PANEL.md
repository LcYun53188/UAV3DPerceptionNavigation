# Pocket UAV 电脑面板

[文档目录](../README.md) · [实体 USB 接入](USB_POCKET_SIMULATION.md)

该面板模拟 UAV / Model 5 的 Mode 2 摇杆、CH1–CH16 输出与任务开关。
鼠标或键盘即可操作，无需实体 Pocket、飞控或接收机。它模拟项目输入协议，
不运行 EdgeTX 固件，也不模拟 CRSF/ELRS 射频链路。

## 启动

在工作区运行：

```bash
./scripts/sim.sh run full --input-source virtual
```

完整项目自动复用或打开模拟面板，并创建供 QGC 使用的 `Pocket Virtual UAV AETR` HID。若只需光流纯手动飞行，使用 `./scripts/sim.sh run manual --input-source virtual`。默认启动两个程序均使用实体 HID，模拟面板须显式选择。

连接要求 SA 上锁、SD 弹起、油门最低；创建设备需要 `/dev/uinput` 权限。Python 标准库与系统 Tk 即可运行。QGC 首次需校准并启用该摇杆，在 Fly 页面发送操纵。

开发时只预览面板可直接运行 `python3 scripts/simulation/pocket/pocket_panel.py`，它不再作为独立仿真启动类型。

## 最新控件定义

SB／SC 的负、中、正值分别对应 988／1500／2012 µs；上／中／下顺序为 −1／0／+1。
虚拟面板使用 `uav-aetr-v1`；实体 Pocket 使用 `pocket-rc-v2`，CH5–CH8
分别读取 Linux axis 4–7。实体输入必须先核对通道方向，不能只按开关外观推断。
旧 `legacy-usb` 映射仅供显式兼容，不随新规划改变。

| SB / CH6 | 主模式 | SC / CH7 = −1 | SC = 0 | SC = +1 | SD 按下 |
| --- | --- | --- | --- | --- | --- |
| −1 | 手动 | VIO＋光流辅助飞行，无 VIO 时请求已确认的独立光流配置 | 纯光流辅助飞行 | Stabilized 纯姿态自稳 | 原地缓慢降落 |
| 0 | 悬停 | VIO＋光流融合定点，无 VIO 时请求已确认的独立光流配置 | 纯光流定点 | Altitude 仅定高，无水平定点控制 | 原地缓慢降落 |
| +1 | 自动（任务） | 起飞准备 | 起飞并执行任务；干预后重新规划继续 | 手动干预：暂停原任务，VIO Position 接管 | 结束原任务，避障返回 PX4 Home 并降落 |

- **SA / L1**：自锁二段，CH5 为 −100%／+100%；请求地面解锁／解除武装。
  空中关闭 SA 不能强制切断电机。
- **SD / R1**：自锁二段，CH8 为 −100%／+100%。按下后保持状态，
  只有低→高边沿发一次请求；再次点击弹起，仅复位，不撤销已提交的返航／降落。
  在 SD 已按下时改变 SB，不重新触发另一种返航／降落请求。
  SD 请求优先于同时发生的 ARM／SC 执行／任务按钮；保持按下期间拒绝解锁、模式切换与任务启动。
  接入时 SD 已按下不会自动降落，但会阻止后续启动，须先释放再重新操作。
- **SE / LB**：自回弹，短按（≤0.9 s）松手后产生 CH10 高脉冲 0.5 s，
  切换任务暂停／继续；长按满 1.0 s 时产生 CH11 高脉冲 0.5 s，取消任务。
  长按保持不重复，长按松手不再产生短按。0.9～1.0 s 间无命令。
- **S1 / RB**：CH9 连续输出 −100%～+100%；当前只输出速度请求，
  尚未接入任务速度限制，不能认为旋钮已经改变飞机速度。
- CH1–CH4 按 AETR 顺序输出 Roll/Pitch/Throttle/Yaw；CH12–CH16 预留为 0%。
  左摇杆油门保持，另外三个轴松手回中。

实体 `pocket-rc-v2` 的 Classic USB 使用已核验的 button 1（CH10 短按脉冲）暂停／继续、button 2（CH11 长按脉冲）取消；虚拟面板仍用 button 0／1，二者编号不同。实体 S1／CH9 是 button 0 数字量，忽略该按钮，不提供连续调速。纯手动入口继续拒绝任务按钮请求；完整任务入口须选择对应设备配置。

### 已接入的控制与定位能力

手动、悬停的前两档使用 PX4 原生 Position：飞手可移动，摇杆回中保持位置与高度。
手动下档使用 Stabilized（主模式 7），而非 Acro；悬停下档使用 Altitude（主模式 2）。
Stabilized 救机与原生 LAND 的准入不依赖伴随计算机 VIO 或水平位置；仍检查实际姿态、
电池、输入新鲜度，LAND 另须有效高度。原生定高也检查高度来源，不能把融合中的 VIO 高度冒充气压／激光高度。
SA 解锁前须实际模式与所选档位一致；不一致时只请求切换，确认后须 SA 关→开，绝不排队解锁。

**档位协议已接入，跨源空中自动降级尚未接入。** `ASSISTED_POSITION` 只接纳当前实际验证
通过的 VIO 联合配置或独立 `FLOW_LOCAL` 配置；不能在 VIO 失效时继续使用旧位姿。
当前 `vio-pocket` 仍以 VIO 提供高度与航向，故 SC 中档“纯光流”及无 VIO 定高请求会明确拒绝，
不会暗改 EKF 参数或宣称已经切换。`FLOW_HORIZONTAL` 也保留 VIO 高度／航向，不等于纯光流。
需要纯光流／无 VIO 定高时须落地上锁后使用 `flow-pocket` 新会话；这不是空中自动降级验收。
任务档 SD 使用当前无 GNSS 的局部 Home／EGO 返航协议，不直接调用依赖全局定位的原生 RTL。
无 Home、地图或健康定位时拒绝返航；“强制动作”表示输入优先，不能覆盖这些实际执行条件。

切入任务档时，SC 必须先在准备档形成一次明确操作；不能携带手动档中的
SC 中档直接起飞。准备通过后仍须 SA 解锁、油门最低、摇杆居中、实际 PX4 已解锁，
以及现有地面、健康与唯一控制者检查通过，SC 中档才允许派发任务。
连接设备时的初始开关位置不会触发解锁、起飞、取消或降落。

键盘：F1/2/3/4 对应 SA/SB/SC/SD，空格对应 SE；W/S 调油门，A/D 控偏航，
方向键控俯仰／横滚。失去窗口焦点时中止未完成 SE 手势并回中三个自回中轴，
油门保持；不会由焦点变化产生暂停或取消。

## 自动飞行中的手动干预

SB=+1 时，SC=-1 起飞准备，SC=0 起飞并执行任务，SC=+1 手动干预。SC=+1 只允许已运行且健康的 VIO／EGO 任务：先通过原任务暂停服务制动并确认停止，再请求 PX4 原生 Position。实际模式确认后控制权记为 PILOT，网关停止发布 Offboard 心跳和位置设定点，摇杆直接经 QGC／PX4 控制飞机。不是 Acro，也不叠加自动任务的轨迹。手动阶段由飞手负责避障，恢复自动前重新检查地图路径。

切入干预和切回执行前，四轴（包括油门）均须回到中位死区，飞机须停稳；Position 的中位油门用于保持高度，不能把最低油门当作悬停。切回 SC=0 时先检查当前点／目标点的已观测安全包络，不合格则保留手动控制并拒绝恢复；通过后，网关预发送 Offboard 存活心跳，实际模式仍为 Position 时不发送轨迹；确认 PX4 已进入 Offboard 后，才发送当前位姿、制动、检查地图并从当前位置重新规划。原根任务身份、任务起点、剩余悬停时间、原悬停锚点及总截止时间保留；不会重新起飞或跳回旧轨迹。航点／返航恢复重新规划，已被手动移开的悬停点也需经 EGO 回接。

手动阶段受原任务总预算和 60 秒暂停上限约束；到期中止任务，重新取得控制权、停止、进入原有有界保持／降落清理。源失效、坐标重置、输入丢失或模式异常会终止本轮任务；不能静默恢复。无 GNSS 配置固定 `NAV_RCL_ACT=3`（遥控输入丢失降落）、`COM_RC_OVERRIDE=0`（控制权只通过显式请求切换）；Offboard 丢失仍使用原生 LAND。VIO Position 的行为见 [PX4 Position 模式](https://docs.px4.io/v1.16/en/flight_modes_mc/position)。

干预时短按 SE 不会抢回自动控制；回到 SC=0 才可继续。长按 SE 取消原任务后，在健康的 VIO＋光流会话中先制动悬停，再等待四轴（含油门）居中，交给原生 Position 手动控制。油门仍最低时显示 `CANCEL_WAIT_CENTER`，继续保持悬停；确认 PX4 实际切入 Position 后显示 `CANCELED_MANUAL`、owner=PILOT，停止 Offboard 心跳和位置设定点。取消后的手动控制不受已结束任务的总预算、60 秒暂停上限或正常 30 秒保持后降落计时约束。摇杆回中保持悬停，推杆直接手动飞行；VIO 和光流保持原融合配置，手动避障由飞手负责。SE／SC 不能恢复被取消的任务，同一会话重复根任务仍需落地重启。

这一交接只用于收到接纳回执的显式 SE 取消，并核对原根身份和实际取消清理结果；BT 租约丢失、干预超时、程序退出等保护清理保留原有行为。交接前定位、输入或唯一控制者检查失效时撤销 Offboard，由原生失联策略处理；请求 Position 后 3 秒仍未确认则进入原有原生降落后备流程。已经提交的降落不能撤销。任务仍活动时，SD 按 SB=自动结束任务、具备条件则避障返回局部 Home 并降落；SE 取消后，即使 SB 仍在自动档，SD 也直接请求原地 LAND。

初次练习 SC 接管可选择 `vio-pocket` 的 `task: "intervention"`：先在起点附近悬停，水平速度上限为 0.5 m/s，包含手动与本轮自动速度受控运动；其他模式不因该配置新增避障。先确认无杆量悬停，再短时小幅推动摇杆并立即回中，避免将连续打杆当作位置偏移命令。包络保护等导致原根任务终止后，任务档 SD 改为请求原生 LAND；仍检查实际飞控身份、姿态／高度、输入及唯一控制发布者。

## QGC 与项目控制器

Linux `uinput` 输出映射如下，**这些编号不等于 QGC Raw Channel 编号**。
QGC 的 SDL 排序与校准必须在通道监视器中实际核对。

| 模型输出 | Linux joydev |
| --- | --- |
| CH1–CH9 | axis 0–8 |
| CH10 暂停／继续脉冲 | button 0 |
| CH11 取消脉冲 | button 1 |
| 预留 | button 2，保持 0 |

面板终端和窗口显示本次 `/dev/input/jsN`。QGC 中选择 `Pocket Virtual UAV AETR`，
完成四轴、方向、油门校准并启用输入。QGC 只负责摇杆；不要再给其按钮绑定同一批
任务命令，否则会形成重复请求。

创建独立 GNSS/PX4/Gazebo/QGC 手动会话：

```bash
./scripts/with_px4_sim.sh python scripts/simulation/px4/run_px4_sitl_smoke.py \
  --usb-joystick --ui --duration 600
```

该入口不自动解锁／起飞。本次增加原生降落参数：`MPC_LAND_SPEED=0.3 m/s`、
`MPC_LAND_CRWL=0.2 m/s`，落地后 `COM_DISARM_LAND=2 s`。实际速度受 PX4
飞行阶段、测距与飞行状态影响；这不是速度已达到的验收证据。

先用窗口显示的设备路径观察新配置的意图：

```bash
python3 scripts/simulation/pocket/pocket_operator_session.py \
  --device /dev/input/jsN --input-profile uav-aetr-v1
```

绑定刚创建的独立手动会话并启用控制：

```bash
./scripts/with_px4_sim.sh bash -e -c \
  'source .deps/mission-install/local_setup.bash; exec python scripts/simulation/pocket/pocket_operator_session.py \
   --device /dev/input/jsN --input-profile uav-aetr-v1 \
   --session .cache/simulation/sitl/<本轮 UUID> --enable-sitl-control'
```

控制器校验本轮 PX4 进程、分区、domain、冻结参数和虚拟设备身份，
复用唯一 FlightServer，不创建第二个 FMU 控制发布者。SD 在任务模式下发
`END_MISSION`，保留根任务身份；返航就绪时制动、经 EGO 返回 PX4 实际 Home，再原生降落。起飞、扫描和暂停／控制交接阶段不再丢弃 SD，而是结束任务并交给原生 LAND 制动、原地降落，使用独立结果 `MISSION_ENDED_LOCAL_LANDED_AND_DISARMED`。SE 取消后的悬停等待／手动 Position 中，SD 直接请求原生 LAND；其他无活动任务或没有 EGO 导航的任务结束请求拒绝。不具备原生姿态／高度、唯一控制者等条件时保留具体拒绝原因。
手动档和定点档 SD 均请求 PX4 原生慢速 LAND，不依赖 GNSS、全局 Home 或地图；定位／高度不健康、任务仍占用控制权时拒绝。降落提交后 SC、SE 和手动接管均不能撤销原生降落。

QGC 负责设置／显示返航点，控制器只读取飞控确认的 `HomePosition`。QGC Plan 页的 Planned Home 是规划参考，不能当作飞控实际 Home；参见 [QGC Home 说明](https://docs.qgroundcontrol.com/Stable_V5.0/en/qgc-user-guide/plan_view/plan_view.html)。当前路线完全无 GNSS，自动返航使用局部 Home。不会补造 GNSS、暗中改为直线返航或把原地降落算作返航成功。

自动避障返航的目标转换、Home 变更与地图条件见 [VIO 返航规则](VIO_POCKET_AVOIDANCE.md#返航基准和条件)。
控制准入 ACK 不代表实际起飞、模式切换或成功着陆；须查看 PX4 状态与根任务结果。

无 GNSS 纯光流仓库可以改用 `./scripts/sim.sh run manual --input-source virtual`，
入口先进行地面融合审计，通过后接入 Pocket 控制器，不自动解锁／起飞。
操作与证据边界见 [Pocket 光流会话](MTF01_STANDALONE_FLIGHT.md#pocket-持续操作会话)。

**定位接入的当前边界：** 上述 GNSS 操作入口绑定 GNSS 手动会话，允许 Acro、任务操作、
返航与原生降落；面板虽可模拟 VIO／光流选择，但该会话不具备相应源。
`VIO_POSITION`／`FLOW_POSITION` 必须对应已接入且健康的 VIO／光流 gate，
否则返回 `REQUESTED_LOCALIZATION_NOT_ACTIVE_OR_READY`。当前没有在此入口中
启动传感器或执行定位源切换；不能把通道输出当成 VIO／光流已接入。
现有联合定位选择器只允许地面解除武装且无任务时切换，光流水平档仍保留 VIO
高度／航向；独立纯光流须使用另一个启动配置。VIO／光流／测距联合定位及深度／EGO
操作入口为 `./scripts/sim.sh run full`，启动前检查、实验范围与验证证据见 [VIO 文档](VIO_POCKET_AVOIDANCE.md)。

`/dev/uinput` 必须可写，生成的摇杆必须可读；udev 建立读取权限可能略晚于创建设备。
面板会显示等待状态。此机器已具备相关权限，不需要将整个面板作为 root 运行。
关闭／断开面板会销毁其虚拟设备，让现有输入丢失与 PX4 failsafe 正常生效。

## 验证

```bash
UAV_RUN_UINPUT_TEST=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 SKIP_WS_SETUP=true \
  ./scripts/with_venv.sh python -m pytest -q scripts/tests/local/test_pocket_panel.py
```

覆盖 SE 阈值与脉冲、SB/SC 上下文、SD 自锁与模式分流、初始位置不发命令、
准备前拒绝执行、焦点丢失、Linux 实际轴／按钮读回和断连。飞控门控测试位于
`src/uav_mission/test/test_pocket_operator.py`。这些测试与界面预览不等于 Gazebo
飞行、实际 VIO／光流模式切换或实体 Pocket 验收。

逻辑开关的 Edge／Duration／Delay 定义见
[EdgeTX 官方手册](https://manual.edgetx.org/bw-radios/model-select/logical-switches)。
SE 实现按上述用户指定的一次性短／长按行为，不声称与未经导入验证的 L02
固件表达式逐周期等价。射频通道精度与刷新率见
[ExpressLRS 开关配置](https://www.expresslrs.org/software/switch-config/)；
本虚拟 USB 路径不受 ELRS Wide/Full 模式影响。

VIO／深度／EGO 的持续操作入口与独立静态绕障验收见 [VIO Pocket 文档](VIO_POCKET_AVOIDANCE.md)。
