# Pocket USB 与行为树决策仿真

[文档总目录](../README.md) · [脚本目录](../../scripts/README.md)

RadioMaster Pocket 使用 EdgeTX，可通过 USB Joystick 模式作为电脑 HID 输入。
连接数据 USB 口，充电 USB 口不传送摇杆数据。使用支持数据的 USB 线，遥控器开机后
选择 USB Joystick；先使用 Classic 模式。建立独立仿真模型，CH1–CH4 映射四个摇杆，
其他开关显式添加至输出通道。USB 路径不需要 ELRS 接收机。
端口区分见 [Pocket 官方说明书](https://cdn.shopify.com/s/files/1/0701/8066/7584/files/Pocket_A1.8.pdf?v=1770617495)，
USB 模式和开关通道见 [EdgeTX Joystick 配置](https://manual.edgetx.org/edgetx-how-to/configure-advanced-joystick-with-edgetx)。

2026-10-13 已连接并识别为 `EdgeTX Radiomaster Pocket Joystick`，USB ID `1209:4f54`，
当前用户可读取 `/dev/input/js0`，HID 暴露 8 个轴、24 个按钮。24 个按钮可能包含虚拟
占位项，不能据此认定所有开关已经映射。已记录四个摇杆轴、辅助轴和 Buttons 1 的
真实变化，开关用途由用户指定；尚未完成操纵飞行验收。

## 本轮 SA / SB / SD 配置与按钮更正

用户指定 SA 解锁、SB 手动/导航、SD 开始/暂停。后续确认 QGC Buttons 1 是有阻尼、
不会自动回中的拨轮；撤下此前按自回弹按键解释的返航绑定。Buttons 0 是自回弹按键，
当前仍未分配；建议用它请求返航/暂停返航/恢复返航，接入前核对对应 Linux button。
当前 SDL 对 Pocket 采用 gamepad 映射，QGC Raw Channel Monitor 的顺序为 USB
轴 `[0,1,3,4,2,5,6,7]`；不能把 QGC 显示编号减一当成 Linux js 编号。
本映射来自本机 SDL 读回、QGC 5.1.5 的
[轴排序实现](https://github.com/mavlink/qgroundcontrol/blob/v5.1.5/src/Joystick/JoystickSDL.cc)
和实际输入变化；遥控器 EdgeTX CH 编号是另一层。

| 控件 | QGC 显示 | Linux 输入 | 低值 / 高值或按下行为 |
| --- | --- | --- | --- |
| SA | Raw 4 | axis 4 | 地面解除武装 / 请求解锁 |
| SB | Raw 6 | axis 5 | Position 手动接管 / 允许导航；中档按手动处理 |
| SD | Raw 8 | axis 7 | 暂停任务 / 地面启动或恢复已暂停任务 |
| 保持位置的拨轮 | Buttons 1 | 历史观察到 button 1；连续轴待核对 | 当前不发任务命令；建议预留速度上限调节 |
| 自回弹按键 | Buttons 0 | 待核对 | 当前未分配；建议每次重新按下切换返航/暂停/恢复 |

高值为 USB 原始值 >20000；低值包括其余档位。初始档位不触发动作，须重新拨动/按下。
SB 导航只授权任务，不单独进入 Offboard；SD 启动必须 SA/SB 都允许、PX4 已实际解锁且
地面停稳。SA 解锁须 QGC 已启用校准后的新鲜 MAVLink 摇杆输入、油门 ≤−0.8、
其余摇杆偏移 ≤0.15，以及真实健康/地面门控。请求拒绝后不因开关持续保持而自动重试。
SA 在空中拨回低档会撤销自动控制，但不直接发送强制解除武装；原生降落继续受保护。

先运行上面的只读识别，再启动下方手动会话。启动后终端/manifest 中记录本轮目录；
只对该目录建立 Pocket 控制所有者：

```bash
./scripts/with_px4_sim.sh bash -e -c \
  'source .deps/mission-install/local_setup.bash; python scripts/pocket_operator_session.py \
   --session .cache/simulation/sitl/<本轮 UUID> --enable-sitl-control'
```

不带启用参数的 `python3 scripts/pocket_operator_session.py` 只观察并打印开关意图。
启用版本核对本轮实例 PID/rootfs、固定场景、domain/partition 和 Offboard-loss 参数，
独占会话锁；继承唯一 FlightServer 发送所有自动命令，不创建第二个 FMU 控制发布者。
默认 recipe 为 W0 起飞/两航点/悬停/返航/降落，导航执行后端为 DIRECT；SB 的“导航”
目前表示自动任务模式，不表示已启用 EGO 避障。必须先在 QGC 完成 Calibrate 并启用输入。
该 Pocket 所有者持续运行，任务结束后可继续手动训练，不依赖 full 飞行记录器的生命周期。
QGC 输入首次有效时，若 SB 选择手动且飞机仍地面未解锁，会请求一次 Position 模式；
不会因此解锁或启动任务。模式完成仍以实际 PX4 状态为准。
旧通用 USB BT 适配器是另一入口，不与 Pocket 所有者混用。

返航意图仅在已接受的自动根任务中改写计划，保留起飞点、根 UUID 和总预算；
停止原曲线 → 确认停稳/保持交接 → 返回起飞点上空 → 原生降落。后续普通航点/悬停
不再执行。返航暂停继续使用原暂停协议，恢复生成新物理子 UUID，不恢复旧曲线。
`RequestReturn` 服务与 Runner `RETURN` 意图分别记录准入 ACK；只有实际 RETURN 运动、
LANDING 反馈及清理确认的 `RETURN_LANDED_AND_DISARMED` 根结果才允许树成功。
原生降落不可暂停，起飞/扫描阶段暂不开放此返航改写。空中手动飞行不能直接加入新 BT 根。
当前 Pocket 实体输入没有返航绑定；上面的返航服务及状态机仍保留，不能把历史
Buttons 1 数字边沿记录当作自回弹按钮证据。QGC Buttons 指示只提供数字状态，拨轮
调速须另行验证连续轴、端点、方向和保持值，并实现限幅/变化速率约束，不直接映射油门。

## 临时返航后恢复原任务（已实现软件协议）

`RETURN` 表示飞手临时返航，进入制动前保存原步骤、未完成步骤索引，以及被打断悬停的
位置和剩余时间。保存根 UUID、控制 session、home、期限、定位 reset、固定对齐及
EGO 的 map/epoch/alignment 身份，不保存旧轨迹。正常 recipe 自身的 RETURN 没有
未完成普通任务，不提供恢复检查点。故障和取消撤销检查点，不能借恢复指令清除故障。

| 意图 | 当前含义 |
| --- | --- |
| `PAUSE` | 制动并确认保持；返航中使用它暂停返航 |
| `RESUME` | 继续刚刚暂停的操作；暂停返航后继续返航 |
| `RESUME_MISSION` | 仅在临时返航已确认 PAUSED 后，恢复原任务未完成步骤 |
| `END_MISSION` | 提前结束原任务并返航降落；丢弃恢复检查点 |

`/uav/px4/resume_original_mission` 使用 `ResumeOriginalMission`，经 Runner 的
`/uav/bt/set_intent` 转发。请求绑定实例、根 UUID、非零 request_id；重放相同请求返回
原准入决策，不重复改写计划，跨命令复用 request_id 拒绝。`RequestReturn.finish_mission`
为 true 时对应 `END_MISSION`，为 false 时对应临时 `RETURN`。

恢复须处于同根的 PAUSED/HOLD_CONTROLLER，实际 PX4 armed/Offboard、停稳、新鲜健康
输入、有效进展租约及唯一 FMU 写入者，并仍处在暂停和总期限内。定位 reset、home、
控制 session、对齐或地图身份变化均拒绝；EGO 允许地图 version 正常更新，但重新验证
当前地图并生成新曲线。原生降落已提交、终止结果已产生、取消或失控后不再恢复。

恢复递增 plan revision 和控制 generation，保留 home 和原期限，重新生成物理 child；
BT 重置执行分支，跳过已完成前缀，不重复 TAKEOFF。原步骤的索引和类型不变。
NAVIGATE 从当前位姿重新规划；被打断的 HOVER 先用新轨迹回到原悬停位置，停稳后
才开始剩余悬停时间。恢复 ACK 只表示准入，必须观察绑定的 revision、阶段和最终结果。
`TaskStatus` 增加 `plan_revision`、`can_resume_mission`、`resume_step_index`。
能力标志不替代 PAUSED/健康准入，也不承诺进程重启后恢复。

恢复原任务后的完整着陆结果为 `LANDED_AND_DISARMED`；临时返航直接完成为
`RETURN_LANDED_AND_DISARMED`；明确提前结束的收尾为 `MISSION_ENDED_LANDED_AND_DISARMED`。
提前结束的收尾成功不代表原普通步骤全部完成。

Pocket 未接入时可以先用假飞机测试和独立 SITL。下面两个入口不依赖 USB 输入，
建立自己的 GNSS/DIRECT x500 会话并自动注入临时返航、暂停、恢复原任务：

```bash
./scripts/run_px4_flight.sh --bt --flight-scenario return-resume-mission
./scripts/run_px4_flight.sh --bt --flight-scenario return-hover-resume
```

两个场景须逐个运行，共用独立实例 7/domain 78 的端口。审计保存真实位姿、暂停漂移、
原任务索引、home/根/期限一致性、新 child/generation、BT 步骤授权及最终着陆。
软件和该 GNSS SITL 的结果不认证真实 Pocket 或 VIO/光流来源交接。

对已运行的隔离 BT，可不用遥控器单次发送意图：

```bash
ROS_DOMAIN_ID=78 ./scripts/with_px4_sim.sh bash -e -c \
  'source .deps/mission-install/local_setup.bash; python scripts/bt_operator_intent.py \
   RESUME_MISSION --enable-sitl-commands'
```

此 CLI 只绑定新鲜、唯一 Runner 的当前根，不创建飞行任务或控制发布者，不自动重试。
`START` 仅适用于现有 WAIT_START。接回 Pocket 后，确认 Linux button 0 对应实体自回弹
按键，才给 Pocket 所有者添加 `--return-button 0`；默认仍未分配。该按键切换返航/暂停/
恢复返航；SD 从临时返航 PAUSED 请求 `RESUME_MISSION`，没有检查点时仍请求 `RESUME`。
SB/SC 三档的完整选择器仍是下一节的设计，当前不能声称已检查 SC 的执行档。

## SB / SC 三级选择方案（设计，尚未接入控制）

SB 建议三档为手动、保持、任务：手动档由 SC 选择辅助程度；保持档暂停并限时保持，
SC 不发任务启动/恢复/结束；任务档由 SC 选择任务意图。任务拨到 SB 中档保留进度，
再拨任务档仍须新的 SD 恢复边沿。任务拨到手动档是撤销自动所有权，不保证可恢复旧根。
定位失效时不能进入或维持定点保持；先制动与控制交接，不直接把原生 Hold 模式当作
ROS 根 Action 的暂停确认。当前 SB 的实际解码仍为高档导航、其余手动。

SC 可根据 SB 的主模式解释为不同的三级选择。当前 `PocketSwitches` 只处理 SA/SB/SD，
尚未读取 SC；已观察到的剩余三段输入为 QGC Raw 7 / Linux axis 6，
仍须单独拨动 SC 核对实体对应和上下方向。下表中的低、中、高表示原始输入值，
不预先认定对应实体开关的上、中、下。

| SC 档位 | SB 手动：建议行为 | SB 任务：建议行为 |
| --- | --- | --- |
| 低 | 基础手动：Stabilized，飞手控制油门，无定高/定点；若“纯手动”指不自动回正，则需改选 Acro | 待机：地面不启动；空中暂停并限时保持 |
| 中 | 光流辅助：IMU + 光流 + 下视测距，满足估计器条件后使用 Position 控制 | 执行：地面起飞并执行；已有暂停根恢复；空中加入新根需要独立入口 |
| 高 | VIO 辅助：IMU + VIO + 光流/测距，融合和坐标对齐合格后使用 Position 控制 | 结束：建议停止剩余普通任务、确认制动、返航降落；须明确最终结果的提前结束语义 |

Stabilized 会自动回正姿态；Acro 控制角速度、松杆不自动回正，两者均依赖 IMU，
不能把“纯手动”解释为关闭陀螺仪。见 [PX4 Stabilized](https://docs.px4.io/v1.16/en/flight_modes_mc/manual_stabilized)
和 [PX4 Acro](https://docs.px4.io/v1.16/en/flight_modes_mc/acro)。光流测量需要高度尺度，
须同时检查测距、光流质量、光照/纹理及高度工作范围；仅“陀螺仪 + 光流工作”不足以
承诺稳定悬停。见 [PX4 光流配置](https://docs.px4.io/v1.16/en/sensor/optical_flow)。

手动中档、高档可能使用同一个 PX4 Position 模式，区别在定位能力和融合策略。
切换飞行模式不会自动改变 EKF 的定位来源；若要求中档严格不融合 VIO，需要另行实现
并验证估计器来源交接。建议 VIO/光流持续运行，依据实际融合状态准入，不随 SC
启停进程、重启 EKF 或直接批量改写融合参数。外部定位的配置和坐标约定见
[PX4 外部位置估计](https://docs.px4.io/v1.16/en/ros/external_position_estimation)。

建议保留 SA 为地面解锁、SB 为控制权主选择，SC 为次级状态选择；SD 保留执行/暂停
许可，Buttons 0 建议用于显式返航请求，拨轮预留调速。地面首次启动必须有新的 SD 执行边沿，且 SC
处于执行档；切换 SB、接入 USB 或结束上一根任务时，即使 SC/SD 已保持执行，也不
自动起飞。已有任务 SC 待机或 SD 暂停均请求暂停；恢复要求 SC 执行、SD 允许，并有
新的 SC 或 SD 边沿。SC 结束请求须覆盖普通开始/恢复，不能因 SD 暂停而忽略结束。
返航/结束期间 SC 的普通执行档不得重建原航点序列；已提交原生降落仍不接受暂停。

空中切换的设计边界：

- 手动辅助级别切换：目标模式的姿态/高度/位置估计必须有效；升级检查融合、对齐及
  reset 状态，匹配摇杆/油门含义，从当前状态建立参考。只收到命令 ACK 不算切换完成。
- 自动转手动：唯一网关撤销旧轨迹及租约，以独占过渡状态完成模式交接；实际 PX4
  模式确认后退出自动输出，拒绝或超时走明确的保持/降落故障路径，不自动抢回任务。
- 手动转自动：需新增空中加入根任务的门控和 recipe；不重复执行 TAKEOFF，不改变
  起飞点/home，采用当前位姿和新轨迹代际。当前代码的 `start_task()` 仍要求地面停稳。
- 空中待机：定位合格时制动并保持，有超时和失效收尾；不能停发 Offboard 心跳或解除
  武装。无有效位置估计时不能宣称定点保持，须进入预先验证的降级策略。
- 空中结束：不直接停电机；提前结束、返航、着陆和解除武装分别确认。降落提交后
  继续既有降落契约，不因 SC/SD 改档恢复飞行任务。

实现顺序为 SC 只读三档识别 → 主/次模式与 SD 的状态机 → 原生手动模式实际确认 →
光流/VIO 能力门控与来源交接 → 空中加入/退出任务。验收须覆盖初始高档不启动、
开关抖动/并发优先级、拒绝升级、实际模式确认、源失效/reset、USB 断开、空中待机、
返航暂停及降落不可暂停；先软件/假后端，再独立 SITL，分别记录请求、真实状态和结果。
当前 Pocket 入口是 GNSS/DIRECT 配置，不具备上述 SC 光流/VIO 模式或空中加入新根的
验收证据；本节不改变正在运行的控制配置。

## 连接检查

```bash
python3 scripts/usb_joystick.py
python3 scripts/usb_joystick.py --device /dev/input/js0 --duration 30
```

第一条列出设备、名称及当前用户的读取权限；无设备时退出 2。第二条只读取事件，输出
轴/按钮数量、编号、原始值以及 initial 标志。不发送 ROS、MAVLink 或飞控指令。
若没有 js 设备，先检查数据线、USB 模式和 `/dev/input/by-id`，可检查
`modinfo joydev`；若系统模块未加载，`sudo modprobe joydev` 后重查。
权限不足时依据本机设备权限处理，不运行整个控制适配器为 root。
当前设备的固定路径为
`/dev/input/by-id/usb-EdgeTX_Radiomaster_Pocket_Joystick_00000000001B-joystick`；
连接检查和适配器均可使用该路径，避免其他手柄插入后 js 编号变化。

## 摇杆手动飞行

```bash
./scripts/with_px4_sim.sh python scripts/run_px4_sitl_smoke.py \
  --usb-joystick --ui --duration 600
```

该入口建立独立 x500/GNSS/PX4/QGC 会话，ROS domain 78，随机 Gazebo 分区。
没有 FlightServer、自动任务或自动解锁。用户在 QGC 的 Vehicle Setup → Joystick
选择 Pocket、校准四个轴及方向，核对油门类型，再启用输入；在 Fly 中选择 Position 模式
进行手动飞行，并在会话结束前降落、解除武装。600 秒到期会清理本轮仿真进程。
最终没有新鲜 landed/disarmed 时报告失败；进程清理不代表成功着陆。
手动会话结果只记录观察范围，不认证飞行性能。

本配置固定 `COM_RC_IN_MODE=1`（Joystick），`COM_RC_OVERRIDE=2`（只开放 Offboard
摇杆接管）；普通自动飞行入口保留 `COM_RC_IN_MODE=4`。未改变 USB 丢失、Offboard 丢失
或定位新鲜度阈值。准确含义见 [PX4 1.16 参数](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference#COM_RC_IN_MODE)
与 [QGC Joystick 设置](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/setup_view/joystick.html)。

## 开关控制自动任务

```bash
./scripts/sim.sh px4-flight --bt --ui --usb-joystick --operator-start
# 在同一个工作区另开终端，使用实际核验的 mapping.json：
ROS_DOMAIN_ID=78 ./scripts/with_px4_sim.sh bash -e -c \
  'source .deps/mission-install/local_setup.bash; python scripts/usb_bt_operator.py \
   --device /dev/input/js0 --mapping /absolute/path/mapping.json --enable-sitl-commands'
```

开关适配器同时读取 HID，摇杆仍由 QGC 读取。QGC 中开关不要再绑定 Arm、Offboard 或
其他会与任务请求冲突的动作。本轮临时 QGC 配置独立；每次核对校准和按钮绑定。
启动 QGC 后保持地面、未解锁。Runner 出现 WAIT_START 后最多等 60 秒；拨动 START
才提交一次根任务，后端仍独立验证授权、地面状态、健康和区域。START ACK 不等于解锁。
Runner 结束后不会再次起飞。

以下只展示 JSON 格式，编号不是 Pocket 的默认映射；必须替换为连接检查观察到的值。

```json
[
  {"kind": "button", "index": 0, "value": 1, "command": "START"},
  {"kind": "axis", "index": 4, "value": -32767, "command": "PAUSE"},
  {"kind": "axis", "index": 4, "value": 32767, "command": "RESUME"},
  {"kind": "button", "index": 1, "value": 1, "command": "CANCEL"}
]
```

初始 HID 状态作为基线；插入时按住按钮不会触发任务。新切换稳定 150 ms 后发出一次请求，
持续按住不重发。同时收到 CANCEL 时丢弃其他请求。状态须来自唯一 Runner 发布者且
接收年龄 ≤0.5 s。请求绑定当前 coordinator、根 UUID 与非零 request_id；不重放旧任务。
不自动重连设备或恢复任务。USB 拔出/适配器退出时，向最后观测的根请求取消；
日志只称“请求取消”，真实结果和物理停止仍由根 Action/网关确认。没有事件不等于断线，
静止摇杆不会因为 Linux 只在变化时发事件而被误判失联。

## 决策优先级与边界

逐步骤树增加 `MissionDecision` 装饰器：停止请求优先，已提交原生降落继续等待结果，
暂停/恢复服务请求及 PAUSING/HOLD_HANDOFF/PAUSED 阶段暂停子节点推进，其余阶段继续
当前 Sequence。暂停不 halt 子节点，避免触发该节点的整段取消契约；仍实际 tick 并续
进展租约。后端拒绝暂停时原任务继续；后端接受后须等待根反馈确认 PAUSED。
服务/阶段确认最多 8 秒；反馈失效超过 0.5 秒时请求取消并停止任务续租。地图、定位、
模式和控制权健康由唯一 FlightServer 独立检查，不因开关状态而放行。

`/uav/bt/set_intent`（SetMissionIntent）：START 使用当前实例和 nil 根 UUID，仅适用于
显式等待启动；其余意图绑定已接受根。相同 request_id/命令返回原准入决策，
跨命令复用拒绝；最多缓存 256 条。`/uav/bt/task_status` 是当前根反馈摘要，ACK 不代表
停稳/恢复/清理完成。TAKEOFF/OBSERVE 阶段暂停仍由现有后端契约拒绝；原生降落拒绝取消。

手动接管以 PX4 实际模式退出 Offboard 为准。网关发现模式丢失后撤销旧任务和全部自动
输出，不自动抢回 Offboard。当前返回 ABORTED/CONTROL_LOST，不能把人工接管称为
已确认停稳/成功取消。Offboard 摇杆接管由 PX4 固件负责，USB 实机接管尚未现场验收。
手动飞行后转自动任务须先落地，再通过 SA/SB/SD 建立新的地面任务；Pocket profile
显式允许已解锁且地面停稳的新根，普通 profile 仍要求未解锁，首版拒绝空中加入新根。
取消只取消并保持，受网关既有期限约束；不等于请求立即切换到手动 Position 模式。
现有 full 飞行记录器要求整个 recipe 成功；用户取消或人工接管会保留 CANCELED/ABORTED
并使该完整路线验收失败，不能把它改写为 SUCCEEDED。它会按既有收尾窗口记录最终状态，
随后关闭本轮 UI；需要继续手动训练时重新启动上述无自动任务的手动入口。

此增量不包含主动绕障、运动中重规划、自动探索、检查点重启恢复或无 GNSS 遥控接管验收。
已有 VIO 实验链路入口不接受上述 USB profile，避免把 GNSS 遥控测试混作 VIO 验收。

## 软件验证

```bash
./scripts/build/build_px4_flight.sh --packages-select uav_nav_interfaces uav_bt uav_mission
./scripts/with_px4_sim.sh bash -e -c \
  'source .deps/mission-install/local_setup.bash; PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
   src/uav_bt/test src/uav_mission/test'
```

ROS 决策用例使用独立测试域和假飞机后端；不发布 FMU 输出。软件通过后还需要 Pocket
USB 校准、手动飞行、开关地面启动、暂停恢复、接管及拔线的现场 SITL 证据。
