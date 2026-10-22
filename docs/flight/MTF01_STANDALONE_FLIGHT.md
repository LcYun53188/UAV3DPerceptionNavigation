# MTF-01 直连飞控：不依赖 Jetson 的纯光流模式

> 日常仿真使用 `./scripts/sim.sh run manual`：完全无 VIO、光流／测距纯手动，默认实体 HID；模拟输入显式添加 `--input-source virtual`。不会自动起飞，自动任务意图被拒绝。下文自动测试属于内部开发回归，已从公共总入口移除；完整项目使用 `./scripts/sim.sh run full`。参见[启动与控制](../simulation/LAUNCHERS.md)。

[文档总目录](../README.md) · [脚本目录](../../scripts/README.md)

`FLOW_LOCAL` 使用飞控 IMU、独立磁罗盘、MTF-01 光流／测距和气压计。MTF-01 通过 UART 直接连接飞控，由 PX4 完成融合和手动 Position 控制；实机这条控制链无需 Jetson、ROS、OAK-D、VIO 状态或 Offboard 心跳。遥控器通过无线接收机接入飞控；Pocket USB 接电脑属于仿真输入。

“纯光流”指不使用 GNSS／VIO 定位，仍然需要陀螺仪、加速度计和可靠航向。MTF-01 不提供航向。SC 可选择 Acro／Altitude／Position：Acro 用角速度控制，Altitude 提供姿态和高度辅助，Position 增加光流水平速度／位置保持。这里切换的是原生控制模式，传感器融合可持续运行；不要把 SC 三段开关直接绑成空中改写 EKF 参数。

## 实机接入，PX4 1.16

MTF-01 供电 5 V、UART 信号为 3.3 V，TX／RX 交叉接入一个空闲 TELEM 端口并共地。模块协议选择 `Mavlink_px4`，支持 Auto 的模块固件也可使用 Auto。所占用的物理 TELEM 端口和 MAVLink 实例需要按飞控当前配置选择，避免与数传或伴随计算机端口冲突。[厂家手册](https://micoair.cn/zh/docs/sensors/sensors/mtf-01-sensors)

以下是地面配置项，不是可直接导入的整机参数文件：

| 参数 | 目标 |
| --- | --- |
| `MAV_1_CONFIG` | 若实例 1 空闲，将其分配给实际连接的 `TELEM n`，然后重启 |
| `MAV_1_MODE` | Normal |
| `SER_TELn_BAUD` | 115200，8N1，`n` 与物理端口一致 |
| `EKF2_OF_CTRL` | 1：启用光流 |
| `EKF2_RNG_CTRL` | 2：持续测距辅助，适用于本次平地参考；坡面／台阶另行验证 |
| `EKF2_HGT_REF` | 2：测距高度参考 |
| `EKF2_EV_CTRL` | 0：关闭外部视觉融合 |
| `EKF2_GPS_CTRL` | 0：关闭 GNSS 融合 |
| `EKF2_MAG_TYPE` | 1：持续磁航向融合，要求独立磁罗盘健康；需重启 |
| `SENS_FLOW_ROT` | 与实际安装方向一致；No rotation 仅适用于未旋转安装 |

厂家提供 PX4 1.14–1.16 的串口和光流／测距配置；EV、GNSS、磁航向项是本项目独立启动方案。[厂家配置说明](https://micoair.cn/zh/docs/sensors/sensors/mtf-01-sensors) 磁融合与高度参考的含义以 [PX4 1.16 参数文档](https://docs.px4.io/v1.16/en/advanced_config/parameter_reference) 为准。

重启后先在 QGC 确认 `OPTICAL_FLOW_RAD`、`DISTANCE_SENSOR`，再确认实际 EKF 光流／测距融合、稳定本地速度／高度及磁罗盘健康。消息出现不等于融合成功。室内纹理、照度、安装偏移和质量门限均需实机检查；不要直接复制仿真的 `OF_QMIN=1`、单 EKF 或 0.10 m 安装偏移。8 m 是厂家特定反射率／照度下的量程，不是通用飞行高度。[传感器规格](https://micoair.com/optical_range_sensor_mtf-01/)

## 独立仿真入口

```bash
./scripts/build/build_px4_flight.sh --packages-select uav_nav_interfaces uav_mission uav_bt
# 显式执行已知区域低空起飞、8 秒悬停和 PX4 原生降落；不需要 Pocket。
./scripts/simulation/environment.sh mission python scripts/simulation/flow/run_px4_mtf01.py --flight --duration 35 --ui
# 原生 Position＋模拟接收机：全程没有 Offboard／轨迹／VIO 发布者。
./scripts/simulation/environment.sh mission python scripts/simulation/flow/run_px4_mtf01.py --flight --duration 35 --native-manual --ui
# 保留原来的未解锁光流／测距与停源审计。
./scripts/simulation/environment.sh px4 python scripts/simulation/flow/run_px4_mtf01.py --duration 45 --stop-sensors
```

新入口先完成连续地面融合审计，只有通过才启动飞行观察器。观察器要求新鲜唯一的原始传感器、原生成功融合样本、IMU／磁航向对齐、本地状态和时钟；不订阅 VIO 或 EV aid。当前 Mission Gateway 只接纳起飞／局部悬停／降落；地图导航、返航和原任务恢复均拒绝。飞控以测距为高度参考时，`dist_bottom_valid` 表示地形状态的可观测性，独立模式以原始下视距离及实际测距高度融合确认高度，不将地形状态冒充距离观测。

`--native-manual` 使用独立原生 Position 观察器代替 Mission Gateway，模拟接收机发出的最低油门、上升和中位摇杆；地面检查通过后请求 Position／解锁，中位悬停 10 秒，再请求原生 LAND。悬停期间没有控制命令流，所有 Offboard／轨迹／外部视觉／VIO 发布者数量都必须为零。这个 ROS 摇杆源只是实机接收机的仿真替身；未测试实际 Pocket 无线链路。

PX4 的 `heading_good_for_control` 表示最终空中磁航向对齐；地面可能为 false。独立局部控制检查原生 `cs_tilt_align`、`cs_yaw_align`、磁融合、航向一致性和方差。后续坐标／航向重置仍立即撤销当前控制会话，不能自动接受新原点继续任务。仿真使用本机绘制下视图像与测距，传感器适配器是模拟 MTF 的数据源；这不意味着实机需要伴随计算机。

## Pocket 持续操作会话

```bash
./scripts/sim.sh run manual --session-duration 3600 \
  --device /dev/input/js0 \
  --qgc-config .cache/simulation/sitl/<原会话 UUID>/qgc-config/QGroundControl/QGroundControl.ini
```

公共手动程序使用运行器默认的地面融合审计窗口；`--session-duration` 是通过后的操作时长，到期后仍等待落地上锁再关闭。内部回归运行器的 `--duration` 单独控制地面审计窗口。
该入口与自动飞行 `--flight`、`--native-manual` 互斥，启动不解锁、不起飞。
关闭 GNSS／EV 融合，使用原生 IMU、磁航向、渲染光流与下视测距。
生成 14×14×6 m 纹理仓库，包含 14 个墙体、隔断、门洞、立柱、货架和箱体障碍，
不修改上游 Gazebo 场景。当前未接入自主避障，复杂场景不等于避障通过。

默认使用实体 Pocket；模拟面板需添加 `--input-source virtual`，在 QGC 选择并启用 `Pocket Virtual UAV AETR`。
可通过 `--qgc-config` 继承校准；实体校准不能作为虚拟输入校准。
QGC 必须离开 Joystick 配置页回到 Fly 页面，才发送操纵数据。
手动光流操作使用 **SB 手动（−1）、SC 光流（0）**，PX4 原生 Position。
按原定义，SB 定点档与 SC VIO 档要求 VIO；本轮纯光流会话没有 VIO，会明确拒绝。
传感器融合持续运行，开关选择控制模式，不在空中修改 EKF 参数。

唯一 Pocket FlightServer 在地面审计通过后启动，继承并核对本轮资产、domain、
分区和融合配置；不启动独立模拟接收机，不自动解锁。自动档的演示任务仅包含
局部起飞、悬停、原生降落；返航／地图航点继续被 `FLOW_LOCAL` 门控拒绝。
手动和定点档 SD 均请求原生慢速 LAND，使用本轮局部位置与高度，不依赖全局地理定位／Home。纯光流档不具备地图避障返航能力。SE 保留短按暂停／继续与长按取消协议。
无 GNSS 模式的低空原生降落参数为 `MPC_LAND_SPEED=0.3`、`MPC_LAND_CRWL=0.2`。

审计记录位于 `.cache/simulation/mtf01/<UUID>/ground-audit.json`、`receipts.json`、
`parameters.json`、`renderer.json`；持续会话信息见 `live-session.json`。
检查实际 `time_last_fuse` 递增、光流／测距成功融合，以及 GNSS／EV 关闭；
消息出现或 QGC 窗口打开不算融合通过。静止审计通过不代表飞行或避障验证。

## VIO／光流选择的边界

| 配置 | 水平定位 | 高度／航向 | 是否依赖 VIO |
| --- | --- | --- | --- |
| `VIO_FLOW` | VIO 位置＋光流速度 | VIO，测距辅助 | 是 |
| `FLOW_HORIZONTAL` | 光流 | VIO，测距辅助 | 是 |
| `FLOW_LOCAL` | 光流 | 飞控磁航向＋测距／气压高度 | 否 |

`SelectLocalization` 当前只支持前两项，要求未解锁、落地、静止、无活动任务、无已绑定地图；参数回读之后还需实际融合连续稳定。新开 EV 水平位置时，锁定的 PX4 1.16 EKF 会重置 XY，所以不能声称支持空中无缝双向切换。`FLOW_LOCAL` 是独立启动配置，与 VIO 配置之间需要落地停机、配置／重启并重新建立本地会话和地图。

地面切换回归使用固定独立 EV 测试源、实际渲染光流／测距及原生 EKF，始终不解锁；它不验证相机 VIO 精度。普通 `select_localization.py` 会拒绝这种 mock 会话。复现：

```bash
./scripts/with_px4_sim.sh bash -e -c 'source .deps/mission-install/local_setup.bash; python scripts/simulation/flow/run_fusion_switch_sitl.py --duration 90'
```

原生 Acro／Altitude／Position 模式可在 PX4 接受该模式且估计健康时空中切换；这与修改定位融合配置不同。光流丢失时由 PX4 原生位置失效逻辑降级，不能依靠缺席的 Jetson 接管；需要按整机遥控、位置失效和电池配置完成实机验收。[PX4 Position 模式](https://docs.px4.io/v1.16/en/flight_modes_mc/position)

已通过无 VIO／GNSS 的 PX4 起飞／悬停／原生降落，以及无 Offboard 心跳的原生 Position 悬停；两轮最大真值漂移分别为 0.0716 m 和 0.0755 m。地面两次往返切换也通过。MTF-01 UART、实机磁干扰、无线遥控、空中传感器失效处置和实机飞行性能仍需验证。记录在 `.cache/simulation/mtf01/<UUID>/`，见 [本轮验证报告](../validation/simulation/2026-10-13-flow-local/REPORT.md)。
