# MTF-01 光流／测距仿真

[文档总目录](../README.md) · [脚本目录](../../scripts/README.md)

2026-10-13：新增不依赖 VIO／Jetson 的 `FLOW_LOCAL` 独立启动与飞行验证入口，见 [MTF-01 直连与独立控制说明](MTF01_STANDALONE_FLIGHT.md)。下文保留原来的未解锁基线及联合审计；它们各自的历史结论不代表新增飞行验证结果。

2026-10-11：新增独立 `px4-mtf01` 入口。使用 Gazebo 渲染的下视图像计算光流，使用 GPU 单射线测量地面距离，再送入实际 PX4 v1.16.2 EKF。关闭 GNSS 和外部视觉融合；模拟磁力计提供航向。该入口保持未解锁，不发送飞行命令，不配置实机串口。

## 启动与检查

```bash
# 需要已完成 PX4/Agent 构建；Ubuntu 系统 Python 需有 python3-opencv。
./scripts/sim.sh px4-mtf01 --duration 45 --stop-sensors
# 增加本次会话的 Gazebo 查看窗口
./scripts/sim.sh px4-mtf01 --ui --duration 45
# 无纹理地面的负向对照：预期拒绝光流，退出码 1
./scripts/sim.sh px4-mtf01 --no-floor-texture --duration 30
```

`--duration` 是墙钟观察期限，12 秒启动等待包含在其中；`--stop-sensors` 在其后额外观察 12 秒停更。每轮使用独立 Gazebo partition、domain 78、PX4 instance 7、UDP 8898，并检查互斥锁和端口占用。只结束本次创建的进程组，保留其他仿真和实机进程。

证据保存在 `.cache/simulation/mtf01/<UUID>/`：资产与输入 hash、有效参数、原始 uORB listener 日志、结构化样本、渲染器证据、检查结果及进程清理记录。健康检查要求最后连续 5 次快照满足源新鲜度、光流质量、下视方向、有效量程、实际 `fused=true`、创新未拒绝和持续推进的 `time_last_fuse`，不能只凭 `EKF2_OF_CTRL=1` 判定成功。停更检查要求实际融合标志撤销、原始源过期并保持未解锁。

本轮结果及失败对照见 [验证报告](../validation/simulation/2026-10-11-mtf01/REPORT.md)。这些检查验证静止链路；没有执行起飞、悬停、航点或避障。

## 参考模型与数据路径

- 配置：[传感器参数](../../simulation/px4/mtf01/sensors.json)、[独立融合参数](../../simulation/px4/mtf01/fusion.json)。相对机体 FLU 的安装为 `(0.03,0,-0.10)` m，对应 PX4 FRD 偏移 `(0.03,0,0.10)` m；光流朝下，机体 yaw 安装旋转为零。
- 相机 100×100、42°、100 Hz；测距范围 0.02–8 m、100 Hz。光流有效高度限制 0.08–8 m，角流速上限按厂家 1 m 高度时 7 m/s 换算为 7 rad/s。地面和空中质量门限均为 1，拒绝质量零观测。
- `/mtf01/flow/image` → ROS Image → 相邻帧 LK 跟踪／往返一致性检查 → `/px4_7/fmu/in/sensor_optical_flow`。缺少角点时输出质量零；不读真值速度，不假造高质量，也不重写样本时间。相机投影和 PX4 接收时的反号均有图像平移测试。
- `/mtf01/range/scan` → ROS LaserScan → 下视安装适配 → `/px4_7/fmu/in/distance_sensor`。无效、重复、乱序和超量程射线不发布；不会把 8 m 以上数据夹紧为 8 m。原始 Gazebo 时间戳直接保留，DDS 时间同步关闭。
- 室内纹理场景额外提供 0.5 m 的近地打印纹理靶，使地面启动时可观察零运动。它仅用于实际渲染，不提供位置／速度输入。无纹理对照移除该靶和地面随机纹理块。

实现使用 OpenCV LK 和理想单射线，不仿真 PMW3901 芯片算法、MTF-01 固件、UART/MAVLink 协议、LED 光斑、照度、反射率或硬件噪声。100 Hz 是仿真时钟中的配置值，墙钟频率受 Gazebo 实时因子影响。8 m 也不能直接当作可靠飞行高度：厂家将其限定在特定光照和反射率条件；光流还要求足够地面纹理及照度。[MTF-01 厂家手册](https://micoair.cn/zh/docs/sensors/sensors/mtf-01-sensors)

## 加入后是否还需要 OAK-D VIO

| 用途 | MTF-01 的作用 | OAK-D 的作用 |
| --- | --- | --- |
| 低空局部悬停 | 光流提供水平速度约束，测距提供离地高度；条件满足时可不依赖 VIO | 可作辅助，但需要另行验收悬停闭环 |
| nvblox／EGO 地图导航 | 不能提供独立航向、地图定位或消除累计漂移 | 保留深度建图与 VIO 位姿；也可以由经验证的其他定位源提供位姿 |
| VIO 延迟／协方差故障 | 增加独立近地观测来源 | 原有 VIO 源的健康、跳变、连续性和地图对齐检查仍需通过 |

项目当前是三维建图和规划导航，因此保留 OAK-D 深度与 VIO。光流／测距可以减轻近地控制对单一视觉源的依赖，但不会修复 SDK 时间延迟、协方差超限或旧地图缺少深度观测的问题。PX4 光流本身也依赖有效测距和质量条件。[PX4 光流说明](https://docs.px4.io/v1.16/en/sensor/optical_flow)

独立模式不改动已有 VIO 归一化、联合协方差检查、FlightServer 准入或地图定位契约。现有固定 5° VIO 飞行仍未获准。

## 同机联合融合

新增 `px4-vio-mtf01`，在同一架 `x500_vio_ref_7` 上保留固定下偏 5° 双目／IMU，同时安装 MTF-01 参考相机和射线。它运行实际 cuVSLAM 和 PX4 EKF，保持未解锁；不分配实机串口。当前验收使用本机 CPU 0–15、NVIDIA 无界面渲染和下面的显式调度参数：

```bash
./scripts/sim.sh px4-vio-mtf01 \
  --sdk-executor single_thread --sdk-cpu-list 0-15 --sdk-image-depth 1 \
  --real-time-factor .8 --render-device nvidia --headless-rendering \
  --ekf-delay-max-ms 160 --quality-policy bounded_gap \
  --source-warmup-seconds 20 --duration 120
# 使用相同参数，追加以下选项，分别验证辅助源停更：
# --joint-fault flow
# --joint-fault range
# --joint-fault all
```

`--duration` 是启动等待之后的墙钟观察窗口，至少 60 秒，联合入口默认 120 秒；采样时间按 0.8× 仿真时钟推进。默认随后停止标准化 VIO 源并继续观察 12 秒。选择 `flow/range/all` 时，先停止对应的本次适配器、继续观察 12 秒，再停止 VIO 并观察 12 秒。其余源和 ROS 回调持续运行，进程回收放在最终清理阶段。证据写入 `.cache/simulation/vio-sensors/<UUID>/`。

[联合配置](../../simulation/px4/mtf01/joint_fusion.json) 使用 `EV_CTRL=11`（视觉位置／高度／航向）、`OF_CTRL=1`、`RNG_CTRL=2`、`HGT_REF=3`（视觉高度参考），关闭 GNSS 和磁力计融合，保留气压高度辅助。视觉速度不参与融合；光流和测距各自进入专用 PX4 输入，不送回 cuVSLAM，也不将 PX4 融合位姿回灌为视觉观测。测距表示离地距离，辅助地形／高度状态，不能直接代替地图 Z。连续测距融合只用于这次平地仓库审计，斜坡、地面高度跳变和其他飞行条件需要单独验证。[PX4 EKF 高度与测距配置](https://docs.px4.io/v1.16/en/advanced_config/tuning_the_ecl_ekf)

联合观察器显式允许光流／测距辅助，保留 VIO 原始时间、协方差、重置与融合连续性检查；生产 `VioGate` 的默认规则仍拒绝额外辅助源。另检查 MTF 样本和接收时间均不超过 0.2 秒、质量、量程、下视方向和唯一发布者。`/uav/vio/joint_observation_status` 只发布观察状态：

| 状态 | 含义 |
| --- | --- |
| `JOINT_OBSERVATIONS_READY` | VIO 门控通过，光流与测距原始观测均健康 |
| `VIO_OBSERVATIONS_ONLY` | VIO 门控通过，MTF 观测组合不完整或无效 |
| `MTF_OBSERVATIONS_ONLY` | 光流与测距观测均健康，VIO 不可用 |
| `UNAVAILABLE` | 两组均不完整或不健康 |

这些状态始终带 `navigation_authorized=false`、`control_authorized=false`，不代表悬停许可。实际光流／测距融合还必须通过原始 uORB aid source 的 `fused`、创新和 `time_last_fuse` 检查；不能凭原始观测健康就宣称 EKF 正在融合。

联合静止和停更证据见 [联合验证报告](../validation/simulation/2026-10-11-vio-mtf01-joint/REPORT.md)。下一步仍需动态轴向与误差、低纹理／超高／斜坡、VIO 高度方差超限与地图对齐验证，再验收已知区域起飞、悬停、空中故障处置和地图导航。地面停源通过不会解除原有飞行门控，也没有实现空中自动切换。
