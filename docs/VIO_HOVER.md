# OAK-D Pro W 辅助室内悬停

目标是相机双目/IMU产生连续 VIO，经 PX4 EKF 外部视觉融合，再由现有 BT 调度
位置 Offboard 起飞、悬停和降落。VIO 计算不放在行为树节点内，飞控输出仍由
FlightServer 独占。定点悬停不以 nvblox 或 EGO 建图规划为前提。

**已实现输入适配、拒绝门控，以及独立 PX4 构建的实际 EKF 融合审计；尚未完成真实 VIO 悬停闭环。**
合成输入审计仅验证外部视觉到 EKF 的融合与输入停更检测，不能算相机/VIO 验证。
原 W0 构建保持不变；新增遥测仅由单独的 VIO 审计构建提供。

## 输入适配

`px4_comm_bridge/vio_input_node` 订阅标准输入 `/uav/vio/odometry` 与
`/visual_slam/status`，要求 odom/base_link ENU/FLU 输入和 vo_state=1。
`source_contract` 默认 `unverified`，有数据也拒绝就绪；只有核验源的机体速度、
观测协方差和坐标约定后才可显式声明 `standard_enu_flu_odometry_v1`。
声明是受信任本机配置契约，不是自动完成标定或语义验证。
明确拒绝将 `/visual_slam/tracking/odometry` 作为输入：本机上游版本的 pose/twist
协方差是滑窗样本统计，静止时可为零、也可为很小的正值，通过数值检查不证明
观测精度；速度来自旧窗口机体系的相对位姿差，不等同于当前机体系瞬时速度。
SDK 位姿协方差另外发布在 `/visual_slam/tracking/vo_pose_covariance`。已针对锁定版本
完成右扰动到世界固定轴的位姿协方差归一化；SDK rig 由 base_link 外参定义，参考点
即 base_link。尚无经过核验的速度观测协方差或完整标准 Odometry 发布者，
真实 VIO 不会据此获准飞行。
不用重发旧位姿的 hold 输出作为 VIO 输入，也不能把 PX4 回读里程计回灌到自身 EKF。

- 原始采样年龄 ≤0.2 s，未来容忍 0.05 s；跟踪接收/源年龄 ≤0.2 s，图像定位与
  跟踪状态时间戳差 ≤0.05 s。两个源各要求唯一发布者。
- 检查有限值、单位四元数，以及完整 pose/twist 协方差的对称性与半正定性。
  位姿六轴、线速度三轴方差须 >0 且 ≤0.25；未知或零置信度不当成完美观测。
  这些是首批保守软件门限，尚未完成 Pro W 测量标定，不能宣称覆盖实机性能包线。
- 输入时间不递增、采样间隔 >0.2 s、发布者 GID 更换、位置跳变 >0.05+3·dt m 或
  姿态跳变 >0.1+3·dt rad 均锁存，须新适配器实例重新建立源会话。
  等价四元数正负号翻转不算姿态跳变。
- 本机 Jazzy 的接收 MessageInfo 未提供 publisher_gid 时，使用唯一 DDS endpoint GID
  绑定源；这属于受信任本机图的连续性检查，不是网络身份认证。
- 适配器每实例有独立 localization_session；发布结构化 `VioStatus`，保留原始
  sample_stamp。定时发布状态不会刷新原始位姿的采样时间。
  cuVSLAM 原始状态不包含显式 reset 计数；当前适配器计数为 0，依靠上述连续性检查，
  尚不能证明检测所有小幅内部 reset。后续须统一代理 reset 服务并撤销原会话。
- 外部视觉转换为 NED 位置、FRD 机体到 NED 的 Hamilton 四元数、BODY_FRD 速度和角速度，
  同时变换位置/姿态/速度方差。PX4 消息只能携带方差向量，不能保留完整交叉协方差。

默认只监视，不发布任何 FMU 输入或飞行命令：

```bash
./scripts/build_px4_flight.sh
# 与 VIO 源使用相同 ROS_DOMAIN_ID；传入审核过的设备标定/安装/配置清单 SHA256。
# 默认 unverified 仅作拒绝检查；标准化源完成审核后才显式声明 source_contract。
./scripts/run_vio_monitor.sh --ros-args -p calibration_id:=<64位SHA256>
```

calibration_id 是配置绑定标识，不是设备标定已经正确的证明。必须核对对应设备的
CameraInfo、双目/IMU外参和 base_link 安装外参；现有旧相机参数及 EEPROM 读取失败后的
默认回退不能作为 Pro W 飞行标定。本入口不自动启动相机，不把默认外参当作已审核。

`emit_px4:=true` 只有在受管 SITL nonce、domain 78、隔离 partition、use_sim_time=true
同时满足时才允许创建 `/px4_7/fmu/in/vehicle_visual_odometry` 发布者，并要求该输出唯一。
它不发布解锁、模式、轨迹目标或 Offboard 心跳。硬件 PX4 时间映射尚未实现，因此当前
明确禁止向实机发外部视觉数据。监视模式可使用系统时间检查 ROS VIO 源。

## 行为树/后端门控

`FlightServer` 新增显式可选 VIO 门控，复用同一根 Action、逐步骤 BT、控制世代、
取消与期限机制；没有独立第二个 FMU 控制器。入口为：

```bash
./scripts/sim.sh px4-flight --bt --require-vio --vio-calibration-id <64位SHA256>
```

该命令当前用于验证缺失输入时拒绝飞行，不是已完成的 VIO 悬停演示。普通 W0 入口
不要求 VIO，继续作为既有控制回归基线，不能将其成功称为 Pro W 悬停。

门控要求以下数据均有唯一发布者且新鲜，连续满足至少 2 s 后才通过：

| 输入 | 约束 |
| --- | --- |
| `/uav/vio/status` | valid=true，odom，标定 ID 匹配，非空会话，原始 sample_stamp ≤0.2 s |
| `estimator_status_flags` | EV 位置/高度/速度/航向启用，无相关拒绝/故障，无惯性航位推算或假位置/高度 |
| `estimator_selector_status` | 当前 primary_instance=0、实例健康，无 IMU 故障；当前实现只接受实例 0 |
| `estimator_aid_src_ev_pos/hgt/vel/yaw` | 实例 0、fused=true、未拒绝、有限 test_ratio∈[0,1)，原始观测与 last_fuse ≤0.5 s，观测与当前 VIO 时间差 ≤0.2 s |

`estimator_status_flags` 和 selector 在固定 PX4 1.16.2 中约 1 Hz 或变化时发布，
两者接收与源年龄要求 ≤1.5 s；其余遥测 ≤0.5 s，未来容忍均为 0.05 s。
实际 VIO 原始采样仍要求 ≤0.2 s，四类融合样本/最后融合时间仍要求 ≤0.5 s。`cs_ev_*` 表示融合意图，不能单独
证明观测已融合。VIO 会话/reset/标定变化、已绑定后的 tracking invalid、主 EKF 切换
在接收回调中锁存，后续健康消息不能擦除。ROS 时钟回退或停滞也锁存；启动前替换
未绑定的源会重新开始 2 s 稳定窗口。

门控适用于根任务准入、恢复、步骤推进和运行中的健康检查。飞行中失效复用后端
撤销控制/停止输出的故障路径，交给固定的 PX4 Offboard-loss 策略处置；这不代表
无定位时仍能定点保持，也未验证实机失定位降落包线。原生降落不会因为此门控重新
请求 Offboard。新门控不修改实机参数或绕过解锁检查。

## 验证与剩余工作

合成 DDS 契约测试（仅消息流，不是真实相机或真实 EKF 融合）：

```bash
./scripts/with_px4_sim.sh bash -c 'source .deps/mission-install/local_setup.bash; source install_uav/isaac_ros_visual_slam_interfaces/share/isaac_ros_visual_slam_interfaces/local_setup.bash; python scripts/run_vio_transport_smoke.py'
```

独立的未解锁实际 EKF 融合审计：

```bash
./scripts/build_px4_vio.sh --jobs 4
./scripts/build_px4_vio.sh --check
./scripts/sim.sh px4-vision-audit --duration 35
# 可追加 --ui 显示 Gazebo；QGC 由受管入口启动。
```

构建输出 `.deps/px4-vio-build`，使用锁定 PX4 上游 DDS 生成器，保留原发布/订阅，
只新增 selector 与四类 EV aid 遥测；构建前后验证原 W0/Agent 二进制 hash。
构建清单绑定配置、生成器、模板、工具、生成头文件与二进制，运行前再次检查。
不修改上游源码、原 W0 构建或版本锁。需要先完成基础 PX4 和任务消息构建。

审计采用独立固定零位姿/速度的合成输入，共用 Gazebo 时钟，经正式坐标转换发布
30 Hz external vision；不读取 PX4 估计或 Gazebo 真值生成观测。使用合成标定 ID，
不运行相机、cuVSLAM、BT 或 FlightServer，不发布解锁、模式、轨迹或 Offboard 心跳。
参数为 EV_CTRL=15、GPS_CTRL=0、MAG_TYPE=5、HGT_REF=3、SENS_IMU_MODE=0、
MULTI_IMU=1、MULTI_MAG=0，保留气压高度辅助。审计要求真实四类 fused 样本、
门控连续健康 2 s 后就绪，以及全程未解锁。停止输入后观测 6 s：先排空 1 s，
随后要求四类 last_fuse 不再推进，并且门控因源过期拒绝。归档同时核对 ULog 生效参数。
这不验证运动中的定位误差、飞行控制或失定位降落，也不是无气压辅助的纯视觉高度验证。

本机 cuVSLAM 节点已通过独立构建、CUDA 加载及模拟双目／IMU静止跟踪检查。
可用 `./scripts/build_vio_node.sh` 构建节点，再通过 domain 94 的
`scripts/run_vio_node_smoke.py` 验证加载。该入口不构建可选图像预处理包，
不代表其 CV-CUDA 依赖已修复。见 [节点加载报告](validation/simulation/2026-10-08-vio-node-load/REPORT.md)。

```bash
./scripts/sim.sh px4-vio-sensors --ui --duration 35
```

新入口在独立 partition/domain 78、PX4 instance 7 中运行 PX4、Gazebo、QGC、
双目／IMU桥接与实际 cuVSLAM；不启动 FlightServer，不发布 FMU 输入。
模型和有纹理场景按固定配置生成到本次缓存，不改上游 x500 或 W0 构建。
参考配置为理想针孔 640×400、FOV 1.21 rad、基线 0.075 m、25 Hz，理想 IMU
250 Hz，安装点 FLU (0.12,0,0.242) m。没有 Pro W 广角畸变、噪声、曝光、
USB 延迟或硬件时间映射；cuVSLAM 默认 IMU 噪声参数是算法假设，非该相机实测。
验收覆盖最近 5 s 的采样率、间隔、双目配对、CameraInfo、TF 配置、IMU 重力方向、
持续跟踪、漂移、源新鲜度、唯一发布者、未解锁着地与进程清理。
结果中的 raw_odometry_numeric_conversion 仅演示原始数值检查，不能用于飞行准入。
详见 [实际传感器跟踪证据](validation/simulation/2026-10-08-vio-sensors/REPORT.md)。

后续按最小闭环推进：

1. 冻结 VIO 仿真双目/IMU传感器、内外参、时间与安装配置；上游 OakD-Lite 深度
   模型不能代表 Pro W 双目/IMU，也不能拿 Gazebo 真值替代 VIO 输出。
2. 接入本机真实 VIO 算法，并完成源 reset 代理与轴向/时间/漂移验证。
3. 在独立飞行配置中接入现有 BT，验证无隐藏定位辅助的起飞、定点悬停、返航、
   降落与定位退化处置；当前合成静止输入不得用于飞行。
4. 有相机后进行未解锁台架标定和时间映射测量，最后进入独立实机受控飞行阶段。

输入首批结果见 [准入验证报告](validation/simulation/2026-10-08-vio-admission/REPORT.md)。
实际 EKF 融合结果见 [遥测审计报告](validation/simulation/2026-10-08-vio-telemetry/REPORT.md)。

## SDK 位姿归一化与 reset 代理

```bash
./scripts/build_px4_flight.sh
./scripts/sim.sh px4-vio-sensors --normalize --ui --duration 35
./scripts/sim.sh px4-vio-sensors --normalize --reset-source --ui --duration 40
```

`cuvslam_pose_node` 使用 SDK 位姿协方差，不读取原始 Odometry 的滑窗统计。
对右扰动协方差使用 diag(R,R) C diag(R,R)ᵀ，保留完整位置/姿态交叉项；
发布 `/uav/vio/pose`（PoseWithCovarianceStamped）与 `/uav/vio/pose_status`（VioStatus）。
保留 SDK 原始采样时间和位姿，不推导速度，不发布 FMU 输入，也不接入现有飞行门控。
这里的 odom 仍为 SDK 初始局部坐标系，尚未实现对 PX4 的初始化对齐或实机时间映射。

须显式声明 `cuvslam15_right_tangent_base_link_v1`；受管入口另外核对
`simulation/px4/vio/pose_contract.json` 的 SDK/header/上游转换及节点二进制指纹。
节点查询并核对原生 base_frame、odom_frame、VIO-only、无地面约束及不覆盖采样时间。
calibration.json 绑定参考传感器、TF、算法参数及原生实现 hash，摘要作为配置 ID；
这不是 Pro W 标定证明。

位姿/状态按原始时间戳配对，缓存不续写旧时间，0.2 s 内缺失配对就拒绝。
首次绑定要求连续健康 2 s，初始化的零/单位协方差会拒绝并重新计时；绑定后不自动恢复。
姿态/位置/时间跳变、发布者变化、源过期、tracking invalid 或协方差超限均锁存失效。
同一源内恢复健康也不能复活原授权。

原生 reset 服务由受管入口重映射至 `/visual_slam/internal/reset`；公开入口
`/uav/vio/reset` 先递增代理计数、立即失效旧源，再转发实际 SDK reset。
服务失败、缺失或 5 s 超时也不恢复源；期限与源接收时效使用单调时钟，暂停仿真时钟
不会阻止超时。SDK 后续输出恢复也不再发布旧源位姿；重建须启动新适配器会话。
该重映射是受信任本机运行约定，不是 ROS 访问控制；计数只统计代理请求，
不承诺检测全部 SDK 内部小幅 reset。

35 s 静止与 40 s 实际 reset 前置审计通过；90 s 稳定性检查失败：SDK 在约 48 s
将竖直位置方差给到约 0.375 m²，超过 0.25 m² 门限，适配器按设计永久停止输出。
尚未确认 SDK 质量变化的根因；静止初始化缺少激励、场景特征与算法噪声假设仍需
通过独立运动数据评估，不能当成已证实原因。保持全部门限，不宣布 S6 或 VIO 悬停通过。
完整数据见 [位姿与 reset 验证](validation/simulation/2026-10-08-vio-pose/REPORT.md)。

下一步先冻结独立运动验证配置与初始对齐，仅将真值用于审计三轴/旋转误差。
再实现显式位姿融合配置：速度观测缺失时不伪造速度或协方差，不能直接使用现有要求
四类 EV 融合的门控；必须分别核验已提供观测的真实融合及 PX4 估计速度，再进入 BT 飞行。


## 独立载台的实际 VIO 运动验证

```bash
./scripts/build_vio_motion.sh
./scripts/sim.sh px4-vio-sensors --normalize --motion --scene layered --duration 120
# 对照：相同启动顺序、传感器与运动轨迹，仅使用原平面纹理场景。
./scripts/sim.sh px4-vio-sensors --normalize --motion --duration 90
# 两种场景均可追加 --ui。
```

新增动态 `vio_motion_carrier`，质量和惯量交给 Gazebo 物理系统；插件通过力/力矩
驱动三轴平移和转向，不设置位姿或速度，不由 PX4 控制。插件读取自身物理状态
进行载台反馈控制；`/vio/truth` 只供审计，VIO 仍只消费双目、CameraInfo、IMU 与
安装 TF。独立官方 x500/PX4 保持未解锁着地，四类 FMU 输入发布者均为零。

参考轨迹在仿真 15 s 后开始，三轴峰峰位移约 1.0/0.8/0.6 m，航向约 0.8 rad。
它是传感器运动台架，不验证多旋翼气动、PX4 闭环控制或真实 Pro W 性能。
`layered` 场景在原墙面外增加固定随机的多深度立体特征及地面纹理；配置、生成
模型/场景、插件源码/二进制与标定清单均记录指纹。启动先等待传感器、PX4 和
QGC 就绪，再启动 VIO，避免把载台先于 PX4 出现带来的初始化负载当作稳定源。
既有 0.2 s 新鲜度、2 s 连续健康及 0.25 协方差门限不变，绑定后仍锁存失效。

误差审计按原始仿真采样时间插值真值，只在首个标准位姿作一次刚体对齐，随后
固定变换；不估计尺度、不逐段重对齐。检查三轴/航向运动、IMU 响应、连续覆盖、
位置 RMSE ≤0.15 m、最大误差 ≤0.30 m、姿态最大误差 ≤10°，以及源/协方差健康。
原始 SDK 误差单独记录，即使误差达标也不能替代标准源的置信度准入。

多深度场景已有持续运动通过结果；原平面场景仍因高度方差超限停止标准输出。
这支持继续使用有深度差异的测试场景，尚不能将 SDK 不确定性退化归因于单一因素。
结果、对照失败、启动间隔诊断及离线重放见
[运动验证报告](validation/simulation/2026-10-09-vio-motion/REPORT.md)。
下一步是显式位姿融合配置与初始化对齐，再验证实际 EKF 和 BT 悬停；当前未发布
PX4 EV，也未改变现有默认四类融合门控或伪造速度观测。
