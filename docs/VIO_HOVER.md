# OAK-D Pro W 辅助室内悬停：首批实现

目标是相机双目/IMU产生连续 VIO，经 PX4 EKF 外部视觉融合，再由现有 BT 调度
位置 Offboard 起飞、悬停和降落。VIO 计算不放在行为树节点内，飞控输出仍由
FlightServer 独占。定点悬停不以 nvblox 或 EGO 建图规划为前提。

**目前已实现输入适配与拒绝门控，尚未完成真实 VIO 悬停闭环。** 当前工作站 USB
枚举未发现 OAK-D Pro W；首批 DDS 测试使用明确标注的合成输入，不能算相机/VIO 验证。
PX4 固定构建尚未导出门控需要的全部 EKF 遥测；保持未就绪是预期行为。

## 输入适配

`px4_comm_bridge/vio_input_node` 订阅原始 `/visual_slam/tracking/odometry` 与
`/visual_slam/status`，要求原始 odom/base_link ENU/FLU 输入和 vo_state=1。
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

所有遥测接收与源年龄 ≤0.5 s、未来容忍 0.05 s。`cs_ev_*` 表示融合意图，不能单独
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

后续按最小闭环推进：

1. 单独冻结 VIO 仿真传感器/标定与 PX4 配置，补充真实双目/IMU输入；上游 OakD-Lite
   深度模型不能代表 Pro W 双目/IMU，也不能拿 Gazebo 真值替代 VIO 输出。
2. 为单独的 PX4 VIO 构建导出 selector 与四个 EV aid source。默认固定 DDS 配置
   缺失这些 topic，本批不修改原 W0 构建/版本锁，也不伪造融合反馈补齐门控。
3. 在新的固定配置中验证坐标、时间、实际 EV 融合、定位 reset 与初始安全体积，明确
   GNSS/其他定位辅助来源，防止由隐藏定位源承担悬停。
4. 完成本机真实 VIO 的定点悬停、漂移与故障验收；有相机后进行未解锁台架测量，
   最后再进入独立的实机受控飞行阶段。

首批结果与适用边界见 [验证报告](validation/simulation/2026-10-08-vio-admission/REPORT.md)。
