# PX4 定位输入门控（S3 首批）

在连接地图规划器之前，W0 网关同时检查 PX4 本地位置与 VehicleOdometry，后者是
`/uav/px4/odometry` 和 `odom → base_link` TF 的来源。只检查 VehicleLocalPosition
不能证明规划器所依赖的里程计/TF 仍然有效。

## 数据与会话约束

- VehicleOdometry 必须通过已有 NED/FRD → ENU/FLU 转换，未知帧、非有限值或零
  四元数使健康门控失败。未知协方差仍采用已有保守处理，不视为零误差。
- 接收年龄、发布源时间年龄、timestamp_sample 采样年龄均受 W0 的 0.1 s 定位门限
  约束；未来时间容忍仍为 0.05 s。采样时间必须非零，新发布不能掩盖旧测量。
- 任务接受时绑定六个计数：xy/z/vxy/vz/heading reset 及 VehicleOdometry.reset_counter。
  任一计数变化（包括 uint8 回绕）在输入回调中锁存 LOCALIZATION_RESET，撤销参考
  控制权，根返回 ABORTED、cleanup_confirmed=false。后续有效消息不能清除锁存。
- 重置事件后不再发布动态里程计/TF；已有 TF 缓存不能被 ROS 撤回，下游仍须检查时间
  与会话。当前恢复方式是新建监督器会话、重新建立对齐，不自动恢复旧任务。
- 无任务/保持所有权的启动阶段允许估计器收敛产生重置，首次任务绑定当前基线。
  已完成根的有界保持期间发生重置，撤销保持所有权，保留原根终态并记录单独故障。
- 只读 AircraftState 聚合器补充 vxy/vz reset 检查；聚合器保持既有独立锁存策略。

定位失效时停止本机 Offboard 输出，由冻结的 PX4 Offboard-loss 策略接管，不能宣称
网关已确认安全停稳。因此 Action 清理结果为 false；后续是否着陆/解除武装由仿真
独立物理观测审计。不得把这些 SITL 参数应用于 USB 实机。

## 可执行回归

```bash
./scripts/build_px4_flight.sh
./scripts/sim.sh px4-flight --bt --ui --flight-scenario full
./scripts/sim.sh px4-flight --bt --ui --flight-scenario odometry-stale
./scripts/sim.sh px4-flight --bt --ui --flight-scenario odometry-reset
```

运行前关闭独立 QGC，释放 UDP 14550；监督器的 QGC 使用独立配置且禁用 USB 飞控
自动连接。算法仿真仍在 domain 68，PX4 回归在 domain 78 与随机 Gazebo 分区。

`odometry-stale` 在 NAVIGATE 中移除网关的 VehicleOdometry DDS 订阅，PX4 与独立
监督器保持运行，验证真实接收流消失；不表示 PX4 发布器或物理传感器损坏。
`odometry-reset` 向网关接收回调注入一个 reset_counter 增量样本，验证接收协议、
任务失败和实际 PX4 fallback；不是实际触发 EKF 重置，也不发送 estimator 参数/命令。

两种故障回归要求根失败、Runner 退出 1、输出停止且后续步骤未授权、检测不超过
0.5 s，并观察真实 x500 最终着陆解除武装。计数与健康边界另由确定性测试覆盖。
此工作不包含 EGO/PX4 的轨迹执行接线、地图会话迁移、动态对齐或完整感知避障验收。
