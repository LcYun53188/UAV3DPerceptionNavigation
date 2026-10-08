# 独立 PX4 VIO 遥测与实际融合审计（2026-10-08）

本批完成单独 PX4 1.16.2 遥测构建、实际 EKF 外部视觉融合、持续准入和停止输入后的
拒绝验证。**输入是独立固定合成位姿，尚未完成相机、真实 VIO 算法或室内飞行验收。**
未向实机发送外部定位或控制数据；原 W0 构建与版本锁保持原样。

## 最终结果

最终源码 run `ef1ae637-90fc-4285-8413-c4406286370e`，35 s 输入窗口后停止输入并
继续观测 6 s，supervisor exit 0，`final-observation.json` 与 `final-vision-fusion.json`
均 passed=true：

- 合成 external vision 发布 913 次。真实 PX4 的 EV position/height/velocity/yaw
  各收到 **707 个 fused=true 样本**；没有用伪造 aid source 补齐门控。
- VioGate 连续健康至少 2 s 后就绪，停止前仍为 READY；停止后最终为
  `VIO_TELEMETRY_STALE:source`。ready_count 是检查次数，不能换算为时间或独立观测数。
- 四类 last_fuse 在停止瞬间均为 32556000 µs；排空 1 s 后为 32740000 µs，
  窗口结束仍为 32740000 µs。允许已排队的观测完成融合，不把停止瞬间的排队融合
  误判为持续提供新观测。最后一条 aid 消息仍可能 fused=true，必须同时检查时间。
- 七类源/遥测各有唯一发布者。vehicle_command、trajectory_setpoint、
  offboard_control_mode 发布者均为 0；不启动 BT/FlightServer。
- 观测到的 arming_states 仅为 DISARMED，着地。GNSS 位置/速度/航向/高度、
  磁航向/三轴磁、光流辅助均未启用；气压高度辅助 **仍启用**，不声称纯视觉高度。
- `final-effective-parameters.json` 从本次 ULog 提取并核对：EKF2_EV_CTRL=15、
  EKF2_GPS_CTRL=0、EKF2_MAG_TYPE=5、EKF2_HGT_REF=3、SENS_IMU_MODE=0、
  EKF2_MULTI_IMU=1、EKF2_MULTI_MAG=0、UXRCE_DDS_SYNCT=0。
  保存了原始 ULog 路径/hash；完整 ULog 留在运行缓存，未复制 rootfs 到此归档。

固定零位姿/零速度观测由 ROS 消息构造，经正式 ENU/FLU→NED/FRD 转换进入 PX4，
共用 Gazebo 时钟，输入约 30 Hz。不读取 PX4 估计或 Gazebo 真值生成输入。
合成 calibration_id=`aaaa…` 仅用于契约测试，不代表任何设备标定。

## 构建与门限

`.deps/px4-vio-build` 使用锁定上游生成器与模板，导出 selector 和四类 EV aid，
其余 DDS 发布/订阅与基线一致；构建器拒绝任何额外订阅或控制 topic 变更。
构建前后核对原 W0/Agent 二进制，W0 PX4 SHA256 保持
`5525e3edf3413d57039f374be8b934bbf53390de41edc54eb6ced15ed0aa72da`。
新二进制/生成头文件/输入 hash 见 `vio-build.json`，运行前 `--check` 已通过。

实测并对照固定上游 `EKF2.cpp:1866` 和 `EKF2Selector.cpp`：flags/selector 约
1 Hz 或变化时发布。两者源与接收年龄限改为 1.5 s，其余遥测保持 0.5 s，
VIO 原始采样保持 0.2 s；四类实际融合时间检查未放宽。新增单测验证正常 1 Hz
不会打断 2 s 稳定窗口，超过门限仍拒绝。

原构建 run `2b6c4413-460a-430a-91ee-1ccb74ba1704`，20 s 默认未解锁
DDS/clock/QGC 冒烟回归 exit 0 / PASS，见 `baseline-*`。本批没有重跑完整 W0
飞行；上一批完整飞行结果保留在 [首批准入报告](../2026-10-08-vio-admission/REPORT.md)。

源码/审计回归 **269 passed / 0.64 s**，见 `pytest.xml`，覆盖桥、任务后端、
接口兼容、只读 DDS 扩展、审计失败判定与既有飞行/深度审计。
额外 BT 生命周期回归 **37 passed / 107.22 s**，见 `bt-pytest.xml`。

## 保留的失败

- `fc1ba78a-8210-4742-b760-1a2a0d5a9095`：归档编码失败，selector 未使用实例的
  NaN 槽位无法写入严格 JSON。现仅在结果序列化时将不可用槽位写为 null，
  不改变运行门控的有限值检查。原 failure、manifest、日志和 cleanup 均保留。
- `6039c5ef-56e6-47ba-98f1-aefcc8490746`：实际融合存在，但 flags 原 0.5 s
  时效门限反复重置稳定窗口，ready_count=0、passed=false。按上游 1 Hz 周期修复后
  重跑最终验收；原失败没有改写为通过。
- 首次独立编译遇到父环境 PYTHONPATH 将 Python 3.12 扩展带入 PX4 Python 3.11。
  构建子进程移除 PYTHONPATH/VIRTUAL_ENV 并固定 PX4 venv PATH 后完成编译。
  `build.log.gz` 保存成功构建日志，首次编译失败日志未单独归档。

`process-audit.json` 确认上述四个试验的根进程与进程组均已退出；仅清理受管试验。
供应商检查确认既有 managed patches 已应用，未将脏子模块混入本批提交。
归档采用结果/日志白名单，不含运行授权、QGC 配置或 rootfs。
`implementation.sha256` 绑定最终源码，`SHA256SUMS` 校验归档。

## 复现与下一步

```bash
./scripts/build_px4_sim.sh
./scripts/build_px4_flight.sh
./scripts/build_px4_vio.sh --jobs 4
./scripts/build_px4_vio.sh --check
./scripts/sim.sh px4-vision-audit --duration 35
```

代码分批提交：`0a4509a` 独立构建，`6616c67` 发布周期门限，`a529ef9` 实际融合审计。

下一步是固定双目/IMU仿真模型和标定/同步配置，接入本机真实 VIO 算法，再验证
运动轴向、重置、漂移与真实定位失效。当前固定合成输入禁止用于起飞/悬停任务；
没有完成室内飞行、VIO 丢失后的降落或 OAK-D Pro W 实机验收。
