# 同机起飞与有限深度观测任务（2026-10-10）

新增 `./scripts/sim.sh px4-observe`。实际 PX4／Gazebo／BT 已连续两轮完成：
已知区域起飞到相对 EKF home 的 2 m → 定点 yaw 观测 → 观测不足时原生降落 →
确认 landed／disarmed 后返回 `ABORTED:OBSERVATION_INSUFFICIENT`、cleanup_confirmed=true。
**通过的是预期观测不足的安全结束验收；不是导航任务成功或 EGO 曲线飞行成功。**

## 实现

新增冻结 `depth_reference` 飞行 profile，使用同机理想 RGBD、固定下偏 5°。
生成模型／场景／光学 TF／profile 的哈希固定在 profile 中；owned cache 下的 receipt
绑定 partition、region hash、coordinator UUID 及完整资产清单。
只重新计算 receipt 或修改相机资产也不能绕过冻结资产哈希。
已知区域中心限制到 x/y +/-1 m（bounds +/-3 m，机体＋跟踪＋制动余量 2 m），
与参考场景 x=4、y=+/-4 m 墙面保持保守间距。
保持官方 GNSS／惯性 EKF，不接 VIO，不使用 Gazebo 真值作为控制定位。

FlightServer 是唯一命令／setpoint／动态 body TF 和 localized odometry 来源。
本入口不启动未解锁 mapping_tf observer；仅单独发布冻结的 camera static TF。
FlightServer 与 map binder 使用监督器提供的同一 UUID，map ← odom 显式为 identity，
不混用 W0 的旧非 identity 变换。binder 等待全部六个 reset 计数稳定 5 秒后绑定。
规划图在根任务前就绪，但未观测空间仍不可导航。

BT 新增 OBSERVE 类型。该步骤只允许冻结深度 profile＋EGO，并且须紧接 TAKEOFF。
位置保持，yaw 指令最大变化率 0.35 rad/s，完成 2π 扫描后仍须通过实际地图的
0.8 m 机体／跟踪半径＋1.2 m 水平制动包络检查并确认稳定，才允许下一步骤。
本 recipe 的观测期限为 25 s。期限结束未通过时，先退役曲线／child，再请求
PX4 native Land；等待新鲜落地和锁定状态稳定 2 s，才确认清理并返回 ABORTED。
底层时钟／定位／控制故障仍使用原有故障路径，不被观测期限替代。

## 实测

| 轮次 | 原始运行 ID | 结果 |
| --- | --- | --- |
| 启动原型 | b6bd82ad-1268-47d5-bfe2-c203302ec9e5 | 失败：继承的 ROS overlay 顺序导致找不到 planning_sources；未开始任务 |
| 首次飞行 | 476fcd8b-8a03-4f84-a266-7911445b55e1 | 起飞／观测／期限降落／锁定通过，任务以预期 ABORTED 结束 |
| 实测航向复验 | 5a505efb-01bf-478f-a2e1-c38436b55a79 | 同上；实际累计航向变化 6.281596843 rad |

最终轮独立真值最大位移 2.029149396 m，OBSERVE 阶段真值样本 1252 条，
相对该阶段首点最大漂移 0.130301447 m（首次轮 0.197817767 m）。
相对 home 的 2 m 目标是 EKF 坐标，不能把命令高度直接视为每时刻的真值高度。
末次地图版本 103，188,551 已观测体素；定位会话为本轮 UUID，
六个 reset 计数 `[4,3,3,2,1,13]`，alignment generation=1。
地图增长和实际 yaw 扫描都未满足闭合机体／制动包络，所有观测采样 clear=false，
原因 `UNOBSERVED_OR_OCCUPIED_START_VOLUME`。该原因不能区分未知与占用的具体分布，
不能直接归因于某一个摄像头盲区。

两轮均 root Action status=6，result code=ABORTED，reason=OBSERVATION_INSUFFICIENT，
cleanup_confirmed=true，最终 landed=true、arming_state=1。
BT 根仅派发一次，步骤接受 [0,1]，完成 [0]；OBSERVE 触发原生降落，
最终 LAND 叶未获得新控制授权，BT 按任务失败退出 1，这是预期语义。
未出现 NAVIGATE／PLAN_REQUEST，未以“观测完成”冒充轨迹导航成功。
`passed=true` 表示预期失败场景验证通过，`observation_task_succeeded=false` 保留任务失败。
旧通用 flight helper 的 scope/profile 文本在两轮原始证据中较宽泛，
最终 helper 已改为真实 profile 和有限观测 scope；原始文件与哈希不重写。

## 验证与复现

`uav_nav_sim`、`uav_mission`、`uav_bt` 隔离构建通过，**183 项相关测试通过**。
新增验收覆盖：资产篡改／重算 receipt／session mismatch／VIO 混用拒绝，
观测类型与期限校验、完整扫描后才能成功、期限结束先退役再降落，以及
启动期 reset 稳定等待。底层既有轨迹与控制授权测试继续通过。

```bash
./scripts/build_px4_flight.sh --packages-select uav_nav_sim uav_mission uav_bt
./scripts/sim.sh px4-observe
python3 docs/validation/simulation/2026-10-10-depth-observation-flight/check_archive.py
```

启动失败已修复：受管 mapping 子进程清除继承的 prefix／PYTHONPATH，
按基础工作区 → 隔离 px4_msgs → mission overlay 的顺序加载，避免父 wrapper 已加载
mission overlay 后再 source 基础工作区使 ROS 选中旧安装包。
只影响本次子进程环境，不修改系统配置。所有受管进程组退出已检查。
日志 gzip 保留原始字节，解压哈希见 RAW_LOG_SHA256SUMS；其他文件见 SHA256SUMS。

## 尚待完成

该任务是观测阶段闭环及失败落地验收，EGO 轨迹导航／避障尚未在此同机源上完成。
下一步记录缺失／占用体素分布，并在同一冻结已知区域内增加有期限的多位置／多高度
观测路径，再接完整导航、悬停、返航、降落。相机继续固定下偏 5°，不预填未知空间，
不降低机体、制动、定位或时间门限；VIO 保持独立验收。
