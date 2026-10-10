# 同机深度地图与 EGO 规划源绑定（2026-10-10）

新增受管入口 `./scripts/sim.sh px4-depth-plan --duration 40`，实际启动同一架
x500 的固定下偏 5° RGBD → PX4 GNSS／惯性 EKF TF → nvblox ESDF →
规划源绑定 → PlanningContext → C++ EGO shadow 图。
同机源绑定连续两轮通过；EGO 曲线生成／执行器接纳另以合成 ESDF 回归通过。
本轮没有下发同机导航目标，也没有执行飞行任务。

## 实现与身份约束

`planning_sources` 要求显式 localization_session、alignment_id 和 map ← odom 变换。
此次未解锁参考地图按定义使用该次 EKF odom 坐标，变换为 identity，
与 W0 FlightServer 当前冻结的非 identity 变换不是同一个飞行配置，不能直接混用。
不从 Gazebo 真值、位置相近或 topic 名称推断对齐。

- map 消息原样转发，保留原始 header/source_stamp、map_id、epoch、version、observed；
  alignment 心跳不刷新地图或里程计的源时间。
- localized odometry 发布 PX4 原始采样时间与会话 UUID，同时携带
  xy、z、vxy、vz、heading、VehicleOdometry 的六个重置计数。
- 初始化只在全部重置计数稳定 5 秒、局部位置有效且时间新鲜时开始 TF／里程计输出；
  开始后任一重置、解锁或无效位姿会停止输出；状态和局部位置过期也不转发。
- binder 校验地图／里程计新鲜度、原始源时间、frame、header、变换和协方差。
  已绑定后地图 session／定位 session／reset 变化、版本或源时间回退、
  发布者丢失／重复／更换以及时钟故障会退役绑定，不能自动重新授权旧地图。
  重复样本不刷新接收时间；失效地图不能清除版本最高水位。
- 首次有效源到达前不激活规划时钟 watchdog；源激活后保留原 0.5 s 时钟停滞保护。
  未绑定时 alignment generation=0，首次有效绑定才为 1，避免同世代 identity 改变。
- PlanningContext 仅在新地图版本／时间戳时转发大网格；定位持续更新，
  减少把同一 ESDF 随每次位姿更新重复发送的负载。

源 binder 仅发布规划地图与 alignment，observer 仅发布 TF／里程计；
不发布任何 FMU 输入，不取代唯一 FlightServer 的命令与 setpoint 写入权。
未来飞行模式须直接使用 FlightServer 自己发布的 localized odometry／会话，
不能并行启动本轮未解锁 TF observer。

## 实际运行证据

| 运行 | 原始 ID | 结果 |
| --- | --- | --- |
| 首轮原型（45 s） | 7da54428-bfa4-4932-ab5a-a24a2040f468 | 失败：源未初始化时启动时钟 watchdog，后续 context 锁存 CLOCK_FAULT；真实深度地图单独通过 |
| 初始化修复后（45 s） | 75dbdfb1-1f61-4fd3-a299-9588d8ffed9b | DDS／时钟／QGC／深度／地图／规划上下文通过 |
| 最终代码（40 s） | dd369c9b-aeb9-49db-a1bd-0db99a55e75b | 同机完整 shadow 图通过，所有受管进程组已退出 |

最终轮规划审计记录 4,530 个 valid context，末次 context 地图版本 69；
最后地图快照版本 71，14,948 已观测体素，其中 10,722 距离为正，
418,241 总体素，地图源年龄 0.704 s。
这些末次文件来自不同回调时刻，context 版本不应被改写成最终地图版本。
会话为本轮 UUID，六个重置计数为 `[4,3,3,2,1,13]`，alignment generation=1。
地图、localized odometry、alignment、context、bound trajectory 和 EGO raw trajectory
发布者各 1，EGO goal reader=1。

**地面飞行包络检查为 false**：机体＋跟踪半径 0.8 m 与水平制动余量 1.2 m
尚不满足已观测无碰撞条件，因此审计没有提交目标。
`context.valid` 表示源契约可用，不表示机体周边全观测或任务可飞。
仿真相机是理想 RGBD 参考，不能作为 OAK-D Pro W 实机标定或 5° VIO 悬停验收。

另运行真实 EGO＋PlanningContext 回归（域 91、合成 ESDF、不含物理跟踪）：
16 个控制点、11.072 s 曲线、绕障偏移 1.546 m；唯一 PX4 执行器核心接纳和采样通过。
多源、重放、对齐世代变化、地图 epoch 变化和 alignment 过期拒绝仍通过，
该轮无 FMU 控制 topic，三个子进程退出码均 0。
该结果不能代替同机深度地图上的轨迹飞行。

## 复验

`uav_nav_sim` 与 `uav_mission` 隔离构建通过。**156 项测试通过**，覆盖源绑定、
六个重置计数、发布者替换／丢失／重复、初始化与运行期时钟故障、控制身份、
曲线接纳、地图更新和历史深度 TF。

```bash
./scripts/build_px4_flight.sh --packages-select uav_nav_sim uav_mission
./scripts/sim.sh px4-depth-plan --duration 40
python3 docs/validation/simulation/2026-10-10-depth-planning-sources/check_archive.py
```

原始日志以 `.log.gz` 保存，`RAW_LOG_SHA256SUMS` 验证解压后的原始字节，
`SHA256SUMS` 验证归档文件。各原始 manifest 保留实际运行源码及资产哈希，
不以最终实现覆盖原型哈希。日志中的基础安装环境仍提示缺失
isaac_ros_image_proc local_setup，但本轮相关节点、构建和验证实际通过；
该提示不计为相机标定或其他包可用性的证据。

## 下一步

新增严格绑定生成场景的官方 GNSS／惯性飞行 profile，复用唯一 FlightServer；
设计显式已知区域起飞与有期限的观测阶段，取得机体／制动包络足够覆盖后才发 EGO 目标。
然后验证完整曲线导航、悬停、返航、原生降落和地图／定位故障降落。
不能复用未解锁 observer 进行飞行，不能填充未知为空闲，也不能放宽安全包络。
