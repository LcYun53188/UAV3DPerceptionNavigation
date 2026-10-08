# 非 identity 坐标、会话绑定与 EGO 影子规划（2026-10-08）

本批打通显式坐标适配 → 实际 EGO → 独立复检 → 带 context 轨迹输出的影子规划链路，
并给 PX4 网关增加带定位 session/reset 的里程计。两者分别通过真实 DDS 回归和完整
PX4 SITL 飞行验证，尚未合并为 PX4 感知避障执行任务。

## 实现

- 新增 LocalizedOdometry、LocalizationAlignment、PlanningContext、ContextTrajectory。
  session 与六个 reset 计数随位姿传递，显式对齐同时绑定 map_id/epoch 和定位身份。
- SE(3) 变换位置、姿态、完整 pose 协方差；child FLU twist/协方差不改变。验证非法
  帧、四元数、非有限值及协方差，保留采样 header，未通过重命名 frame 伪造坐标转换。
- Context 摘要排除正常地图版本增长，包含地图/定位/对齐世代和变换。输入超时、多个
  发布者、epoch 切换和身份冲突撤销目标；session/reset 不匹配使当前对齐世代失效，
  字段恢复不能自动重用旧对齐，必须由明确的新世代重新确认。
- 目标时间戳不可重用；仅静止起点；曲线需匹配当前目标、context 和版本下限。对最新
  地图独立检查扫掠碰撞、未知空间、动态界限、端点和终端静止，每目标仅绑定一次。
- EGO 增加 opt-in coordinate_frame=map；默认 odom identity 模式保留。新 Python bridge
  只发布影子规划接口，不发布 cmd_vel、FMU 或控制权；context 摘要不是执行授权。

## 真实 EGO / DDS 回归

运行器在独立 domain 91 使用系统时间、合成 ESDF 和显式对齐，实际启动新 bridge 与
重建的 C++ EGO。变换平移 (4,-3,1) m、yaw=90°，原 odom 起点 (1,0,1) 实际变为
map (4,-2,2)，目标 (6,-2,2)。网格中的实心墙阻断直线，不能用纯平移/改 frame 绕过。

最终接收一条绑定轨迹：16 个控制点、规划时长 11.07219 s、绕行 y 偏移最大 1.54612 m，
起终点与目标匹配，独立 Python 校验整个曲线的碰撞/动态门限通过。EGO 日志记录
优化器线搜索失败后采用既有 SAFE_SEED_FALLBACK；这是 EGO 原 A* 路线的保守曲线
回退并通过复检，不能声称本轮 rebound 优化成功，也未执行该曲线的物理运动。

DDS 回归实际验证：重复原始结果拒绝、第二个对齐发布者导致上下文失效、同世代变换
修改锁存、增加世代后旧目标不恢复、地图 epoch 改变须重新绑定、对齐超时拒绝旧曲线。
旧默认 identity 模式另启动同一 C++ 二进制，实际产出 odom 曲线并独立复检通过。
影子域没有 cmd_vel/FMU topic；三个自有进程均退出 0。result.json、contexts.json、
bound-trajectory.json 及三个日志记录最终回归，源码/二进制 hash 已与归档时文件核对。

## 启动失败与修复

excluded-discovery-startup 保存一次未计入验收的超时及当轮日志，不复制该输出目录中
上一轮遗留的轨迹/上下文文件。该轮 bridge 已报告 READY，但 EGO 没有规划活动记录。
代码审计发现输入就绪不等于 EGO DDS 发现完成，一次性 goal 可能在订阅匹配前丢失；
现场未记录首次派发时的匹配数，故这条归因是结合代码的推断。

现增加 bridge 的唯一目标订阅者检查，运行器在派发前确认 EGO 订阅/发布发现，并记录
planner_discovery_confirmed。运行器还要求显式输出目录为空，默认每次创建新目录，
避免新失败混入旧成功文件。最终上述完整回归已重新通过。

## PX4 真实飞行与里程计输出

独立 W0 full 用例 0aa5c806-1450-4661-ad8e-045fe741908b：实际 PX4 v1.16.2/x500_7，
完成起飞、两航点、悬停、返航和原生降落，根 SUCCEEDED/LANDED_AND_DISARMED，
cleanup=true、mock=false，七个步骤各授权/完成一次。最大真值位移 4.18809 m，
起飞/两航点/返航误差分别为 0.08846、0.07400、0.06489、0.08158 m。
两个悬停观察段 32.004 s、5.018 s，最大漂移 0.11289、0.10729 m。

通过实际 DDS 接收新 LocalizedOdometry 10110 条，唯一定位 session 为
3c182922-abd2-4dcd-ad47-3302606088f5，均保持 odom/base_link、外层/内层 header 一致。
计数取实际输入，不假设为零；封装计数顺序由针对性测试验证。这个用例没有运行影子
EGO，也没有连接 nvblox；不能据此声称 EGO 已经控制 PX4。

PX4 会话的源码/二进制 hash 已核对，受管进程均退出。原算法 domain 68 的 Gazebo、
RViz、地图保持运行，状态复核 ready=true、HOLD、map.valid=true。独立 QGC 已恢复，
PID 464425 持有 USB ttyACM0；本批未启动实机控制桥或发送实机解锁/飞行控制指令。

## 测试与交付边界

导航侧 259 passed（11.49 s，含 36 项新上下文/变换用例）；飞行/桥/审计侧 164 passed
（0.59 s）。新增测试覆盖完整协方差交叉项、roll/yaw 变换、各输入新鲜度、身份与世代
恢复、重放、旧版本、端点、动态/未知空间、时钟锁存及 PX4 里程计封装。
三包算法侧构建通过（最终 39.1 s），四包隔离 PX4 构建通过（29.0 s）。
managed vendor/ego、Python AST 和 Git whitespace 检查通过；vendor 内容未提交。

证据仅复制白名单结果/日志/轨迹/诊断，不含授权私有 JSON、QGC 配置缓存或 rootfs。
SHA256SUMS.json 校验归档。原算法内存地图没有重置，磁盘地图包未改动。

仍需实际 PX4 深度传感器/nvblox、已验证的地图对齐来源、持久化地图 schema 2、
父任务/ControlSession 与 map 样条到 PX4 local 执行接线，以及制动/净空物理验收。
本批是 S3 和 S5 前置接口的推进，不将 S3/S5 或感知导航整体验收标记完成。

实现提交：c842b6f（接口）、e0c35a6（坐标与规划）、abd9657（PX4 输出）、2205b03（DDS 回归）。
