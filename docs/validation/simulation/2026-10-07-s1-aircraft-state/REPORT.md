# S1 首批：只读 AircraftState 与故障失效

日期：2026-10-07。本机 x86_64/Jazzy/PX4 v1.16.2，继续延期 Jetson。
实现提交：`b2e8dd4`；本目录证据与 `--aircraft-state` 运行器在后续验收提交中归档。
实现文件的 SHA-256 见 [implementation.json](implementation.json)。

## 交付与验证

新增 AircraftState / StateDimension 接口、`uav_mission` Python 包及只读 PX4
适配器。状态维度独立失效，携带源时间/ROS 源年龄/单调接收年龄、实例 UUID
与递增序号。重复包不能续新鲜度；时钟故障、源时间倒退、定位 reset 锁存。
稳态时钟定时器保证 `/clock` 停止后继续报告故障。详细契约和复现命令见
[包说明](../../../../src/uav_mission/README.md)。

- 独立构建 `uav_nav_interfaces`、`uav_mission`：两包成功，使用 `.deps/mission-install`，未覆盖旧 workspace 的 PX4 消息。
- 聚合器 36 项故障/质量/身份回归与现有仿真工具 46 项回归：**82 passed**。
- Python 编译与 Git 空白检查通过。
- 最终未解锁 x500 会话：`6de59b83-879c-4aaf-a3fb-ca8393dcfa7a`，30 s 基础观察后注入故障，**PASS**。

基础观察收到 VehicleStatus 40、land_detected 21、local_position 2045、odometry
2045、clock 7389、AircraftState 657 条。计数截取于故障注入前；故障阶段仍继续
接收状态消息。原 DDS/clock/QGC 判据也通过，无任何解锁或运动命令。

| 阶段 | 实测结果 |
| --- | --- |
| 新鲜检测器窗口 | DISARMED / ON_GROUND 均 valid；link CONNECTED；navigation NOT_READY 且 invalid |
| 停止 PX4、时钟继续 | arming/ground/mode UNKNOWN，reason STALE_SAMPLE；link LOST |
| 再停止 clock bridge | 状态继续发布；arming/ground/mode UNKNOWN，reason ROS_TIME_STALLED |

见 [observation.json](observation.json)、[会话参数](manifest.json)、
[清理结果](cleanup.json) 与 [节点日志](aircraft_state.log)。仅结束本次创建的进程组，
QGC 仍需要 SIGKILL，其退出码 -9 不作为正常退出；进程检查无本次残留。
完整原始会话在 `.cache/simulation/sitl/6de59b83-879c-4aaf-a3fb-ca8393dcfa7a`。
较早同样通过的会话 `02d8bb38-9d8a-415f-abc3-f2d501f6d2d6` 亦保留在缓存。

## 实测限制与下一步

PX4 启动期间定位 reset counter 变化触发 `LOCAL_POSITION_RESET` 锁存，本次
LOCAL_POSITION **未计通过**，也未自动恢复。需要在后续 FlightSession/定位会话
协议中实现明确的故障确认与重新授权；当前只能在定位稳定后显式重启观察器。
定位质量的纯函数用例通过不等于定位/TF 动态验收。

默认 land_detected 约 1 Hz，0.5 s 新鲜度会出现无效窗口；运行器等待新鲜窗口，
未放宽阈值。link 仅代表 VehicleStatus 可观察性，Agent 健康、battery、控制所有者、
地图/TF/导航就绪提供者仍缺失；UNKNOWN 不授权。S1 的 ExecuteMission/
NavigateToPose3D/TaskStatus/控制会话/Pause/Resume 和 FlightSession mock 尚未交付，
因此 **S1 未整体完成**。S3 坐标转换与飞行时间门槛、S4 起降控制尚待验证。

S0 cuVSLAM 固定数据样例构建补齐了 26 个依赖包，在 isaac_ros_image_proc 的
`pad_node.cpp` 因缺少 CV-CUDA 头文件 `nvcv/Tensor.hpp` 失败，visual_slam 尚未
构建完成；见 [失败尾日志](vio-build-tail.log)。原始日志为
`.cache/simulation/vio-build.log`，归档尾日志仅清理行尾空白。仍需补齐并冻结
CV-CUDA/运行时依赖，再运行固定双目/IMU 与 nvblox 数据样例；未计 S0 感知通过。

接下来完成 S1 任务身份、Action/暂停协议与 FlightSession mock，验证旧事件隔离、
取消/保持交接和固定最终期限，再接入 S2 MissionServer/BT。
