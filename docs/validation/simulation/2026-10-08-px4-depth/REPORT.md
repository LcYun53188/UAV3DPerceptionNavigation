# PX4 参考深度相机链路验收（2026-10-08）

目标硬件经用户确认是 **OAK-D Pro W**。本批使用固定 PX4 v1.16.2 中的
`x500_depth` / `OakD-Lite` 参考模型，完成未解锁的 Gazebo → ROS 深度/标定数据
审计；没有把参考模型改名冒充 Pro W，没有宣称实机视场、内参或双目/IMU 已验证。

## 结果

| 检查 | 结果与证据 |
| --- | --- |
| 深度链路单元测试 | PASS，23 项；`depth-pytest.xml` |
| 飞控/里程计/任务审计回归 | PASS，164 项；`flight-regression.xml` |
| 深度相机独立 SITL + Gazebo/QGC UI | PASS，run `bff7b70b-de74-4aea-aa84-baf8ed55cee8` |
| 原 x500 默认未解锁冒烟 | PASS，run `a6f84eee-bb03-4b3e-bc0a-a422bf723732`；`baseline-x500/` |
| 参考相机与飞行参数互斥 | PASS，4 组命令均在启动前 exit 2；`cli-validation.json` |
| 清理与保留现场 | 本次进程组无存活成员，独立 QGC 已恢复；`process-audit.json`。原 domain 68 地图有效、HOLD、ready=true；`algorithm-status.txt` |

最终相机运行 `./scripts/sim.sh px4-depth --ui --duration 45`：

- 707 帧深度图、1,143 条 CameraInfo 均通过审计，没有时间戳回退或审计错误。
- 深度尺寸 640×480、32FC1，末帧有限深度 149,626 像素，占 48.71%，范围
  0.44745–16.48864 m；背景无效深度不作为自由空间使用。本批只统计，不发布地图。
- 末帧深度/标定 frame 均为 `x500_depth_7/camera_link/StereoOV7251`，stamp
  40.988 s；相对末次仿真时钟源年龄 0.012 s。
- K/P 中 fx=fy=432.496042、cx=320、cy=240；R 为 identity，D 为零。
  这些是上游参考模型的标定，**不是 OAK-D Pro W 标定**。
- 两个 ROS 输入各只有一个发布者；DDS、唯一 clock、QGC 连接、有效局部定位均通过。
  整个观察窗口只有 DISARMED 状态，末次着地为真。没有启动 FlightServer 或 BT。
- `manifest.json` 固定 x500_depth、x500、x500_base、OakD-Lite、world/server config
  与审计源码 hash；`implementation.sha256` 对应本批脚本源码。
- X11 记录确认独立相机 Gazebo/QGC 窗口与原算法 Gazebo/RViz 共存；窗口框与内部
  client 不重复计为两套 UI。

配置更新率为 30 Hz；实际 ROS 接收帧数不用于宣称达到完整 30 Hz 性能门槛。
当前验收只要求连续有效输入及末次新鲜度，没有完成丢帧率、同步分布、运动畸变或负载压力测试。
Gazebo EGL 警告已保留在压缩日志中，实测存在有效渲染输出。

## 缺失标定的失败证据

首次 run `b8535440-91a7-4a43-9487-1b38d31f2a85` 使用错误的 Gazebo
`/depth_camera/camera_info` 路径：收到 676 帧有效深度、零条 CameraInfo。
DDS/QGC 通过，但整体明确 FAIL。`excluded-missing-camera-info/` 保存该次
观测与清理记录。随后用第二次独立会话列举实际话题，确认深度标定发布在
`/camera_info`，修正桥接后才通过。话题列表中错误路径也存在，是桥接订阅建立的
端点，不能据此断言该路径有真实标定发布者。没有用理论内参补发 CameraInfo 伪造通过。

## 适用边界与下一步

本批未发送 USB 实机飞行控制指令，没有新增飞行参数豁免。
未验证 Pro W 实机标定、安装外参、光学坐标轴、左右目/IMU、VIO、nvblox 融合或
EGO 驱动 PX4。原 W0 飞行仍使用原 x500；本批只重新运行其未解锁冒烟，未重飞完整任务。

下一步明确 Pro W 的仿真参考参数与光学外参，在隔离 domain 内接入 nvblox 地图会话，
保留地图/定位 session、reset 和 alignment 门控，再推进地图样条到飞控的授权执行。
