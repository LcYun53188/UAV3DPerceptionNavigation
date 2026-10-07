# 本机 PX4 SITL 基础环境

当前实现：PX4 v1.16.2、Agent v2.4.3、对应 px4_msgs 的独立构建，以及未解锁 x500 的 DDS/clock/QGC 冒烟验证。全部运行于本机，Jetson 延期。此入口尚未包含任务、Offboard 控制、坐标迁移、W0 安全区域或 VIO。

```bash
# 首次下载锁定源码、SITL 所需子模块并构建；不构建 CUDA 导航栈
./scripts/build_px4_sim.sh --jobs 4
# 仅准备源码并核对消息定义
./scripts/build_px4_sim.sh --prepare-only
# 校验已有源码与 Agent 已解析依赖是否发生漂移
./scripts/with_venv.sh python scripts/prepare_px4_sim.py --check --check-external
# 编译后的独立 ROS/Agent 运行环境
./scripts/with_px4_sim.sh python scripts/run_px4_sitl_smoke.py --duration 30
```

源码/构建/安装均在 `.deps/`。PX4 构建用隔离的 Python 3.11 环境及 `requirements/px4-sim.txt`；ROS Jazzy 消息生成和运行用工作区已有 Python 3.12 环境。构建时清除继承的 PYTHONPATH，避免 Python 扩展 ABI 混用。Agent 安装到本工作区，不需要 sudo 或全局 ldconfig。

现有 `src/px4_msgs` 和 `install_uav` 保留；SITL 用 `.deps/px4_msgs` 与 `.deps/px4-msgs-install`，必须使用 `with_px4_sim.sh` 选择该 overlay。该包的 v1.16.2 tag 对应 commit `392e831c1f659429ca83902e66820d7094591410`。`check_px4_interfaces.py` 按固件 dds_topics.yaml 检查导出消息及其嵌套类型的声明顺序/字段/常量；注释和空白不参与比较。

构建脚本拒绝不符合版本锁或有 tracked 修改的根 checkout。PX4 仅初始化当前 SITL 所需子模块，NuttX、Gazebo Classic 等不初始化。Agent superbuild 的底层 Fast-CDR/Fast-DDS/spdlog 会解析上游分支；已记录本次实际 commit 和构建生成的 tracked diff hash，并在构建末尾及运行前核对，漂移会拒绝使用。全新构建若解析到更新的依赖，将失败并要求显式重验版本锁，不自动认可新依赖；这仍有首次下载时的上游分支可用性依赖。

冒烟运行器使用 instance 7、system_id 8、ROS domain 78、XRCE UDP 8898、GCS UDP 14550、PX4 GCS 本地端口 18577，Gazebo partition 每次为唯一 UUID。DDS namespace 是 `/px4_7`，VehicleStatus topic 是 `/px4_7/fmu/out/vehicle_status_v1`。运行前检查端口与 PX4 实例锁，并通过工作区互斥锁防止本工具重复启动。已有占用时退出，不终止其他会话。

一个 supervisor 分别管理 Gazebo server、standalone PX4、Agent、clock bridge 和 QGC，每个组件属于本次新建进程组。Gazebo 只由 supervisor 启动，PX4 使用 `PX4_GZ_STANDALONE=1`。结束或失败只清理这些组。QGC 使用独立 XDG 配置/缓存、关闭串口自动连接，offscreen 模式连接模拟飞机；不会改变用户现有 QGC 配置，也没有人工 GUI 验收结论。

PX4 采用 `UXRCE_DDS_SYNCT=0`；ROS `/clock` 从同一 Gazebo 单向桥接。检查状态/里程计/位置/着陆样本、接收年龄、源时间与 clock 差、唯一发布者、未解锁与已着陆，并从 QGC 日志核对 system_id 8 的识别事件。所有等待期限用单调时钟。该短测试没有实现 CLOCK_FAULT latch、暂停恢复或低 RTF 正式验收，也没有验证 ENU/NED 转换。

当前机器缺少 GStreamer development 依赖，默认的摄像头串流插件未构建。项目的 `server_control.config` 从锁定 PX4 的 Gazebo server 配置派生，仅删除 GstCameraSystem 加载项；其他物理与传感器系统保留。它仅用于 x500 基础状态验证，摄像头/深度/VIO 能力不计通过；后续 S5 应使用完整且单独验证的感知配置。

日志、manifest（命令/PID/版本/场景配置 hash）、rootfs 参数、ULog、观测和清理结果保存到 `.cache/simulation/sitl/<run_id>/`。回归包括源码 pin 拒绝覆盖、消息字段重排与嵌套类型缺失检测。基础链路报告见 [本机 SITL 验证](../../docs/validation/simulation/2026-10-07-px4-sitl-build/REPORT.md)。
