# PX4 v1.16.2 本机构建与基础链路验证

日期：2026-10-07。结论：独立构建、接口对应与未解锁 x500 的 DDS/clock/QGC 冒烟通过；S0 本机 VIO/nvblox 固定样例与 S1 状态/协议尚未完成，S3/S4 飞行门槛未通过。

## 已交付

- `scripts/build_px4_sim.sh`：准备精确源码和 SITL 子模块，独立编译 PX4/Agent/消息包，不触发 CUDA 导航全量构建或全局安装。该脚本已完整执行成功，退出码 0。
- `scripts/prepare_px4_sim.py`：核对源码身份，拒绝覆盖已有 checkout 的不同 commit/tracked 修改；核对 Agent 已解析依赖，漂移时拒绝使用。
- `scripts/check_px4_interfaces.py`：检查固件 DDS 导出的全部消息与嵌套类型。
- `scripts/with_px4_sim.sh`：隔离的 ROS 消息 overlay 与本地 Agent 库环境。
- `scripts/run_px4_sitl_smoke.py`：单 supervisor 管理独立 Gazebo/PX4/Agent/clock/QGC 进程组，采集基础状态，结束后清理本次进程；没有运动命令、解锁或起飞操作。

## 版本与构建

| 组件 | 版本/commit | 本机结果 |
| --- | --- | --- |
| PX4 SITL | v1.16.2 / `54f0455ffcd755534539a7cf33a09a20bf71d29d` | px4_sitl_default 构建成功 |
| Micro XRCE-DDS Agent | v2.4.3 / `73622810d984349b80bbac0ef55fc0b694d62222` | 安装到 `.deps/microxrce-install` |
| px4_msgs | v1.16.2 / `392e831c1f659429ca83902e66820d7094591410` | 独立 ROS Jazzy overlay 构建成功 |
| QGC | 本机 AppImage v5.1.5 | offscreen 模式识别到模拟飞机 |
| Gazebo | Harmonic / Sim 8.11.0 | x500/default 动力学世界启动成功 |

Agent v2.4.3 的选型依据为 [PX4 1.16 官方 ROS 2 指南](https://docs.px4.io/v1.16/en/ros2/user_guide#setup-the-agent)。该指南使用 UDP Agent；本次实际验证与所编译 Client 成功互通。Agent 底层依赖 commit、PX4 已初始化/未初始化子模块、Python 环境和二进制 hash 见 [build.json](build.json) 及版本锁。底层依赖首次下载仍使用上游 superbuild 分支，构建后必须通过已锁定 commit/diff 检查；后续分支漂移不静默接受。

现有 `src/px4_msgs` 保持原样。本批 SITL 消息包位于 `.deps/px4_msgs`，运行使用 `.deps/px4-msgs-install`，避免混用原导航栈接口。接口比较结果为 **46 种消息、51 个 DDS topic 声明全部一致**，包含嵌套类型；见 [interfaces.json](interfaces.json)。比较声明顺序、字段和常量，不把注释差异判为不兼容。

## 实际会话结果

最终 run_id：`4e18eac8-4967-40a8-b65a-631f77ebc002`。30 s 单调时间采样窗口含启动阶段；实际 clock 从 0.008 s 推进到 27.748 s，不把启动时长当作稳态性能统计。

- instance 7 / system_id 8，ROS domain 78，XRCE UDP 8898，GCS UDP 14550，PX4 GCS 本地端口 18577；Gazebo partition 为本次 UUID。完整命令、PID、配置/hash 见 [manifest.json](manifest.json)。
- QGC 日志记录 `Adding new vehicle ... "UDP Link (AutoConnect)" 8 1 12 2`，并设定 active vehicle；见 [qgc.log](qgc.log)。这证明 QGC 程序接收到/识别了模拟飞机，不表示人工检查了 GUI。
- DDS 样本：VehicleOdometry 2042、VehicleLocalPosition 2042、VehicleStatus 40、VehicleLandDetected 21；clock 6936。全部采样到的 arming_state 为 1（DISARMED），最终 landed=true，xy_valid/z_valid=true。
- 采样末尾里程计源时间与 clock 差约 0.008 s；状态接收年龄均小于该冒烟脚本的 1 s 窗口。PX4 启动日志明确将 `UXRCE_DDS_SYNCT` 从 1 设为 0，见 [px4.log](px4.log)。这不代替完整的源年龄/时钟跳变验收。
- `/clock` 有且仅有一个发布者，所发现的 `/px4_7/fmu/out/*` 均为一个发布者；VehicleStatus 使用带版本后缀的 `/px4_7/fmu/out/vehicle_status_v1`。详见 [observation.json](observation.json)。
- 本次进程组全部清理，见 [cleanup.json](cleanup.json)。QGC 未响应 SIGINT，期限后由本工具终止其所属组；其他会话不受影响。rootfs 参数、ULog 和原始完整记录保留在 `.cache/simulation/sitl/<run_id>/`，ULog 不加入 Git。

独立针对性回归 **250 passed**，覆盖原算法/仿真控制、验收工具，以及新增的接口字段重排/嵌套类型缺失/错误 commit 不覆盖本地工作的验证；详情见 [build.json](build.json)。离线环境与受管 vendor patch 检查通过，见 [environment.json](environment.json)。

## 排错记录与当前边界

第一次 PX4 构建因 Python 3.11 环境继承 ROS/Python 3.12 的 PYTHONPATH 而无法加载 rpds 扩展；已在构建入口清除继承路径，重新构建成功。失败日志保留于 `.cache/simulation/px4-build-initial-failure.log`，不修改全局 Python。

第一次冒烟 run_id `d6884c08-21e4-497f-bb28-7bebcd859740` 的观察器遗漏 instance 7 自动生成的 `/px4_7` namespace，DDS 检查失败；日志和失败观测保留在对应缓存目录。订阅路径修正后，`f17ede57-09d1-4ca8-ba9e-4b521c5f753d` 的 DDS/clock 通过，`12f29daf-cbac-4c81-a39e-20909f554e79` 加入 QGC/源时间判据后通过，最终版本再次运行通过。没有把失败样本删除或重写成成功。

机器没有 GStreamer 开发依赖，故 GstCameraSystem 未构建。项目 `server_control.config` 从锁定 PX4 的 server.config 派生并仅移除该串流插件，其他系统保留；最终 Gazebo 日志无该加载错误。该配置只支持当前基础控制环境验证，摄像头/深度/VIO 不计通过。原文件来自 PX4-Autopilot 的 `src/modules/simulation/gz_bridge/server.config`，上游许可证见该仓库 LICENSE。

本批没有修改已知有问题的 PX4→ROS 里程计转换，没有非 identity 对齐、AircraftState 聚合、CLOCK_FAULT 锁存、W0 安全区域、FlightSession、Offboard 授权或起降控制。因此不能执行飞行验收或声称 S3 完成。现有零重力算法基线保留，厂商板卡固件与上游 SITL 身份分别记录。

## 复现与后续

按 [本机 SITL 使用说明](../../../../simulation/px4/README.md) 执行：

```bash
./scripts/build_px4_sim.sh --jobs 4
./scripts/with_px4_sim.sh python scripts/run_px4_sitl_smoke.py --duration 30
```

后续补齐 S0 本机 VIO/nvblox 固定数据样例；推进 S1 AircraftState/UUID/暂停协议/FlightSession mock，并按 S3 修复坐标和时钟故障契约、提供 W0 区域。通过这些门槛后再进入 S4 飞行控制。Jetson 继续延期。

Git 归档的 px4.log 仅清理行尾空白；逐字节原始日志仍保留在上述缓存会话目录。
