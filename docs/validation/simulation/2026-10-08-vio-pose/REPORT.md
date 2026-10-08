# SDK 标准位姿、reset 代理与质量退化（2026-10-08）

本轮交付 **SDK 位姿协方差归一化、连续健康绑定、reset 先撤销后转发、旧源永久停发**。
实际 35 s 静止及 40 s SDK reset 前置审计通过；**90 s 持续就绪失败**，SDK 位置
协方差超限，源正确锁存失效。尚未验证运动精度、外部视觉融合或 VIO 悬停，S6 未通过。

代码：`3eb1aa8`（归一化与源撤销）、`d5018ba`（实际 SDK/DDS 审计）。
没有向实机发送数据或控制，没有执行飞行或降低任何准入门限。

## 核验的 SDK 契约

本机 SDK 为 `15.0.0+74f0e317-modified`。SDK 头文件 cuvslam2.h 的 PoseWithCovariance
定义随机位姿为 mean_pose · exp(u)，协方差位于右扰动切空间；顺序为旋转三轴、平移三轴。
上游 cuvslam_ros_conversion.cpp 已调整到 ROS 平移/旋转顺序及 FLU 基，但未将右扰动
旋转到世界固定轴。上游 visual_slam_impl.cpp 用 base_link 相机/IMU外参定义 SDK rig，
因此本次参考点就是 base_link，无需再加入相机到机体原点的杆臂。

对上游 SDK pose_covariance 使用 A=diag(R,R)、C_world=A C_body Aᵀ，R 为当前
odom_from_base_link 姿态；保留完整位置/姿态交叉项。只去除允许误差内的浮点反对称残差，
不裁剪负特征值、不添加方差下限、不接受零/未知协方差。位置与姿态各轴方差要求
>0 且 ≤0.25（位置 m²，姿态 rad²），其余有限性、PSD、时间门限保持严格检查。

源码、SDK 头、原生节点/库指纹冻结在 simulation/px4/vio/pose_contract.json，
受管入口对漂移直接拒绝。节点另查询并核验 base_frame=base_link、odom_frame=odom、
tracking_mode=1、VIO-only、无地面约束、override_publishing_stamp=false。
calibration.json 绑定传感器 profile、TF、实际算法参数及原生实现 hash；三轮最终运行
配置 ID 相同：8085258e2b43d333b3e8a7e5d6227966d488480f6b67e5b03561994ed0507bec。
它是配置绑定，不是设备标定证明。

## 输出、时间与会话

- `/uav/vio/pose`：PoseWithCovarianceStamped，保留 SDK 原始位姿及采样时间，输出世界固定轴协方差。
- `/uav/vio/pose_status`：VioStatus，保留原始 sample_stamp、配置 ID、源 UUID 与代理 reset 计数。
- `/uav/vio/reset`：公开 Reset 代理；原生 reset 服务在本次运行重映射为 `/visual_slam/internal/reset`。

这里的 odom 仍是 SDK 初始局部坐标系；没有做 PX4 初始化对齐、全局航向对齐或硬件时钟映射。
SDK 未提供速度观测协方差，节点不推导速度，也不发布完整 Odometry 或 FMU 输入。
现有 FlightServer 仍要求四类 EV 实际融合；新 pose_status 没有接入该门控，不能据此获准飞行。

首次绑定要求连续健康 2 s。SDK 初始化阶段出现零或近单位协方差时拒绝、重新计时，
不会绑定最初一两个看似合格的样本；绑定后所有故障永久锁存，恢复健康也不自动重建源。
位姿/状态按原始采样时间精确配对，解决不同 DDS 话题回调到达顺序的歧义；等待上限
仍为 0.2 s，不重写旧时间。时间回退、接收断流、跳变、发布者变化、tracking invalid、
未知/超限协方差及输出多写者均拒绝。接收新鲜度及服务期限使用单调时钟。

reset 代理先递增计数并发布旧源失效，再转发 SDK 调用；失败、服务缺失或 5 s 超时
也不恢复旧源。实际 SDK 恢复输出后仍无标准位姿，重建必须启动新适配器会话。
重映射是受信任本机约定，不是 ROS 访问控制；计数只涵盖代理请求，不承诺识别全部
SDK 内部小幅 reset。源 UUID 在撤销后不变，计数变化且 valid 永久为 false。

## 最终实际运行

三轮均显示 Gazebo/QGC，使用官方 PX4 v1.16.2、Harmonic、同一参考双目/IMU与真实 cuVSLAM。
PX4 全程 arming_state=1、着地，四类 FMU 输入发布者均为零；没有 FlightServer。
参考模型仍非 OAK-D Pro W 标定模型，也没有测量噪声、曝光/USB 延迟或硬件时间映射。

| 归档 / run_id | 结果 | 证据 |
| --- | --- | --- |
| continuous-35 / `808b05da-269c-4384-8c9e-66a954951d31` | PASS，静止前置检查 | 656 个标准位姿，最近窗口保持原始时间/位姿、协方差 PSD/正方差及唯一写者；最后 5 s 静止漂移 0.07639 m |
| reset-40 / `aff9e9a9-fd94-476f-9898-82ccfb23ed02` | PASS，真实 SDK reset 撤销 | reset 前 581 个标准位姿；SDK 返回成功、日志确认重置并重新初始化；0.5 s 排空后无标准位姿，状态持续 RESET_REQUESTED / reset=1；原始 SDK 输出恢复 |
| quality-limit-90 / `aebc9f96-ba8e-4ff2-963f-1ef21a019f9e` | **FAIL，持续 VIO 就绪**；质量退化撤销正确 | 先连续健康约 39.77 s、输出 994 个位姿；随后竖直位置方差超限，源永久停发 |

reset 轮最后 5 s 的漂移数据来自触发前窗口，约 0.06422 m；触发后不再把源视为健康。
这些漂移仅是静止估计位置相对窗口起点的位移，**不是独立真值 RMSE 或绝对定位误差**。
所有实际运行的数据/采样/TF/CameraInfo及未解锁检查见各目录 result.json。
最终三轮全部输入 hash 匹配当前文件；current-input-check.json 保存逐项结果。
原生节点、适配器均 exit 0，所属根进程/进程组清理确认通过；QGC 超出 8 s 退出期限后
由所属进程组 SIGKILL，未声称所有进程优雅退出。

## 90 s 失败的直接证据与边界

最后一个标准位姿采样时间 45.24 s。下一 SDK 样本 45.28 s 的 body 竖直位置方差
约 0.37735 m²；转换后 world 竖直位置方差 **0.37545 m² > 0.25 m²**。
首个失效状态在仿真时间 45.344 s，reason=VIO_UNCERTAINTY_INVALID、reset=0，
对应启动后约 48 s。此后 SDK 原始位姿持续至 86.60 s，但没有重新发布标准位姿。
结束时 tracking 仍正常、传感器检查通过，不能仅凭 tracking 状态或有输出判定可飞。
后续 SDK 协方差继续偏大，晚段还出现近单位数值。

`assess_retirement.py` 直接读取压缩归档，独立重算拒绝样本的世界方差，检查状态持续
失效、源身份不变、排空后无输出及原 SDK 仍输出；retirement-assessment.json 为 PASS。
这仅证明故障撤销正确，**不将 90 s 稳定性失败改写为通过**。

SDK 为什么变得不确定尚未确认。静止缺少惯性初始化激励、场景特征/深度分布与算法
噪声假设是后续实验方向，当前数据不能将其中任意一项断言为根因。
尚未创建新的受控运动配置，不通过改名原 W0 场景/模型或跳过其冻结 hash 来执行飞行。
当前纹理场景、参考传感器模型也不能冒充原 W0 已知空旷区域配置。

## 保留的诊断失败与回归

- startup-rejection：首版在前两个有效样本后即绑定，随后初始化近单位协方差被正确拒绝，
  源过早进入永久失效；新增连续 2 s 的启动稳定窗口，绑定后的拒绝逻辑不放宽。
- tracking-rejection-before-pairing / tracking-rejection-after-pairing：旧版本有
  TRACKING_INVALID 锁存记录。精确配对解决跨话题读到上一帧的歧义，但不能据此证明
  两轮旧失败都来自回调重排；旧日志缺少分项时效，现已加入源时间/接收间隔/vo_state 诊断，
  长时与负载复验仍需继续。没有将旧失败隐藏或当作通过。
- 早期一次审计因 numpy.bool_ 无法 JSON 编码而没有保存最终 result；已转为原生 bool，
  并让进程先清理、再保存诊断数据，采集失败也保留原始记录。该部分缓存未计为通过。
- DDS reset 超时夹具最初因上游挂起服务占用观察者回调组而误判；将夹具服务回调组隔离。
  代理自己的超时/参数客户端和 steady timer 也使用独立回调组，保持超时期间状态可见。

数值/契约/传感器几何回归 **57 passed / 0.33 s**，pytest.xml；任务包构建 4 packages finished。
transport/ 四项为 domain 93 **合成 DDS 契约测试**，不是传感器或 EKF 证据：
成功、上游失败、缺失、5 s 超时均通过；模拟状态比位姿晚两帧约 80 ms，仍按原时间
正确配对。验证初始化单位协方差拒绝及完整 2 s 暖机、reset 在转发前撤销、持续停发，
没有 FMU 发布者。

本轮没有再次飞行 W0。上一轮完整 W0 回归的控制源及 Runner 二进制 hash 仍全部匹配，
见 current-input-check.json；新代码不修改现有 FlightServer、VioGate 或原始输入转换器。
managed vendor/EGO 检查通过，原有供应商子模块状态保留。

## 复现与后续

```bash
./scripts/build_px4_flight.sh
./scripts/sim.sh px4-vio-sensors --normalize --ui --duration 35
./scripts/sim.sh px4-vio-sensors --normalize --reset-source --ui --duration 40
./scripts/sim.sh px4-vio-sensors --normalize --ui --duration 90
.venv/bin/python docs/validation/simulation/2026-10-08-vio-pose/assess_retirement.py
```

前提是基础 PX4、任务消息和原生 VIO 节点已构建。90 s 命令是失败样本复现入口，
因算法结果可能变化，不承诺每次在相同时间超限；不得为取得 PASS 临时提高门限。
归档采用白名单，不收录 QGC 配置/缓存、rootfs 或授权私有 JSON；payload-sha256.json
校验压缩前数据，SHA256SUMS 校验归档文件。

下一步冻结独立运动验证配置和只初始化一次的真值对齐，评估三轴/旋转精度及 SDK 质量，
再实现独立显式位姿融合配置及 PX4 估计速度健康检查。速度观测缺失时不伪造速度，
也不把当前要求四类 EV 融合的门控静默降级。通过运动和实际融合后，才执行 VIO 的
BT 起飞→30 s 悬停→原生降落，再进入导航/避障阶段。
