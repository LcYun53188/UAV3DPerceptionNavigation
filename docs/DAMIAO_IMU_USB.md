# 达妙 USB IMU 与视觉里程计融合

## 当前实现与数据流

新增 `damiao_imu` ROS 2 Python 包和 `uav_bringup/damiao_visual_odometry.launch.py`。现有启动入口保持原行为，只有显式运行新入口才启用达妙融合。保留 VINS 可选性和完整 Isaac 系列。

```text
OAK-D 图像 + OAK-D 原生 IMU → cuVSLAM → visual_odom_guard ─┐
或独立启动 VINS/其他视觉里程计 → visual_odom_guard ────────┤
达妙 DM-IMU-L1 → 上位机 USB → /damiao/imu/gyro ──────────┤
                                                       ↓
                                        robot_localization 三维 EKF
                                                       ↓
                                      /uav/localization/odometry
                                                       ↓
                                      后续 nvblox / EGO 定位适配
```

EKF 使用视觉六自由度位姿和达妙三轴角速度，不融合视觉派生速度、达妙内部姿态或加速度。它改善的是导航使用的融合里程计；**不会向 cuVSLAM/VINS 回灌 EKF 输出，也没有实现视觉前端外部状态先验接口**。nvblox/EGO 的消费接口仍需按主开发手册接入。

达妙加速度独立发布到 `/damiao/imu/accel`，供单位与标定检查。两个话题均为 `sensor_msgs/Imu`，无效字段使用 covariance[0] = -1。没有合成“同步原始 IMU”话题。直接将此驱动替换 VINS/cuVSLAM 的 IMU 输入仍需实现时间对齐、成对原始采样输出和相机—IMU 标定。

## 协议与实现边界

参考用户指定的[达妙源码固定版本](https://gitee.com/kit-miao/dm-imu/tree/39a0c4e3ebeae719310e1e3d717ab9bddeb69e3b)：说明书 V1.2、ROS1 Noetic 与 ROS2 Humble 示例。新驱动独立实现，不复制示例中的 build/install/log 文件。

- USB CDC，默认 921600 baud、设备 ID 1；推荐 `/dev/serial/by-id/...` 稳定设备名。
- `55 AA + ID + RID + payload + CRC16(小端) + 0A`。RID 1/2/3 为三轴 float32 小端，19 字节；RID 4 四元数，23 字节。
- 厂商 CRC16：使用 CCITT 表、初值 FFFF，但递推为左移 **1** 位（不是标准 CCITT 的左移 8 位），默认覆盖帧头；已用真实硬件帧验证。若确认固件采用不含帧头的 CRC，显式设置 `crc_mode:=exclude_header`；不会自动放宽校验。
- 全部类型参与正确分帧，仅发布 RID 1 加速度和 RID 2 角速度。检查 CRC、帧尾和有限值，支持分包、粘包和错误重同步。
- 只读接收，不自动设置模块、保存参数或烧写固件。需预先使用厂商工具启用 USB 加速度和角速度输出。
- 帧中没有硬件采样时间。消息使用上位机读到串口数据时的 ROS 时间；同一批数据可能共享时间戳，USB 延迟和积压不能由该时间戳恢复。该驱动用于真机系统时间，不与仿真 `/clock` 混用。
- 无新帧不重发；断连后每 2 秒尝试重连并清空接收缓存。日志报告无新角速度和连接失败，尚未提供导航有效性/失效切换状态机。

## 安装与运行

```bash
uv pip install --python .venv/bin/python pyserial
./scripts/with_venv.sh colcon build --build-base build_uav --install-base install_uav \
  --packages-select damiao_imu uav_bringup
```

当前环境已完成驱动构建与 7 项自动测试。`uav_bringup` 完整 colcon 构建因已有依赖尚未构建而失败；目前未完成端到端 EKF/视觉运行验证。

完整运行还需要已构建的 `robot_localization`、`nav_guard`、`oakd_perception` 和 Isaac Visual SLAM 及其依赖；VINS 模式不需要启动 Isaac。串口须有当前用户读写权限。

**必须先确认实际固件单位**：ROS1 示例直接赋值不能证明所有固件输出单位。驱动故意不默认猜测，要求 `gyro_unit` 为 `rad_s` 或 `deg_s`，`accel_unit` 为 `m_s2` 或 `g`。原始静置加速度模约 1 对应 g，约 9.81 对应 m/s²；已知转角和时长的旋转用于检查角速度尺度。ROS 发布统一为 rad/s 与 m/s²。

以下命令仅适用于已经确认输出为 rad/s、m/s² 的模块：

```bash
# 先只检查 USB 数据，不启动视觉和融合
./scripts/with_venv.sh ros2 run damiao_imu usb_driver --ros-args \
  -p port:=/dev/ttyACM0 -p gyro_unit:=rad_s -p accel_unit:=m_s2

# 停止上面的独立驱动，再启动 OAK-D/cuVSLAM + 达妙 + EKF
./scripts/with_venv.sh ros2 launch uav_bringup damiao_visual_odometry.launch.py \
  port:=/dev/ttyACM0 gyro_unit:=rad_s accel_unit:=m_s2 \
  imu_x:=0.0 imu_y:=0.0 imu_z:=0.0 \
  imu_roll:=0.0 imu_pitch:=0.0 imu_yaw:=0.0
```

安装外参表示 `base_link → damiao_imu_link`，位置单位 m、角度 rad。示例零外参**仅适用于共点、同轴安装**，必须替换为实测安装关系。`base_link` 遵循前左上（FLU），通过静态 TF 表达模块实际坐标轴关系，不在驱动中猜测翻转轴。

接入独立启动的 VINS/其他视觉源：

```bash
./scripts/with_venv.sh ros2 launch uav_bringup damiao_visual_odometry.launch.py \
  start_oakd:=false visual_odom_topic:=/vio/odometry \
  port:=/dev/ttyACM0 gyro_unit:=rad_s accel_unit:=m_s2
```

外部视觉输入必须是 `nav_msgs/Odometry`，位姿表示机体 `base_link` 在连续 `odom` 系中的位姿，带合理协方差。若 VINS 输出相机/IMU 位姿或不同世界系，先做外参和参考系适配；这里的 guard 不能代替几何转换。不要把 `/uav/localization/odometry` 指回输入。

仅 EKF 发布 `odom → base_link`。外部 VINS、其他 EKF 或已有导航启动文件必须关闭同一 TF 的发布；不要同时启动两个完整定位入口。本入口关闭 guard 的拒绝后保持发布，避免把冻结位姿当新观测重复融合。

## 标定与验收

1. 检查实际端口、设备 ID、CRC 模式、输出频率和单位。驱动不依赖欧拉角或四元数输出。
2. 静置记录角速度偏置、各轴方差和加速度模长；分别绕机体各轴旋转，验证符号、尺度和 TF。当前驱动没有自动去零偏。
3. `gyro_variance` 默认 0.0004 (rad/s)²，独立驱动 `accel_variance` 默认 0.04 (m/s²)²，均为调试起点，需按实测噪声调整；EKF 过程噪声同样需调参。
4. 检查融合前后位姿、漂移、延迟和协方差；验证视觉断流、USB 拔插及视觉恢复。EKF 在缺少视觉时可能持续预测，输出存在不等于定位有效；接入飞控前必须配置独立定位健康监测与失效动作。
5. 六轴 IMU 无绝对航向来源，不能长期消除 yaw 漂移，也不能仅凭 IMU 获得长期可靠位置。

自动验证命令：

```bash
PYTHONPATH="$PWD/src/damiao_imu" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  ./scripts/with_venv.sh python -m pytest src/damiao_imu/test -q
```

覆盖协议拆包/粘包、CRC、错误重同步、非有限值、单位换算，以及伪终端模拟 USB 的 ROS 消息发布、停止更新和断连重连。伪终端测试不替代真实模块、电气连接、时间同步和飞行验证。

## 2026-09-24 实机 USB 验证

已连接设备 `/dev/serial/by-id/usb-DM-Tech_DM-IMU-L1_DMIMU20250212-if00`（当前 `/dev/ttyACM0`）。只读采集约 8 秒，确认 ID 1、四种 RID 均约 1000 Hz，约 80 kB/s；厂商 CRC 算法覆盖帧头时校验错误为 0，不含帧头模式不适用于该模块。

本次采样加速度模均值约 10.075，支持其单位为 m/s²，仍需确认静置条件、零偏和比例误差。原始角速度均值约 `[0.000311, 0.000047, -0.007130]`，静态数值不足以单独确认角速度单位；目前按官方 ROS1 示例采用 rad/s 进行通信测试，仍需已知转角实验验证。未据此自动改写零偏、噪声或固件参数。

实机帧发现厂商 CRC 的非标准递推，已修复并纳入固定报文回归测试。当前 7 项自动测试通过。以上不表示视觉/EKF 端到端或飞行验证已经完成。
