# 同低负载配置的仓库运动验收

2026-10-09，WH-V02 **PASS**。运行 ID：`ad713ed2-73b9-41bf-9771-ee06dde79db4`。
与已通过两轮 WH-V01 的图像/IMU负载配置相同：640×400、15°下倾、仓库地面纹理、
`bounded_gap`、0.8×、NVIDIA、图像队列深度 1、无界面。运动载台不向 PX4 写 EV，
因此不传 `--ekf-delay-max-ms`；160 ms 缓冲仅属于前两轮同机 EKF 融合配置。

```bash
./scripts/sim.sh px4-vio-sensors --normalize --motion --scene warehouse \
  --warehouse-floor-texture --camera-pitch-deg 15 --image-resolution 640x400 \
  --quality-policy bounded_gap --real-time-factor .8 --render-device nvidia \
  --sdk-image-depth 1 --headless-rendering --duration 120
```

| 指标 | 实测 |
| --- | --- |
| 墙钟观察 | 120 s |
| 归一化位姿配对数 | 2069 |
| 位置 RMSE / 最大 | 0.025974 / 0.037782 m |
| 姿态最大误差 | 0.579107° |
| 三轴运动范围 | 0.999529 / 0.799493 / 0.599541 m |
| yaw 范围 | 0.799869 rad |
| 原时间真值审计区间 | 16.920–99.648 s |
| 最后真值 / 位姿 | 99.684 / 99.640 s |

原始检查全部通过，包括持续源、结束覆盖、IMU 激励、新鲜度、协方差和真实静态 TF。
仅一次初始刚体对齐，没有缩放或轨迹拟合；真值仅观察，不输入 SDK。
原生参数服务确认 `image_qos_depth=1`、`tracking_mode=1`；新鲜渲染日志确认
NVIDIA RTX 4070 Laptop GPU。不是队列占用测量，也不是实时/UI稳定性验收。
PX4 始终未解锁，EV/command/setpoint/offboard 四类输入均无发布者，所属进程清理完成。

43 个原始文件及生成资产按原字节摘要归档，日志无损压缩。独立重放原 SDK
误差、归一化运动检查、诊断及仓库验收一致。重放命令：

```bash
./scripts/with_venv.sh bash -e -c 'source .deps/px4-msgs-install/setup.bash; source .deps/mission-install/local_setup.bash; python docs/validation/simulation/2026-10-09-warehouse-low-load-motion/check_archive.py'
(cd docs/validation/simulation/2026-10-09-warehouse-low-load-motion && sha256sum -c SHA256SUMS)
```

同时增加纯策略 `FlightAlignedPoseStream`：只有先在未解锁着地状态完成对齐，
再收到调用方明确确认的空中状态，才可持续转换原样本；旧地面审计仍拒绝空中状态。
失效、身份变化、时间倒退与超时均锁存，落地也不恢复旧流。原流及新策略共 27 项测试通过。
**策略尚未接入实际飞行启动器，不是飞行实测结果**。

WH-V01/WH-V02 源前置条件已有此低负载配置的实测证据。下一步是仓库场景与模型摘要
准入、持续 EV 会话、唯一 FlightServer/BT 控制接入，再运行 WH-F01 起飞→悬停→降落。
WH-F01/F02/F03/N01 仍为 NOT_RUN，尚无仓库 VIO 悬停、返航或避障完成证据。
