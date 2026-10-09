# 地面纹理与 30° 下俯 VIO 对照

2026-10-09，本机 PX4 1.16 / Gazebo Harmonic / cuVSLAM 15。新增显式 `--warehouse-texture-style unique`：2048 px 随机位置、大小、灰度的矩形图集，没有原 16×16 周期网格。只改变地面视觉纹理；碰撞体、货架、双目基线、传感器频率保持一致。新增 30° 下俯，实际 camera pose 与 optical TF 同时重新生成。22 项场景、变换及 UI 地面终态测试通过。

三轮使用 640×400、NVIDIA 无界面、图像队列深度 1、0.8× 和 bounded_gap；同机融合另用 EKF 最大延迟 160 ms。没有放宽 200 ms 新鲜度或 0.25 协方差门限。每轮均未解锁；独立载体由自己的物理力驱动，PX4 不接受该载体的位姿或控制。

| 运行 | 条件 | 结果 |
| --- | --- | --- |
| `43fe5128-f399-41dc-9687-a47bc2d7a4b4` | 新纹理、15°、120 秒独立运动 | PASS；位置 RMSE 3.096 cm，最大 5.055 cm，最大姿态误差 0.572°；归一化位置方差最大值 xyz 为 0.07838 / 0.15388 / 0.19625 m² |
| `36ba47bb-7274-46b9-816f-81d0b883c2a1` | 新纹理、30°、120 秒独立运动 | PASS；位置 RMSE 1.295 cm，最大 2.731 cm，最大姿态误差 0.411°；位置方差最大值 xyz 为 0.06738 / 0.05971 / 0.10744 m² |
| `327fc31f-b971-4b19-8036-fead92c54b86` | 新纹理、30°、120 秒同机地面融合 | FAIL；实际 EV 已进入 PX4 EKF，连续 READY 仅 30.180 s；原始 SDK 协方差持续升高后源撤销，未达到 110 秒连续 READY 门槛 |

两轮运动使用同一轨迹（载体中心高度 1.3±0.3 m，三轴及 yaw 激励），唯一显式差异为下俯角。30° 相对 15° 位置 RMSE 降低约 58%，最大 z 方差降低约 45%。各仅一轮，未证明重复性。历史原纹理 15° 低负载运动 RMSE 2.597 cm、最大 z 方差 0.20328 m²；历史运行不能作为本轮严格同期 A/B。独立运动通过并不代表实际飞机在地面能稳定初始化或悬停能通过。

## 同机地面融合失败证据

- 最后有效归一化样本为 34.68 s。原始 SDK 消息在 34.72、34.76、34.80、34.84 s 仍按时到达，记录 ROS 样本年龄约 16～24 ms。
- 这些样本的六维原始协方差对角项从约 0.0006～0.0019 突变到约 1.0。到 106.64 s 结束仍有相同高不确定度，共 1702 个原始位置方差超限样本。
- normalizer 记录 `VIO_UNCERTAINTY_INVALID` 并拒绝这些样本，未修改协方差或刷新最后有效样本时间。
- 到 34.888 s，最后有效样本 34.68 s 的年龄为 208 ms，源以 `VIO_SAMPLE_STALE` 退休；EV 接收侧锁存 `VIO_RECEIVE_GAP`，后续 EKF 定位失效／重置是停止供源后的结果。
- 已发布的归一化样本最大 z 方差只有 0.00193 m²，因为高不确定度样本被过滤。不能用这个数声称整段 SDK 原始协方差很低，也不能把本轮失败归因为 DDS 传输卡顿。

结束时的地面左目画面存在较大均匀区域；系统 OpenCV Shi-Tomasi（maxCorners=500、qualityLevel=.01、minDistance=8）检测到 53 个候选角点，灰度标准差 48.17。该图是结束时静态画面，不是带 SDK 内点测量的故障瞬间帧。它提示 30° 近地面视角可能需要更细尺度的非重复特征，但不能确定 SDK 退化的内部原因。当前 SDK 未提供已验证的跟踪内点观测；不得将候选角点数等同于真实跟踪质量。

因此 30° 新配置尚不允许进入 BT 飞行，现有 flight guard 仍只接受先前已验证的 15° 原纹理配置。1.5 m 悬停故障没有解决，也未执行导航或避障。下一步需补近地面细尺度纹理，并做同机静止交叉对照（15° 新纹理、30° 原纹理）及重复长时验证，区分纹理、视角与静止观测退化。

## 归档与界面

三轮完整源验证均已确认未解锁、着地及管理进程清理。JSON / 日志 / PPM 按需 gzip，保留原始字节；资产包含世界、模型、TF 与所有纹理，收据记录哈希。校验与独立误差重算：

```bash
./scripts/with_venv.sh python docs/validation/simulation/2026-10-09-vio-pitch-texture/check_archive.py
```

另一次 UI 启动存在双目窗口参数错误，已关闭、清理并修正；未计入上述三轮。失败启动的 manifest、result 和窗口日志单独保存在 `ui-startup-failure/`，不用于源验收。随后重新运行 60 秒未解锁 UI 融合诊断；GUI 增加负载，显式标为诊断，不能用于长时验收。

新增 UI 保留支持未解锁融合诊断：仅 fresh PX4 telemetry 同时确认 landed / disarmed 才保留窗口，先停止 EV 与归一化源。窗口最多保留 30 分钟；`touch <运行目录>/close-ui` 请求清理。关闭独立双目窗口不会发送任务或飞控命令。

```bash
./scripts/sim.sh px4-vio-sensors --fuse-pose --normalize --scene warehouse \
  --warehouse-floor-texture --warehouse-texture-style unique --camera-pitch-deg 30 \
  --image-resolution 640x400 --quality-policy bounded_gap --real-time-factor .8 \
  --render-device nvidia --sdk-image-depth 1 --headless-rendering \
  --ekf-delay-max-ms 160 --duration 60 --ui --diagnostic-ui --ui-hold-seconds 1800
```
