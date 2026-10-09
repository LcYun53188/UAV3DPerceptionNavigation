# 1.5 m VIO 飞行画面与 UI 诊断

2026-10-09，本机 PX4 1.16 / Gazebo Harmonic / QGC / cuVSLAM。两轮均使用已有 NVIDIA、640×400、图像队列深度 1、0.8×、15° 下俯、地面纹理、EKF 最大延迟 160 ms 配置，显式新增 Gazebo GUI、QGC、RViz、只读双目窗口。UI 增加负载，结果属于诊断，不继承无界面配置验收。

| 运行 | 实际结果 | 终态 |
| --- | --- | --- |
| `12d64f92-74e9-4221-b212-0a95efbbefbf` | 最大真实位移 1.4759 m；悬停 2.6843 s、漂移 0.0511 m；`VIO_HEALTH_LOST:VIO_SOURCE_LOST`，6 个对齐协方差拒绝样本后 `VIO_RECEIVE_GAP` | PX4 自动降落、锁定；关闭 UI 后完整进程清理确认 |
| `8615e793-e4c8-4e98-9e09-8e3ae1fd6e1e` | 最大真实位移 1.2731 m，未进入 HOVER；同样的任务失败原因，4 个对齐协方差拒绝样本后 `VIO_RECEIVE_GAP` | PX4 自动降落、锁定；控制网关与 EV 源已停止，UI 与模拟器保留，尚未做最终进程清理 |

第一轮 `0023`～`0025` 是实际 HOVER 图像，世界高度约 1.416、1.442、1.460 m。左目 Shi-Tomasi 候选角点均达到 500 的配置上限，8×5 网格全部 40 格有候选点；相应归一化世界 z 方差约 0.08161、0.09920、0.11860 m²。较低高度 0.092 m 时为 278 个候选点、26 格覆盖。这不支持“飞行画面几乎没有纹理”的简单解释。

候选角点不是 SDK 实际跟踪点、双目匹配点或内点，500 是截断上限，不能据此认定 VIO 观测充分。图中地面及货架存在重复图案；匹配歧义、升高后的深度约束、悬停缺少视差和保守初始协方差传播仍是待区分因素。本次没有测量 SDK 内点数量，未证明哪个因素是根因，也没有解决 1.5 m 悬停失败。当前 vendor 中 observations 可视化 helper 被注释，启用参数不会直接提供所需的跟踪证据；未改动 vendor 或放宽安全门限。

第一轮监视窗口 PX4 local 订阅缺少 `/px4_7`，高度字段为空；分析通过同时间戳归一化 pose 的单调接收时刻与真实 flight-truth 关联，保留时间差，不能当作严格传感器同步测量。第二轮已修正话题与版本，记录 PX4 z、真实图像 ROS 时间戳及接收单调时间。图像以低频诊断记录，未修改 SDK 输入、EV 时间戳、协方差或控制流。

下一项对照应先在独立传感器载体上比较非重复近场纹理与更大下俯角，验证同一高度的位姿误差、协方差和新鲜度；新配置通过源验证后再运行 1.5 m BT 任务。不能直接沿用 15° 配置的前置通过结论。

原始 JSON、日志使用 gzip 保存原字节；代表性左右目 PPM 亦无损压缩。`image-analysis.json` 为派生指标，检测参数写在文件中。第二轮为 UI 保留期间快照，没有 `result.json` 最终清理收据；不得将其标为完成验收。校验归档：

```bash
cd docs/validation/simulation/2026-10-09-vio-flight-ui
sha256sum -c SHA256SUMS
```

复现（诊断，预期可能失败；只在已隔离且前置验证通过的 SITL 执行）：

```bash
./scripts/sim.sh px4-vio-sensors --flight hover --normalize --scene warehouse \
  --warehouse-floor-texture --camera-pitch-deg 15 --image-resolution 640x400 \
  --quality-policy bounded_gap --real-time-factor .8 --render-device nvidia \
  --sdk-image-depth 1 --headless-rendering --ekf-delay-max-ms 160 \
  --ui --diagnostic-ui --ui-hold-seconds 1800
```

只读双目窗口不能启动、暂停或重试任务。地面终态确认后停止控制源并保留窗口最多 30 分钟，`touch <运行目录>/close-ui` 请求管理脚本清理本轮进程。离线分析使用系统 Python 的 OpenCV：`python3 scripts/analyze_vio_flight_images.py <运行目录> --output <输出.json>`。
