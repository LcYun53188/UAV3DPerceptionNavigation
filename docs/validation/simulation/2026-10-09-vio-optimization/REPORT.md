# VIO 负载优化、坏样本隔离与飞行准入接入

本轮降低双目计算量，并将显式仅位姿 VIO 门控接入 FlightServer。最终配置的 120 s 墙钟未解锁融合有一轮通过，也有一轮跟踪新鲜度失败；尚未完成实时稳定性或 VIO 起飞、悬停、降落验收。这里的“VIO 自稳”指位置保持，PX4 Stabilized 模式自身并不提供定点保持。

## 实现与边界

- 新增 `--image-resolution 480x300`：相对 640×400 减少 43.75% 像素，保留宽高比、25 Hz 图像、250 Hz IMU、视场及基线；生成的 CameraInfo/标定摘要随分辨率更新。默认仍为 640×400。
- 新增显式 `--quality-policy bounded_gap`，默认仍为 `strict`。仅允许已绑定健康源在最近有效样本的原采样和接收期限内拒绝协方差异常样本。不截断协方差、不发布坏位姿、不重发旧位姿、不延长 200 ms 最后有效样本期限。跟踪、身份、reset、时间等异常仍走原失效路径；连续坏样本不能保持 READY。
- 所有观测到的位姿时间戳都参加单调性检查，拒绝的样本不能掩盖随后时间倒退。
- FlightServer 显式支持 `aligned_pose_v1`：检查 EV 位置/高度/航向实际融合，并检查 PX4 本地速度有效性、不确定度及 reset；不伪造外部视觉速度。原完整 Odometry/四类融合默认配置保留。
- `--require-vio` 选择独立 PX4 遥测构建，记录 profile；显式仅位姿配置应用冻结的 EKF 参数。相机源和持续 EV 写入会话尚未由该飞行启动器管理。

当前传感器融合入口要求未解锁着地并禁止飞行命令发布者；不能直接与 FlightServer 混用。本轮没有更改该约束，也没有把新传感器场景冒充既有 W0 飞行区域。实际飞行仍需匹配模型/场景的安全区域、持续 EV 会话以及同机闭环验收。仿真参考相机尚非 OAK-D Pro W 实机标定模型。

## 原始结果

每轮均保留原始 manifest/result、SDK/归一化数据、时序与融合日志及退出证据。`current_inputs` 表示采集时的输入摘要与最终 VIO 源代码一致；中间轮只作诊断。所有传感器轮均确认清理完成。

| 归档目录 | 配置 | 原始结果 | 解释 |
| --- | --- | --- | --- |
| realtime-reduced | 480×300，strict，SDK 队列 1，1.0×，120 s | FAIL | 融合样本年龄 208 ms；随后 tracking 年龄 216 ms。降低分辨率仍未消除实时失效，代码早于最终单调性补充。 |
| paced-intermediate | 480×300，bounded_gap，队列 1，0.8×，120 s | PASS | 最长连续 READY 65.80 s；有 EKF/就绪重新收敛，代码早于最终单调性补充。 |
| paced-current-failure | 最终代码，480×300，bounded_gap，队列 1，0.8×，120 s | FAIL | ROS 41.188 s 时 tracking 采样为 40.960 s，年龄 228 ms，正确锁存失效。 |
| motion-current | 最终代码，480×300，bounded_gap，独立三轴运动载台，120 s | PASS | 位置 RMSE 5.02 cm、最大 9.09 cm、姿态最大 1.31°；实际拒绝 1 个协方差异常样本后恢复有效输出。无 FMU 输入。 |
| paced-current | 最终代码，480×300，bounded_gap，队列 1，0.8×，120 s | PASS | 2310 个输入/回显，位置/高度/航向各融合 2308 次，最长连续 READY 114.22 s。 |

最终通过轮中，源停止后 106.61 ms 出现 SOURCE_LOST；该轮未发生协方差样本拒绝，因此实际拒绝与恢复的证据来自独立运动轮。最后融合时间与结束边界均为 101116000 us。完整 ULog 的 14 项 EKF 参数与 manifest 一致：EV 位置/高度/航向开启，EV 速度关闭，GNSS/磁/光流/测距/辅助全球/阻力辅助关闭，气压高度辅助保留。

最终通过轮 SDK Track 最大 123.37 ms，双目到 SDK 发布最大 128.81 ms。减小分辨率的实时失败轮分别为 141.80/143.27 ms。这些是不同运行的观测，不能据此宣称单一根因或确定性能提升。最终相同配置仍有失败，不把偶发成功当作稳定资格；0.8×也不等于实时通过。

运动轮误差大于上一轮 640×400 的约 1.36 cm RMSE；仍满足预先冻结的 0.15 m RMSE、0.30 m 最大误差、10° 姿态阈值。独立载台不代表飞机本体在飞行中融合有效。

## 验证与重放

282 项针对性测试通过，随后补充时间倒退回归并运行节点测试（4 项通过，包含 3 项已覆盖测试），合计覆盖 283 项。四个 ROS 包构建通过（2.19 s）；两组受管 vendor patch 检查通过。飞行门控的本地 reset 转发及默认四类融合保留有节点级回归。

```bash
./scripts/sim.sh px4-vio-sensors --normalize --fuse-pose --scene layered \
  --image-resolution 480x300 --quality-policy bounded_gap \
  --sdk-image-depth 1 --real-time-factor .8 --duration 120
./scripts/sim.sh px4-vio-sensors --normalize --motion --scene layered \
  --image-resolution 480x300 --quality-policy bounded_gap --duration 120
python3 docs/validation/simulation/2026-10-09-vio-optimization/check_archive.py
cd docs/validation/simulation/2026-10-09-vio-optimization
sha256sum -c SHA256SUMS
```

`check_archive.py` 重放原始字节收据、全部原始通过/失败结果、独立时序/融合/运动评估及完整 ULog 参数；失败轮保持失败。运动浮点重算允许 1e-12 的数值舍入差。摘要 `source_fault=VIO_WRITER_COUNT` 是源结束后写入者消失，不是运行期失败原因；停源前就绪长度和停止事件分别记录。

## 缺源启动检查

`--flight --require-vio --vio-fusion-profile aligned_pose_v1` 的 25 s 缺源检查先暴露
启动器仍选择默认构建的问题（missing-source-before-build-fix）；已修复。
修复后的 missing-source-current 确认独立遥测构建及全部 EV aid writer。
两轮原始结果均为 FAIL：关闭 GNSS 且没有 VIO 输入，`xy_valid=false`，在
DDS/定位预检阶段退出，未启动 FlightServer/飞行任务，全程 arming_state=1、landed=true。
这证明缺定位不能进入任务，但不是 FlightServer 门控拒绝的实测；后者本轮仅有
节点级测试。全部启动进程退出码已记录，含 QGC 清理强制结束。

## 后续接入

先解决当前 tracking 新鲜度长尾并重复长时检查，再冻结带双目/IMU的飞行模型、场景与区域证据。将持续 EV 会话交给受管飞行启动器，保持 FlightServer 单一控制写入，随后实测行为树起飞→位置悬停→降落及源失效处置。S6 与 S7 均未宣布通过。
