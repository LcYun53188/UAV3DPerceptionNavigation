# VIO 悬停准入首批验证（2026-10-08）

完成源适配与实际融合门控的软件首批实现。**没有完成 OAK-D Pro W → VIO → PX4
室内悬停验收**；当前 USB 枚举没有相机或达妙飞控，未发送实机控制或外部定位数据。

## 已验证

- 最终回归 **288 passed / 106.48 s**：桥、任务后端、BT 生命周期和既有审计；见
  `pytest-final.xml`。其中新的 VIO 专项 **64 passed / 0.38 s**，见 `focused-final.xml`。
- 隔离 domain 92 的真实 DDS 消息流契约 PASS：run
  `e5d1a0c6-048b-4ec8-89cd-40820e60a5dd`，见 `synthetic-dds.json`。
  输入是合成的 Odometry 与 VisualSlamStatus；验证健康、跟踪丢失、跟踪恢复、
  发布者更换后的失效锁存及没有 FMU 输入发布者。不是相机或 cuVSLAM 算法测试。
- 受管 SITL 缺失 VIO 的预检拒绝 PASS：run
  `d8ec416c-8796-4ae9-805d-705503aa1bde`，见 `negative-assessment.json`。
  DDS/clock/QGC 通过，但 VIO 遥测不齐，后端报告 `VIO_TELEMETRY_WRITER_COUNT`。
  20 s 预检窗口中没有 BT 飞行步骤，控制输出 0、命令 0，状态记录全为 DISARMED，
  最终着地/未解锁。原始 `flight-observation.json` 和 supervisor 结果仍保留
  passed=false，进程 exit 1；独立负向评估通过不能改写为飞行成功。
- 同一试验中的标定 ID 是合成 `aaaa…` 占位 ID，不能代表真实设备标定。
- 在受管 SITL 之外运行 monitor 并设置 emit_px4=true，实际在创建 FMU 发布者前
  拒绝，提示要求 owned SITL 和共同仿真时钟。硬件时间映射尚未实现。
- 正常监视默认不创建 FMU 发布者；转换 round-trip、大小轴/姿态/协方差、无效值、
  不合理置信度、输入时效、时间回退/跳变、源/主 EKF 替换、瞬时跟踪失效覆盖等
  均有针对性测试。

独立 W0 飞行回归结果见下方补充；这些回归不要求 VIO，不作为室内 VIO 飞行证据。

## 适用边界

- 受管 FlightServer 仍限定本机 instance 7 / domain 78 / W0 场景模型 hash。
  开启 `--require-vio` 才使用新门控，默认入口保持原有控制回归用途。
- 门控同时要求源健康、标定/会话绑定、主 EKF=0、四类成功 EV aid sample 和
  状态标志，连续健康 2 s；任意缺失不能通过。默认固定 PX4 DDS 缺少 selector 和
  四个 aid source，所以当前没有可通过正向验收的真实 VIO profile。
- 源、时钟与 EKF 的安全处置仍需动态验收。本批证明软件拒绝条件；没有通过失去
  VIO 后的真实飞行降落测试，也不承诺失定位时能定点保持。
- 源 adapter 当前没有 cuVSLAM 显式 reset 服务代理。消息无 reset_counter，适配器
  暂以 0 标记，依靠采样/位置/姿态/GID 连续性发现异常；不能声称检测所有内部小幅 reset。
- 传感器内外参、实际跟踪率/漂移、VIO 到 PX4 坐标时间映射、无隐藏 GNSS 辅助的
  真实悬停仍待完成。完整接口和下一步见 [VIO 悬停](../../../VIO_HOVER.md)。

归档只收录白名单结果和日志，不含运行授权文件、QGC 配置或 rootfs。
`implementation.sha256` 标识最终实现，`SHA256SUMS` 校验归档内容。

## 最终 W0 控制回归

最终源码 run `78cab9cf-0187-44c0-81ba-7621ae0dab06` 经 BT 完成起飞、两个航点、
悬停、返航、原生降落；Action SUCCEEDED / LANDED_AND_DISARMED / cleanup_confirmed=true。
逐步骤授权恰为 0–6；最大独立真值位移 4.19030 m。两段悬停分别 32.009 s / 5.029 s，
最大漂移 0.08481 m / 0.10097 m。最终未解锁且着地，DDS/clock/QGC 均通过，
`vio_required=false`。见 `w0-full/`，该通过不能证明 VIO 能力。

该次记录的所有 source_sha256 与最终源码/BT 可执行文件一致；两个最终 SITL 会话
进程组均无存活成员，见 `process-audit.json`。本轮没有主动重启旧算法地图会话。
正常接口/桥构建 2 包通过（11.1 s）；隔离任务构建 4 包通过（最终增量 1.82 s）。
