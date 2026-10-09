# 仓库 BT VIO 飞行接入与初始闭环

2026-10-09，**0.8 m 起飞→15 s 悬停→原生降落连续两轮 PASS**。
这是真实 Gazebo x500 同机双目/IMU→cuVSLAM→归一化位姿→固定对齐→PX4 EKF→
唯一 FlightServer→BehaviorTree.CPP 的控制闭环，无 GNSS、无合成位姿/速度输入。
**原定 1.5 m 悬停仍失败，WH-F01 的完整高度验收尚未通过；航点、返航、避障未执行。**

| 低高度闭环运行 | 悬停阶段墙钟 / 最大真值漂移 | 最大真实位移 | EV 写入 / 各 aid 融合 | 连续 READY |
| --- | --- | --- | --- | --- |
| `7280d119-ebaf-4019-8927-89bb99dfdc8c` | 17.02394 s / 0.108158 m | 0.807911 m | 878 / 872 | 41.62027 s |
| `273b5b5f-0efb-4787-88ef-751d600f9fa2` | 17.01388 s / 0.087332 m | 0.790931 m | 896 / 892 | 42.54000 s |

两轮根任务均 `SUCCEEDED / LANDED_AND_DISARMED / cleanup_confirmed=true`，
BT 单次派发，三个步骤全部接受/完成，树终态 SUCCESS。起飞到点真值误差分别
0.095385/0.098659 m；悬停配方为 15 s，记录到下一阶段包含稳定等待。
最终真实着地并解除武装，VIO 源无故障，未知速度保持 NaN，EV velocity 未融合，
全部所属进程清理完成。独立重放真值漂移、原样本时间、场景资产及 BT 结果通过。
完整 ULog 扫描确认全部本轮参数覆盖，包括 GNSS/光流禁用、EV pose-only、
`EKF2_DELAY_MAX=160`、`EKF2_EV_DELAY=0`、Offboard 丢失降落策略。
SDK 参数服务与新鲜 NVIDIA 渲染日志保留。GID 校验使用唯一图端点，Jazzy 回调
未提供发布者 GID；这不是端点身份认证。理想针孔参考模型不是 OAK-D Pro W 实机标定。

## 接入与修复

- 仓库使用独立区域 profile、生成场景/模型/外参/纹理摘要、校准 ID 与所属分区。
  原 W0 仍验证原 upstream 文件摘要。飞行入口要求已通过的 WH-V01/WH-V02
  原始前置条件，并显式锁定 640×400、15°、地面纹理、0.8×、NVIDIA 无界面、队列 1。
- 持续 EV 会话只写位姿和健康状态；唯一 FlightServer 写 command/setpoint/offboard。
  固定地面对齐覆盖解锁仍着地、起飞、接地过渡。新鲜状态缺失、身份/reset/时钟变化
  或超时锁存，不在落地后恢复旧流。
- Fast DDS 先发现 GID、后补齐节点名称。在未对齐的启动期间，元数据未完成会
  阻止对齐；对齐后未知/缺失/替换控制端点直接失效。
- 无 GNSS 默认 Loiter 需要全局位置，Position 模式又需要遥控输入。本机无遥控器，
  因此 FlightServer 的 PREFLIGHT 所有者先发送固定地面参考并请求 Offboard；
  **只在 PX4 原生预检通过后才由 BT 解锁**。准备阶段没有 ARM 命令，健康丢失/超时停止并锁存。
- 对齐层新增显式 `bounded_gap`：仅拒绝单个协方差超限样本，不截断协方差，
  不重发旧 EV，不推进最后有效采样或接收时间。200 ms 期限不变，持续坏样本仍失败。
  默认地面审计保持 strict 对齐策略。
- 0.8× 世界下，1 Hz land 状态对应约 1.25 s 墙钟。仓库单独声明 land 接收期限
  1.5 s，原 ROS 发布年龄上限仍为 1.2 s；其他状态和 VIO 200 ms 期限不变。
- 准入和最终观察文件原子发布，飞行证据落盘后才通知监督器；保留唯一网关直到
  最终图/源/着陆状态记录完成，再清理所属进程。

130 项相关测试通过；mission/bridge 两个包构建通过，vendor 补丁及 gitlink 检查通过。

## 保留的失败证据

| 运行 | 结果与原因 |
| --- | --- |
| `f0c88e87-3a7f-4969-8367-cbdaa28e6500` | 未解锁；网关加入时源因发布者检查退役 |
| `4e0e4573-40dc-4bb2-b8c8-5d23d610ca53` | 未解锁；诊断确认 `_NODE_NAME_UNKNOWN_` 启动窗口 |
| `5a1bade3-ba7e-4f45-8bb1-610755b6aeb6` | 未解锁；READY，但默认 Loiter 原生预检不通过 |
| `031a511e-463a-4891-b7d7-70472ee1e7f1` | 未解锁；FailsafeFlags 没有 MESSAGE_VERSION，网关构造失败 |
| `28874090-60a0-42df-bb71-eb01aa46abf7` | 未解锁；Position 模式选择成功，但无遥控输入，原生预检仍不通过 |
| `3aa848c4-cdd7-46bb-b4b6-8cf41843cd01` | 实际上升约 1.22 m；对齐后协方差单帧 0.250259 >0.25，严格策略退役，随后真实降落/解除武装 |
| `18263fb1-06d1-44f6-bdbd-a7fce02ac6a8` | 实际进入 1.5 m 悬停阶段；对齐坏样本持续超过 200 ms，7 帧拒绝后失效，真实降落/解除武装 |
| `2631415c-f7ba-43b7-b676-5c3e820b8d27` | 0.8 m 起飞、悬停成功，漂移 0.089416 m；降落时 land 墙钟年龄 1.207 s 触发旧 1.2 s 接收检查；VIO 健康，实际降落/解除武装 |

637 个原始文件（含全部完整 ULog）按原字节归档，日志无损压缩。部分失败轮的
子进程 JSON 在退出时中断，原始截断字节照存，不补写为成功证据；独立飞行与
完整参数重放仅用于两个成功轮。摘要列出全部失败，不用低高度 PASS 替代 1.5 m。

## 运行与重放

```bash
./scripts/sim.sh px4-vio-sensors --flight hover-low --normalize --scene warehouse \
  --warehouse-floor-texture --camera-pitch-deg 15 --image-resolution 640x400 \
  --quality-policy bounded_gap --real-time-factor .8 --render-device nvidia \
  --sdk-image-depth 1 --headless-rendering --ekf-delay-max-ms 160

python3 scripts/assess_warehouse_flight.py .cache/simulation/vio-sensors/<run-id>
python3 docs/validation/simulation/2026-10-09-warehouse-bt-flight/check_archive.py
(cd docs/validation/simulation/2026-10-09-warehouse-bt-flight && sha256sum -c SHA256SUMS)
```

`--flight hover` 保留原定 1.5 m 配方；`--flight sequence` 已接入仓库航点配方，
但尚未验收。`--duration` 仅适用于未解锁传感器观察；飞行以任务终态结束，
观察墙钟上限 280 s，不把 manifest 中旧的 duration=120 当作实际飞行观察长度。
下一步先改善 1.5 m 的实际 VIO 观测/协方差，重新验证同配置源前置条件，
再完成原定高度 WH-F01、航点/返航 WH-F02 与实际停源/reset 控制交接 WH-F03。
