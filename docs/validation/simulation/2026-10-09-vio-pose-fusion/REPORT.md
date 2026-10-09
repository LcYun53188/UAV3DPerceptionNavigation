# 固定初始化对齐、显式仅位姿融合与 BT 回归

日期：2026-10-09。实现提交：`5ca7b85`（对齐/转换）、`af7716c`（门控/实际 EKF 审计）。

已交付固定仿真锚点对齐、保守不确定性传播和显式 `aligned_pose_v1` 配置。
**未解锁的合成位姿 → 实际 PX4 EKF 三类融合/停更检查通过**，默认四类融合与 W0
完整 BT 飞行回归也通过。尚未将实际 cuVSLAM 位姿接入该流程，不能称为 VIO 飞行。

## 对齐和转换

- 输入契约是已经归一化的 odom/base_link 位姿及世界固定轴完整协方差；不是原始
  SDK 右扰动协方差，也不是 cuVSLAM Odometry 的滑窗统计。helper 不负责 DDS 源
  身份/状态配对；未来接收层必须验证现有源契约、发布者和 VioStatus。
- 调用方提供已知仿真位置/yaw、锚点配置 ID、源会话/标定/reset，以及未解锁着地
  状态。初始化窗口 ≥2 s、≥40 样本、间隔 ≤0.081 s，平移 ≤0.03 m、转角
  ≤0.03 rad、初始倾斜 ≤10°，最新样本年龄 ≤0.2 s。
- 只冻结 yaw 和平移，不修改重力方向；绑定后从下一原始样本开始输出，不逐段
  对齐，不用 PX4 回读或真值修正观测。`alignment_id` 绑定锚点、源身份、原始
  初始化时刻、变换与协方差策略；源/标定/reset、时间或数值异常永久撤销实例。
- 配置锚点在本次**合成仿真**中视为精确已知，a/b 重复的摘要是明确的测试身份，
  不是设备标定证明。实机锚点误差、硬件时间映射以及真实源接收/健康配对尚未实现。
- SDK 没有初始/当前时刻联合协方差。令 J_current 传播当前固定轴误差，J_initial
  传播初始位置/yaw 及 yaw 对位移的耦合，使用未知时间相关下的 PSD 上界：
  C_out = 2·(J_current C_current J_currentᵀ + J_initial C_initial J_initialᵀ)。
  不假定独立，不把初始化误差当零。线性化传播后六轴方差仍须 >0 且 ≤0.25，
  因此实际 SDK 样本可能在对齐后被拒绝；本次未验证实际 SDK 在此上界下持续可用。
- 对齐输出为 `px4_local_enu`，转换到 PX4 NED/FRD。速度/角速度/速度方差为 NaN，
  velocity_frame=UNKNOWN，不估计或伪造速度。位置协方差旋转到 NED；姿态方差
  由完整角度协方差经 ZYX Euler Jacobian 得到。锁定 PX4 `ev_yaw_control.cpp`
  使用 getEulerYaw(quat) 与 orientation_var(2)，倾斜时不能直接用旋转扰动 z 方差。
  VehicleOdometry 只携带对角方差，跨轴/位置姿态交叉项不能传给 PX4。

## 显式融合门控

`VioGate` 默认 `full_odometry` 保持四类 aid 要求；FlightServer 仍只调用默认模式。
新模式必须显式声明 `fusion_profile='aligned_pose_v1'`，源 frame 必须为
`px4_local_enu`，要求位置/高度/航向实际 fused、创新正常、instance=0、唯一且新鲜，
再检查 PX4 local position/velocity 有效、有限、航向可控、无 dead_reckoning。
位置/速度 sigma 在 (0,0.5]，heading_var 在 (0,0.25]；local 源/接收年龄 ≤0.5 s。
源原始年龄仍 ≤0.2 s，flags/selector ≤1.5 s，其余融合 ≤0.5 s，稳定窗口 2 s。
五类 local reset 计数在绑定后变化会在回调锁存，不能被下一个正常消息覆盖。

配置固定 EV_CTRL=11、GPS_CTRL=0、MAG_TYPE=5、OF_CTRL=0、RNG_CTRL=0、AGP_CTRL=0、
DRAG_CTRL=0、EV_NOISE_MD=0、HGT_REF=3；单 EKF/单 IMU，气压高度 BARO_CTRL=1 保留。
额外 GNSS/磁/光流/测距高度/辅助全球位置或 EV 速度融合启用即拒绝。
PX4 自己估计的速度只用于健康检查，不输入 EV。该配置没有 BT/FlightServer 飞行入口。

## 实测结果

| 运行 | 判定 | 输入与实际融合 | 停更结果 |
| --- | --- | --- | --- |
| `pose-three-aids` `50a60413-ca89-4648-84cf-ebb5fec021ea` | PASS | 702 次输入及 DDS 回读；位置/高度/yaw 各 700 个 fused 样本；EV 速度 0 | 0.18495 s 后源失效拒绝；三类 last_fuse 停止推进；最终锁存 EKF local reset |
| `default-four-aids` `d42c15e9-447d-4862-8036-6c69064d49f0` | PASS | 默认独立合成 Odometry，位置/高度/速度/yaw 四类 fused，默认门控就绪 | 停更后拒绝，排空后四类 last_fuse 不再推进 |
| `w0-full` `02fc6f06-dedb-4654-9c0c-d528055c2bd2` | PASS | 实际 BT 起飞/两航点/悬停/返航/原生降落，步骤 0–6 各一次 | SUCCEEDED / LANDED_AND_DISARMED |

仅位姿合成源从非零 (2,-1,0.3) m、yaw=0.6 rad 对齐到已知仿真锚点 (0,0,0)、
yaw=0，实际发送独立固定零位置、NED yaw=π/2，与 PX4/真值无反馈关系。
DDS 回读的速度/角速度/速度方差全部为 NaN（JSON 以 null 编码），速度 frame=0。
`alignment_id` 包含在源会话中，原始 sample_stamp 与输出 timestamp_sample 一致。

停更前 ULog local-position 原始记录：xy/z/vxy/vz 有效、航向可控、dead_reckoning=false，
eph=0.06427 m、epv=0.04121 m、evh=0.06969 m/s、evv=0.03785 m/s。
运行时门控同样检查这些字段；简化 JSON 快照未保存 eph/epv，归档从同一时刻
ULog 补充原始记录、格式与 SHA256，供独立解析，不修改原始审计结果。

实际 ULog 参数记录核对全部覆盖项，包括 EV_CTRL=11、无 GPS/磁/光流/测距高度/
辅助全球位置/阻力辅助、保留气压高度及 UXRCE_DDS_SYNCT=0。
启动前磁/其他传感器存在不等于使用其融合，以生效参数与融合 flags 同时核对。
PX4 对观测噪声另有参数下限，本次不声称直接使用输出方差而无内部噪声下限。

W0 full 最大真值位移 **4.17785 m**，两段悬停分别 **32.00860 s / 0.12734 m** 和
**5.00386 s / 0.04859 m**，步骤接受/完成均为 [0,1,2,3,4,5,6]。这是既有 GNSS
已知区域控制回归，不是无 GPS 或 VIO 飞行。当前三个最终运行的源码指纹均一致，
原 W0/Agent 二进制与版本锁未修改。全部受管根进程已退出；未改动 vendor 源码。

## 保留的诊断

`diagnostic-stop-policy`（`97adb413-a863-4897-b178-375a422884c5`）原结果 FAIL：
三类融合/就绪/停更本已成立，但旧审计只接受最终 `VIO_TELEMETRY_STALE:source`，
实际 PX4 在失去 EV 后重置 local 状态，门控转为更强的 `VIO_EKF_LOCAL_RESET` 锁存。
修复审计要求：先在 ≤0.5 s 内因源失效/源过期拒绝，再允许终态为持续过期或 local
reset 锁存，同时要求融合停止推进；保留原运行失败，不改成通过，不放宽运行门控。
随后用最终实现重新执行上述完整审计。开发中间的其余缓存运行未作为最终证据。

## 验证与重放

140 项相关测试通过（`pytest.xml`），含非 identity 固定对齐、时间/源/reset/协方差
拒绝、未知速度、倾斜姿态 Jacobian 有限差分、未知跨时刻相关的随机 PSD 上界、
输入缓存冻结、隐藏辅助/无效 EKF 速度、默认四类要求、DDS 假速度和停更审计反例。

```bash
./scripts/build_px4_flight.sh
./scripts/sim.sh px4-vision-audit --vision-fusion-profile aligned_pose_v1 --duration 35
./scripts/sim.sh px4-vision-audit --duration 35
./scripts/sim.sh px4-flight --bt
python3 docs/validation/simulation/2026-10-09-vio-pose-fusion/assess_fusion.py
```

`assess_fusion.py` 独立核对聚合证据、最后融合时刻、NaN/时间/会话、ULog local 原始
记录和参数记录。它不是全量 DDS 时序重放；未归档全量 ULog。参数提取器扫描完整
本机 ULog 的所有相关 P 记录及历史，归档初始化 header 和提取结果；header 支持
启动参数重读，完整文件 SHA256 绑定本机缓存，不能凭 header 证明其后没有改参。
`payload-sha256.json` 保存原始未压缩摘要，`SHA256SUMS` 覆盖本档案文件。

下一步：接入实际 SDK 源的健康/会话/时间配对和受管对齐发布，先未解锁验证实际
VIO→EKF，再冻结相同物理机体/场景的飞行安全区域与退化处置，最后接入 BT 悬停。
独立运动载台不能向另一架静止 x500 提供飞行观测；实机标定/锚点误差/时间映射仍待开发。
