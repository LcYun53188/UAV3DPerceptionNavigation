# 仓库 VIO 漂移与观测条件对照

更正：本报告的相机下倾一致性仅验证了生成 SDF／frames.json。后续发现实际静态 TF 发布器仍写死为水平旋转，故本报告 pitched-vio 与 pitched-stereo-diagnostic 的运行时外参不匹配；不能据此判断 IMU 融合算法优劣。原始证据与失败标记保留。修复及新实测见 [运行时外参报告](../2026-10-09-runtime-mount-fix/REPORT.md)。

本轮新增按轴误差/协方差时序诊断、IMU 与真值微分的近似残差，以及显式地面纹理、相机下倾和纯双目对照。仓库 VIO 仍未通过；纯双目诊断的运动误差检查通过，但没有归一化 VIO 源，不允许据此飞行。所有轮次未解锁、无 FMU 输入，并已清理。

## 已确定的观测

上一轮 baseline-vio 的位置 RMSE 17.51 cm，按轴分别为 X=3.38、Y=16.93、Z=2.95 cm。固定初始对齐后，Y 偏差随时间累积；ROS 61.24 s 起的 10 s 窗口平均 Y 误差约 -14.5 cm，121.24 s 窗口约 -31.2 cm。没有逐段拟合、尺度修正或时间/偏置校准。

按原时间戳对机体真值加 IMU 杆臂后微分，再转回机体系并计入重力，得到 IMU 比力近似残差各轴 RMS 0.000906/0.001252/0.001579 m/s²，均值约 1.5e-6/8.6e-7/6.2e-6 m/s²。该计算只供诊断，包含有限差分误差；暂未发现显著加速度轴向或重力符号错误，不代表硬件标定或物理同步证明，也未验证陀螺偏置。

实际 CameraInfo K 与生成内参一致，右 P 的 Tx=-fx×0.075。查阅锁定 SDK ROS 源确认 FillIntrinsics 使用 K、FillExtrinsics 使用 TF，P 的 Tx 不会叠加第二次基线。相机下倾比较同时更新 SDF sensor pitch 与光学 TF（R_y(15°)×原光学旋转），IMU 外参保持原值；测试验证两者一致。

## 单变量对照结果

| 目录 | 配置 | 原始 SDK 位置 RMSE / 最大 | VIO 资格 |
| --- | --- | --- | --- |
| baseline-vio | 上轮墙面纹理、水平双目、480×300、120 s | 17.51 / 33.50 cm | FAIL：无有效归一化源 |
| floor-vio | 仅新增连续地面纹理，其他设置同基线 | 22.83 / 38.64 cm | FAIL：无有效归一化源 |
| pitched-vio | 在地面纹理基础上仅下倾双目 15° | 18.97 / 34.19 cm | FAIL：仅发布 2 条归一化位姿后因连续质量异常撤销；最终 VIO_RECEIVE_GAP |
| pitched-stereo-diagnostic | 同地面/下倾/分辨率，SDK tracking_mode=0，无 IMU 融合、无归一化器 | 8.70 / 16.57 cm | 不适用 VIO；原始 passed=false，diagnostic_passed=true |

纯双目诊断的三轴运动、转向、源时效/连续性、误差与惯性响应审计通过，姿态最大误差 6.34°。它仍不具备审查过的 VIO 质量/融合源，入口禁止与 normalize/fuse/reset 合用；仓库验收器即使看到原始通过标记也拒绝此诊断模式。SDK 位姿不能直接替代 VIO 飞行输入。

这些是同参数/同场景/同载台运动规则的独立运行，不是同一录制输入回放。纯双目结果优于 VIO 是排查 IMU 融合的线索，尚不能把算法模式、初始化时机、图像选择、负载或 SDK 内部估计中的某一项判为唯一原因。地面纹理与下倾也没有通过 VIO 门槛，未设为默认。

## 实现与验证

- `diagnose_vio_motion.py` 使用原始采样时刻插值真值、单次初始刚体对齐，输出每轴 RMSE、10 s 窗口偏差、原始 SDK 协方差及归一化位姿覆盖。
- `--warehouse-floor-texture` 与 `--camera-pitch-deg 15` 是显式对照；默认场景、水平相机、分辨率、IMU参数和全部质量/新鲜度门限保留。新增地面材质没有碰撞体，不改变障碍净空。
- `--diagnostic-visual-only --motion` 是未解锁、无归一化源的诊断模式，始终不算 VIO 通过。
- 17 项相关测试通过；随后新增 IMU 残差回归并运行诊断器 2 项测试，其中 1 项已覆盖，合计 18 项。Python 编译、两组 vendor 检查及上轮六轮证据重放通过。没有修改 SDK 原生代码、硬件或控制输出，没有新增飞行验收。

```bash
./scripts/sim.sh px4-vio-sensors --normalize --motion --scene warehouse \
  --warehouse-floor-texture --camera-pitch-deg 15 --image-resolution 480x300 \
  --quality-policy bounded_gap --duration 120 --ui
# 仅供模式对照，不能准入飞行：
./scripts/sim.sh px4-vio-sensors --motion --diagnostic-visual-only --scene warehouse \
  --warehouse-floor-texture --camera-pitch-deg 15 --image-resolution 480x300 --duration 120 --ui
python3 scripts/diagnose_vio_motion.py .cache/simulation/vio-sensors/<run-id> --output /tmp/motion-diagnosis.json
python3 docs/validation/simulation/2026-10-09-warehouse-diagnosis/check_archive.py
cd docs/validation/simulation/2026-10-09-warehouse-diagnosis
sha256sum -c SHA256SUMS
```

归档保留四轮完整传感器/SDK/归一化/真值/IMU数据、生成资产、日志与逐文件原始字节摘要。重放独立误差、按轴诊断和仓库验收，原始 passed=false 保持不变，纯双目 diagnostic_passed=true 单独保留。无完整 ULog或同机飞行结论。

## 下一步

先采集/核对 SDK 实际消费的双目和 IMU 时序、初始化与偏置状态，并建立同一输入的可重放对照；避免继续盲目调整场景或噪声。VIO源连续有效、运动精度和同机120 s融合通过后，才接仓库飞行会话。仓库悬停、导航与避障仍为 NOT_RUN；不能把本轮纯双目诊断通过当作 VIO 通过。
