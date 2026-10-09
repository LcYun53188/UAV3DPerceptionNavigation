# 完整任务优先：官方 SITL 基线回归

2026-10-09，按用户调整：先完成整套任务、导航和避障实现，再细化 VIO；参考相机固定下偏 5°，camera pose 与 optical TF 同步生成，18 项资产／场景／TF／UI 地面门控测试通过。历史角度仅留作复现，不继续调角度。5° VIO 源尚未验收，默认 VIO flight 明确拒绝，不挪用历史 15° 的源通过收据。

本轮两次 W0 已知区域 x500 / PX4 官方 SITL / BehaviorTree.CPP / QGC 回归使用 GNSS／惯性 EKF；**W0 x500 不包含本次 VIO 相机**，任务回归不是 5° 相机或 VIO 飞行验收。Gazebo 真值用于独立结果审计，不由项目发布到飞控定位输入。

| 运行 | 结果 |
| --- | --- |
| `c0061261-8e89-45c9-824d-c14025766279`，`px4-flight --bt --ui` | PRESTREAM 在解锁前以 `STALE_OR_INVALID_AIRCRAFT_STATE` 拒绝；始终最终 landed / disarmed。现场诊断同时记录了部分源时间领先 ROS clock 68～76 ms，不能仅凭一轮归因为 GUI 负载；未绕过时间保护。 |
| `74fa88e8-dbd4-4165-98c7-f833746627d3`，`px4-flight --bt` | PASS：2 m 起飞 → (3,2,2) 航点 → 30 s 悬停 → (-2,2,2) 航点 → 返航 → 3 s 悬停 → PX4 原生降落。 |

成功轮根 Action `SUCCEEDED / LANDED_AND_DISARMED / cleanup_confirmed=true`；BT 根派发一次，0～6 步各接受／完成一次，树成功退出。真实最大位移 4.1674 m；两段悬停真值覆盖 32.014 / 5.029 s，最大漂移 7.82 / 8.28 cm；两个导航航点误差 3.89 / 3.28 cm，返航误差 3.84 cm。真实落地并锁定，受管运行已结束。

这只确认完整基础控制任务仍可执行，不代表感知避障、地图导航或 VIO 已通过。EGO 当前是影子规划；下一实现主线为同机深度地图 → EGO 曲线授权／采样 → PX4 唯一网关跟踪 → 暂停／取消／重规划／返航及故障收尾。功能开发允许使用官方 EKF，不等待 VIO 稳定性；未知空间、碰撞、地图会话、定位重置、新鲜度及唯一控制写者门限继续执行，最终验收分定位来源记录。

原始失败与成功收据、日志、真值按需 gzip 保留字节。校验及单用例独立检查（不宣称完整四用例 BT suite 重验）：

```bash
python3 docs/validation/simulation/2026-10-09-task-first-baseline/check_archive.py
```
