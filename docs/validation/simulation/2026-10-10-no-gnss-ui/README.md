# 无 GNSS、固定 5° 的四窗口 VIO 诊断

2026-10-10 用户指定禁用 GNSS 并显示 UI。本轮使用双目／IMU参考模型，实际 cuVSLAM → PX4 EV／EKF；未使用 Pro W 实机标定，未派发飞行任务。

归档 run_id=f5bda84e-d16c-47a5-be55-087dcb0384c2。120 s 带 UI 未解锁诊断的 diagnostic_checks_passed=true，real_pose_fusion.passed=true；实际融合与停源失效、唯一 EV 写入者、无飞行控制输出、始终着地未解锁等检查通过。由于明确 diagnostic_ui=true，qualification=false、整体 passed=false；不视为无界面飞行源验收或 VIO 飞行成功。较早 d802bf21 轮诊断失败，原始缓存保留。

参数 EKF2_GPS_CTRL=0、EKF2_EV_CTRL=11。最后保留窗口阶段诊断按设计停止归一化源／EV 输出，SDK 与传感器画面继续显示；不回退 GNSS。最终会话 8f3a7fac-0832-4ade-b78d-f5f3d49238d8 正运行相同固定角度配置，额外加入仅 RViz 使用的 odom→vio_display_origin 静态别名，不定义机体位置或地图对齐。保存的只读回查确认 GNSS 位置／速度／高度／航向融合均 false；该文件采集于审计停止 EV 后，EV 标志也为 false。启动过程中另曾观测实际 EV 三类融合，但不是窗口保留阶段的持续融合声明。

修复 RViz 实机图像话题残留，新增 px4_vio_sim.rviz，使用 /vio/left/image 和 odom 视角；双目监视器显示简洁任务／诊断摘要。UI 共 Gazebo、QGC、RViz 和双目监视器四窗口，在主屏排列。UI保留期最多一小时；在当前缓存目录创建 close-ui 文件即可有序关闭本次所属进程。

相关现有 UI 生命周期、运行 TF 与传感器资产测试 12 项通过；脚本语法及 RViz 配置加载通过。历史 GNSS 的完整 EGO 任务保留为功能证据，后续按无 GNSS 的 VIO 准入继续推进。

最终 8f3a7fac 轮已结束诊断并保留四窗口，最新 diagnostic_checks_passed=false、real_pose_fusion.passed=false，原始 ui-session 快照见 live-session。其待修问题以快照失败检查为准；前一轮通过不证明重复稳定性。未派发飞行任务，窗口保留阶段停止归一化源及 EV，实际 SDK／双目画面继续运行，飞机着地未解锁。RViz 已实际加载新配置，图像可见、Global Status OK；此状态仅说明显示配置，不能作为 VIO 飞行健康证明。
