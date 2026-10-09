# 同机固定 5° 深度建图验证（2026-10-09）

已实现并在官方 PX4 SITL 中验证：同一架 x500 的固定下偏 5° RGBD 深度 →
PX4 GNSS／惯性 EKF 的历史位姿 TF → nvblox → 有效 ESDF。
这是未解锁建图验证，不是 VIO 融合、规划避障或曲线跟踪飞行验收。

## 实现

入口 `./scripts/sim.sh px4-depth-map --duration 40`；仅本次域 78、GZ partition、
实例 7、XRCE 8898 下启动和清理受管进程。QGC 禁止串口自动连接。
生成模型 `x500_depth_ref` 合并上游 x500，保留原动力学，显式使用 4001 机型启动。
相机 640×400／15 Hz／水平 FOV 1.21 rad，光学帧与下偏 5° SDF 一致。
只渲染 RGBD，不运行 cuVSLAM；理想深度不代表 OAK-D Pro W 的标定或双目匹配质量。

只读 TF observer 使用 PX4 原始采样时间戳和 NED／FRD → ENU／FLU 转换，
不订阅 Gazebo 真值、不发布 FMU 输入。map 定义为本次 EKF odom 坐标系。
初始化重置计数须稳定 5 秒；开始转发后重置或无效位姿锁存停止，
解锁也锁存停止，状态过期不转发。地图会话等待每帧对应历史 TF，
只把真实深度积分产生的 ESDF observed 标志作为已观测空间，不预填未知区域。

## 实测

| 轮次 | 原始运行 ID | 结果 |
| --- | --- | --- |
| 首次建图（修复前） | e363436f-0fe5-4d14-8cbc-b66adaea3f61 | 失败：启动 EKF 重置后 TF 停止，ESDF 源过期；全程未解锁 |
| 40 秒建图 | b19a98dc-b8e4-4197-a1fa-8af49ff1a387 | DDS／时钟／QGC／深度／ESDF 通过 |
| 最终代码 30 秒复验 | 2aed4e7f-601f-44f1-b51c-6ee641258ff5 | DDS／时钟／QGC／深度／ESDF 通过，TF observer 正常退出 |

40 秒轮最终地图版本 69，网格 101×101×41（约 0.1 m），共 418,241 体素；
14,418 已观测，其中 10,480 距离值为正，地图源年龄 0.104 s。
深度 265 帧有效、CameraInfo 511 条有效，无错误、无时间回退，发布者各 1。
所有运行原始 manifest、末次深度／ESDF 数据、日志和生成资产见各归档子目录；
manifest 保存运行时源代码及资产哈希。早期失败和通过轮使用各自原始源码哈希，
不得用最终代码哈希覆盖它们。部分旧轮在 SIGINT 清理时存在 ROS shutdown 异常；
最终轮已修复 TF observer 重复 shutdown，退出码以各 `cleanup.json` 为准。

53 项原有和新增测试通过后，补充真实 ROS MapSnapshot 的 numpy uint32 序列化回归，
最终 **54 项通过**。首次测试收集遗漏 nvblox_msgs 安装环境，改用基础工作区＋隔离 PX4／mission overlay 后通过。
另修复实测暴露的自定义模型启动 ID、`use_sim_time` 参数重复声明和证据 JSON 序列化问题。
这些启动期失败不计为建图成功。

```bash
./scripts/with_venv.sh bash -e -c '
  source .deps/px4-msgs-install/setup.bash
  source .deps/mission-install/local_setup.bash
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q \
    scripts/test_px4_mapping_tf.py scripts/test_px4_depth_assets.py \
    scripts/test_px4_depth_audit.py scripts/test_vio_sensor_assets.py \
    src/uav_nav_sim/test/test_depth_sync.py
'
python3 docs/validation/simulation/2026-10-09-px4-depth-mapping/check_archive.py
```

## 下一阶段与验收边界

当前仅约 3.4% 网格已观测；距离值为正不代表整个机体／制动包络已满足规划门限。
不能把本轮有效 ESDF 当作完整导航任务通过，也不能迁移到 5° VIO 悬停验收。
下一步把这套受管传感器／地图源与唯一 FlightServer 的定位 session、重置计数、
map epoch 及 alignment 绑定，接入 PlanningContext 和 EGO。
起飞前地面相机对发射／飞行包络的覆盖仍不足，须设计显式已知区域起飞后的有限观测阶段，
再进行完整曲线导航、悬停、返航、降落与障碍／地图过期失效验证，不能放宽未知空间门限。
