# Gazebo 算法验证记录（2026-09-26）

范围：默认 `uav_ego_lab`、Gazebo 真值定位、无重力 VelocityControl 模型、机体查询半径 0.3 m。nvblox 从 Gazebo 渲染深度建立三维地图，官方 EGO 的 A* 与 rebound B 样条优化使用地图快照。独立验收根据 SDF 的柱体、围墙、地面几何计算机体净间距。

版本由 [algorithm_versions.json](../../src/uav_bringup/config/algorithm_versions.json) 固定，EGO 源码修改由 [vendor patch](../../patches/vendor/ego_planner.patch) 管理。构建入口为 `scripts/build_algorithm_sim.sh`，本轮开发已完成 28 个算法依赖包和 bringup 构建。

地图包：`.cache/maps/uav_ego_lab_verified`，不进入 Git。地图 ID 为 `467ee5e5-2849-4616-921e-b70b291642d0`；数据 SHA-256 为 `a3acc94f69f98746d2dbd41fdb523429b0520a03e57d5362c9e25bded05b8844`。该地图在前一轮在线扫描中创建，本轮机器重启后由新进程加载。原始启动日志位于忽略目录 `.cache/validation/`，本目录保存小型结果 JSON。

## 已保存结果

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 新进程加载后正向绕柱 | 通过；净间距 0.225224 m，终点误差 0.001438 m | [正向 JSON](gazebo_loaded_forward.json) |
| HOLD 重复加载后反向绕柱 | 通过；净间距 0.188710 m，终点误差 0.001395 m | [返航 JSON](gazebo_reloaded_return.json) |
| 地图会话切换 | 两次轨迹 epoch 从 `1790395027358041676` 递增至 `1790395027358041677` | 上述轨迹元数据 |
| 障碍内与地图外目标 | 均拒绝；0 条轨迹、无非零运动命令 | [负向目标 JSON](gazebo_negative_goals.json) |
| 执行中加载地图 | 拒绝 | [故障 JSON](gazebo_map_loss.json) |
| 地图进程暂停 | 数据过期停止、HOLD、零速度命令；测试后恢复进程 | [故障 JSON](gazebo_map_loss.json) |
| 单元回归 | 44 项通过：20 项算法/会话、24 项飞控桥历史回归 | `pytest` 本机执行 |
| 补丁重复应用、Python/XML/SDF/Shell 语法及 Git 空白检查 | 通过 | 本机执行 |

两次曲线时长均为 19.392 s，各取得 995 个真值位置样本。导数控制点计算的速度、加速度、jerk 上界均低于 0.5 m/s、1 m/s²、2 m/s³，详见 JSON 的 `bounds`。机体净间距已扣除 0.3 m 查询半径。

## 发现与修复

首次跨进程加载暴露了会话不一致：加载失效通知使用旧地图 ID 与新 epoch，成功快照使用加载地图 ID 与同一 epoch，执行器因此拒绝快照，未执行运动。已将地图 ID 更新移到新 epoch 的失效通知之前；执行状态检查通过后才变更 ID。新增两个回归测试，覆盖新地图身份发布与执行期间拒绝加载时状态不变。最终结果由修复后的完整启动重新生成。

地图命令行等待执行器状态可用，减少启动发现期间的误操作；服务端仍要求收到新鲜 HOLD 状态。执行器拒绝日志增加地图/里程计年龄，便于判断未就绪原因。

停止完整 launch 后，Gazebo、nvblox、地图管理、规划与执行进程均已退出，无本轮残留。退出时发现既有里程计辅助节点重复调用 `rclpy.shutdown()`；已增加上下文检查，独立 SIGINT 回归以退出码 0 通过。

## 复现

按 [运行手册](../EGO_NVBLOX_GAZEBO.md) 创建地图并启动 localization 模式。所有终端的 ROS_DOMAIN_ID 和 GZ_PARTITION 应一致。本轮使用域 69、分区 `uav_ego_reloaded_20260926`。依次执行加载、正向目标、HOLD 下再次加载、返航目标和故障检查；不要并发发布目标。

```bash
./scripts/with_venv.sh ros2 run uav_nav_sim map_bundle load .cache/maps/uav_ego_lab_verified
./scripts/with_venv.sh python scripts/check_gazebo_goal.py --goal 3 0 1.2 --output docs/validation/gazebo_loaded_forward.json
./scripts/with_venv.sh ros2 run uav_nav_sim map_bundle load .cache/maps/uav_ego_lab_verified
./scripts/with_venv.sh python scripts/check_gazebo_goal.py --goal -3 0 1.2 --output docs/validation/gazebo_reloaded_return.json
./scripts/with_venv.sh python scripts/check_gazebo_failures.py --output docs/validation/gazebo_negative_goals.json
# 从该次 launch 日志取得 map_session PID，再运行；仅针对本实验进程。
./scripts/with_venv.sh python scripts/check_gazebo_map_loss.py --map-session-pid <PID> --map-directory .cache/maps/uav_ego_lab_verified --output docs/validation/gazebo_map_loss.json
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./scripts/with_venv.sh python -m pytest -q src/uav_nav_sim/test src/px4_comm_bridge/test
```

## 限制

这是固定静态场景的基线回归，未达到开发手册建议的每类场景 20 次验收。没有运行真实 VIO、旋翼动力学、PX4 SITL、真实重定位或实机。离线扫描通过 `set_pose` 获取多视角，不是自主探索。样条动态约束是参考曲线界，零速度停止由简化仿真模型实现；不能据此证明真实制动能力。复杂三维场景、运动中连续轨迹接管和飞控仍为后续工作。
