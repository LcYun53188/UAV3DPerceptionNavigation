# UAV 障碍穿越场地

[文档总目录](../README.md) · [脚本目录](../../scripts/README.md)

当前场景为 `src/uav_bringup/gazebo/worlds/uav_obstacle_course.sdf`，已更新为 **34×34×6 米**四区赛道：绕柱、穿矩形框、穿圆环、绕树，中央保留起降／观察区。几何、五套任务、地图准备和验收规则统一见[四区赛道说明](OBSTACLE_COURSE.md)。旧 20×20 米门框场地的坐标与布局不再适用。

## 完整项目

```bash
# 预览命令；按四区赛道说明完成构建和地图准备后再运行
./scripts/sim.sh run full --course all --dry-run
./scripts/sim.sh run full --course all
```

此入口使用 PX4 SITL、实际 VIO、深度地图、EGO 与 BT，仍由遥控器准备和启动任务。不能把静态净空回归或场景加载当成全程穿越验收。

## 几何预览

```bash
./simulation/scripts/run_uav_obstacle_course.sh
# 可选 MID360 传感器桥：
./simulation/scripts/run_uav_obstacle_course.sh launch_mid360:=true
```

上述兼容脚本只加载原生 SDF 和可选传感器桥，机型为简化速度控制模型，不启动完整 PX4／VIO／EGO 链路。中央起点为 `(0,0)`；任务目标使用相对于本轮起点的 ENU 偏移。预览用于检查障碍碰撞和布局，不能据此宣称完成自主穿越或 PX4 飞行验证。
