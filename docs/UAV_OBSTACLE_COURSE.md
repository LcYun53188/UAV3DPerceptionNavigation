# UAV 障碍穿越场地

场景：`src/uav_bringup/gazebo/worlds/uav_obstacle_course.sdf`。
保留约 20 m × 20 m 围合区域、立柱、三组门框、堆物平台、绿色起点与红色终点。

```bash
./simulation/scripts/run_uav_obstacle_course.sh
# 可选 MID360 传感器桥：
./simulation/scripts/run_uav_obstacle_course.sh launch_mid360:=true
```

起点 `(0, -7, 0.11)`，初始 yaw 为 `1.5708 rad`；终点地面标记为 `(0, 7)`。
门框沿 Y=-3、0.5、4 分布。飞行目标高度尚未接入规划器。

启动 Gazebo 和传感器桥；自动目标和地面导航已移除。
当前四旋翼为简化速度控制模型，场景可用于传感器与空间布局检查，不能据此宣称完成自主穿越或 PX4 飞行验证。
