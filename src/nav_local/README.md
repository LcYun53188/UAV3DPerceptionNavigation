# nav_local

本包已降级为兼容层，原先的本地导航节点已拆分到独立功能包：

- `nav_mapping`：点云处理与局部栅格生成
- `nav_planning`：局部规划与速度决策
- `px4_comm_bridge`：PX4 数据/控制桥接
- `nav_safety`：安全检测与急停
- `uav_bringup`：统一启动入口

兼容启动方式仍保留：

```bash
colcon build --packages-select nav_local nav_mapping nav_planning px4_comm_bridge nav_safety uav_bringup
ros2 launch nav_local nav_local.launch.py
```
