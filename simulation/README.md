# 无人机场景仿真

```bash
./simulation/scripts/run_uav_obstacle_course.sh
# 默认演示场地：
./simulation/scripts/run_gazebo_harmonic_nav.sh
```

需要构建 `uav_bringup` 并安装 Gazebo Harmonic / ros_gz。
入口只启动 Gazebo、传感器桥、静态 TF 和里程计速度估计器。
四旋翼使用简化速度控制模型，不含 PX4 飞控或旋翼动力学闭环。

场地说明见 [无人机障碍场地](../docs/UAV_OBSTACLE_COURSE.md)。

Isaac Sim 独立 UI 入口：`./simulation/scripts/run_isaac_sim_45_ui.sh`，需已有对应安装。参见 [Isaac 内容说明](../docs/ISAAC_ROS_COMPONENTS.md)。
