# Gazebo Harmonic 无人机场景

```bash
./scripts/build_uav_stack.sh
./simulation/scripts/run_gazebo_harmonic_nav.sh
./simulation/scripts/run_uav_obstacle_course.sh
```

同一时间运行一个场景。可通过 `launch_mid360:=true` 打开 MID360 桥接，
通过 `world:=/absolute/path/world.sdf` 指定场景。

保留 RGB-D 图像、相机内参、IMU、点云、TF 和 Gazebo 里程计桥接。
`gazebo_odometry_velocity.py` 为模拟里程计补充速度。
`/cmd_vel` 是简化模型的速度控制接口，与 PX4 `/nav/cmd_vel` 链路独立。
此入口不提供自主导航、PX4 SITL 或真实飞行动力学验证。

场地布局见 [障碍场地](UAV_OBSTACLE_COURSE.md)。
