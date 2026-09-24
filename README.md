# 无人机导航工作区（uav_nav_ws）

ROS 2 无人机导航实验项目，包含 OAK-D / MID360 感知、VIO / LIO、三维 EKF、三维导航接入基础和 PX4 通信桥。
当前具备组件和接口验证基础，**尚未形成经过验证的三维自主飞行闭环**。

地面底盘串口桥、Nav2 导航入口、DWB / SE(2) 地面规划器及坡面通行过滤已移除。
完整保留已有 Isaac ROS 系列源码、工具和示例，撤销本地包屏蔽；详见 [Isaac 内容说明](docs/ISAAC_ROS_COMPONENTS.md)。

后续主线采用 **EGO + nvblox**，覆盖统一三维重建、轨迹规划与地图复用。模块接口、实施阶段和验收标准见 [开发手册](docs/EGO_NVBLOX_DEVELOPMENT_GUIDE.md)；该路线尚待实现。

## 构建

环境准备见 [安装指南](docs/INSTALLATION.md)。使用独立构建目录，避免旧地面包残留进入运行环境：

```bash
./scripts/build_uav_stack.sh
source install_uav/setup.bash
```

`with_venv.sh` 加载 `.venv`、系统 ROSji x 和 `install_uav`。旧 `build/`、`install/` 保留为历史产物，不再由脚本加载；请从未加载旧工作区的新终端运行。
可选 NVIDIA 定位/建图依赖使用 `./scripts/build_nvidia_3d_nav_deps.sh`。

## 验证入口

OAK-D + cuVSLAM + 三维 EKF + RViz（硬件定位验证）：

```bash
./scripts/with_venv.sh ros2 launch uav_bringup oakd_visual_slam_rviz.launch.py
```

EKF 模拟输入验证：

```bash
./scripts/with_venv.sh ros2 launch uav_bringup ekf_mock_validation.launch.py
```

无人机场景和传感器仿真：

```bash
./simulation/scripts/run_uav_obstacle_course.sh
```

场景使用简化速度控制模型，不等于 PX4 SITL，不会启动自动导航。

## 定位选项与三维导航计划

当前 `nav_stack.launch.py` 保留传感器、三维 EKF、安全监测和 PX4 接口，不再启动二维建图或规划节点。EGO + nvblox 统一三维地图与轨迹规划尚待接入。

- Isaac ROS Visual SLAM：使用 `oakd_visual_slam_rviz.launch.py` 独立验证。
- VINS-Fusion：保留源码、标定和启动入口，可通过 `nav_stack.launch.py enable_vins:=true odometry_source:=vio` 使用；`enable_vins:=false` 关闭该节点，不会自动切换至 cuVSLAM。
- FAST-LIO：保留 `odometry_source:=lio` 选项。

上述 nav_stack 参数是硬件联调配置，并非可直接飞行的已验收方案；其中 PX4 通信与状态机行为见项目评估。VINS 曾有不稳定跟踪记录，保留为可选实验定位，不声明已解决漂移。

## 主要代码

| 包 | 用途 |
| --- | --- |
| `uav_bringup` | 无人机启动、三维 EKF、硬件定位验证、Gazebo 场景 |
| `oakd_perception` / `imu_fusion` | OAK-D 图像、深度、点云和 IMU |
| `nav_mapping` | Livox 消息转换与三维点云合并 |
| `nav_safety` / `nav_guard` | 输入健康监测和里程计跳变保护 |
| `px4_comm_bridge` / `px4_msgs` | PX4 数据与控制接口 |
| `VINS-Fusion-ros2` | 可选 VINS 定位 |
| `nav_px4_bridge` | 已禁用构建的历史 PX4 兼容层 |

完整资料见 [文档索引](docs/INDEX.md)。
