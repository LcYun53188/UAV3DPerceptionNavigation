# 无人机导航项目评估

本评估基于当前源码和本次本地检查，不代表实机飞行验收。
项目处于算法仿真集成阶段：已实现 Gazebo 真值定位、nvblox 三维地图、EGO 轨迹规划和仿真执行闭环；真实定位、飞控动力学与实机导航尚未验收。

## 主要发现

| 优先级 | 代码依据 | 影响与待办 |
| --- | --- | --- |
| 已修复，待 SITL | 自动解锁统一为 `auto_arm=false`，移除 `sm_auto_arm` 和回调直接解锁 | 已补状态机与控制桥回归测试；完整飞行验收仍待完成 |
| 高 | `uav_bringup/launch/nav_stack.launch.py` 默认 VINS，cuVSLAM 位于独立 `oakd_vio.launch.py` | README 原先把两套定位链路混为一谈；应确定唯一主定位源并验证时间戳、TF、协方差及 PX4 外部定位输入 |
| 中 | `uav_quad_mid360/model.sdf` 使用 VelocityControl | 无旋翼动力学和 PX4 SITL，现有场景可验证感知接口和算法避障，不能验证飞行稳定性 |
| 中 | `mock_px4_validation.launch.py` 包含完整硬件导航入口 | 并非完全无硬件的测试；优先使用独立 EKF mock，后续隔离传感器和串口开关 |

保留的 `nav_guard` 阈值源于低速验证配置，需要按无人机运动包线重新标定；其 hold/reseed 机制不能恢复失效的定位。

## 本次清理

移除底盘串口桥、omni bringup、Nav2 子模块及补丁、地面 SE(2)/DWB 规划器、二维 EKF / Nav2 参数、坡面通行过滤与断崖检测、地面导航仿真和诊断脚本。
将共享 OAK-D / cuVSLAM / EKF 逻辑提取为独立 `oakd_vio.launch.py`，保留已有外参和硬件调试参数。
保留无人机模型和场地、传感器、PX4、三维 EKF、可选 VINS 及完整 Isaac 系列源码。项目自有二维栅格与 APF 已移除；后续已接入 EGO 三维规划，见下方算法更新。
Isaac 上游插件、示例和测试完整保留；本地构建排除标记已撤销，主线按依赖选择构建。

新构建与运行统一使用 `build_uav/`、`install_uav/`，旧输出保留但不由脚本加载。
本次删除前的受影响工作文件（包括已有未提交修改）备份于：
`/tmp/uav_nav_before_cleanup_20260923_175840.tar.gz`。该路径是临时备份，需长期保存时请转存。

## 后续优先顺序（2026-09-25 用户调整）

1. 先完成 Gazebo 真值定位、nvblox 三维建图和 EGO 避障的完整算法链路。
2. 完成离线地图创建、保存、重启加载与静态场景复用，保留独立真实几何验收。
3. 扩展复杂三维场景、失败路径、真实感知定位和算法性能验证。
4. 最后固定 PX4 版本并接入飞控/SITL，验证坐标、时序、跟踪、制动和人工接管，再做实机。

新增入口、补丁式依赖管理与运行限制见 [Gazebo 算法链路](EGO_NVBLOX_GAZEBO.md)。
先前飞控桥修复保留，但不再将剩余飞控问题作为算法开发前置。

## 历史验证结果（2026-09-23，以下已删除包结果不代表当前版本）

- 6 个核心包构建通过：`px4_msgs`、`nav_guard`、`nav_mapping`、`nav_planning`、`nav_safety`、`px4_comm_bridge`。
- `uav_bringup` 独立 CMake 配置和资源安装通过；7 个 launch 文件均可生成 LaunchDescription，共享 VIO 参数声明完整。
- Python AST、Shell 语法、XML/SDF 解析及 Git 差异空白检查通过；colcon 包发现不再包含地面底盘、Nav2 或 nvblox 地面示例入口。
- 现有单元测试：16 项通过，1 项失败。失败为未修改的 `test_2_arm_to_offboard`：首次 update 未发送 ARM 命令，与测试期望不一致，需单独修正状态机时序/测试约定。
- 当前 pytest 与 ROS `launch_testing` 自动加载插件存在 API 不兼容；单元测试通过 `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` 运行。未执行 launch_testing 集成测试。
- 未做完整 GPU/传感器栈重编译、实机传感器验证、Gazebo GUI 或 PX4 SITL/飞行验证。


## 2026-09-24 清理更新

移除项目自有二维栅格、融合、APF、nav_local 兼容层及旧集成测试。nav_mapping 仅保留点云转换/合并；VINS 启动开关和标定完整保留。撤销 13 个本地 Isaac 排除标记，保留嵌套上游原有忽略规则；Isaac 源码、插件和示例不裁剪。


## 2026-09-26 算法仿真更新

已完成 Gazebo 深度 → nvblox 三维地图 → EGO 轨迹 → 仿真执行，以及地图包创建、重启加载、重复加载后的绕柱回归。新依赖固定 commit，差异保存在 patches，源码位于忽略目录。当前单元回归 44 项通过；实测和边界见 [验证报告](validation/EGO_NVBLOX_GAZEBO_20260926.md)。飞控/SITL 与实机继续留在后续阶段。
