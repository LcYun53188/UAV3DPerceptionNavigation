# Isaac 系列保留范围

完整保留工作区已有的 Isaac ROS 上游仓库：

| 仓库 | 内容 |
| --- | --- |
| `src/isaac_ros_visual_slam` | cuVSLAM 与接口 |
| `src/isaac_ros_nvblox` | 三维重建、地图存取、工具、测试和示例，包括上游附带的集成插件 |
| `src/isaac_ros_image_pipeline` | 图像、深度、立体处理及 GXF 扩展 |
| `src/isaac_ros_nitros` | NITROS、GXF、消息类型与示例 |
| `src/isaac_ros_common` | 公共工具和接口 |

删除二维栅格仅针对本项目自有实现，不裁剪 Isaac 上游源码。本地添加的包排除标记和补丁已移除，上游原有目录忽略规则保留。
“完整保留”指已有源码、示例及依赖资产保留，不表示每个示例依赖均已安装或全部包已构建通过。`build_nvidia_3d_nav_deps.sh` 仍按主线依赖选择构建，未恢复项目地面导航入口。

上游 nvblox 的 Nav2 插件和示例可能需要单独安装 Nav2 等依赖；这是可选上游集成，不是本项目的无人机规划链路。尚未存在于工作区的其他 Isaac 产品不在本次安装范围。

Isaac Sim 独立 UI 启动脚本已保留：`simulation/scripts/run_isaac_sim_45_ui.sh`。需要已有对应安装；该脚本只启动 UI，不提供已删除的地面二维导航流程，也不代表已有无人机 PX4 仿真闭环。

定位可以使用 Isaac ROS Visual SLAM，也保留 VINS-Fusion 实验选项。两者的里程计需要经统一坐标、时间和有效性检查后才能成为 EGO + nvblox 的输入。
