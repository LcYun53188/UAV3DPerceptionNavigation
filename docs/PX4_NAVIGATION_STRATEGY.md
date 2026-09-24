# PX4 三维导航策略

采用 EGO + nvblox：三维重建与地图复用由 nvblox 提供，轨迹规划由 EGO 提供，定时执行通过 PX4 接口实现。该链路尚待开发。

定位首选 cuVSLAM，同时保留 VINS-Fusion 和 FAST-LIO 可选路径；不同后端必须统一坐标、时间、协方差和 reset 语义，不允许无检查地热切换。

项目原二维栅格、平面投影融合、APF 和兼容启动层已删除。完整保留 Isaac 系列上游内容，按任务选择构建与启动。

实现步骤与验收标准见 [EGO + nvblox 开发手册](EGO_NVBLOX_DEVELOPMENT_GUIDE.md)，现有问题见 [项目评估](UAV_PROJECT_ASSESSMENT.md)。
