# nav_mapping

保留三维点云预处理功能：

- `livox_custom_to_pointcloud2`：将 Livox 自定义消息转换成 PointCloud2。
- `pointcloud_combiner`：按 TF 变换并合并 OAK-D / MID360 点云。

本包不生成二维栅格，也不实现三维地图积分。统一三维重建与地图存储使用 nvblox，接入计划见 [开发手册](../../docs/EGO_NVBLOX_DEVELOPMENT_GUIDE.md)。
