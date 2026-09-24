# nav_guard

安全监视与急停发布包。

- 订阅: `/oakd/points`
- 发布: `/nav/emergency`

当前实现检查点云、TF 和里程计健康状态，发布应急状态；里程计保护阈值需按无人机运动范围标定。
