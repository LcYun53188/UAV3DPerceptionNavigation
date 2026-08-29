# nav_safety

安全监视与急停发布包。

- 订阅: `/oakd/points`
- 发布: `/nav/emergency`

当前实现是原型阈值检测，后续会扩展为超时、地图失效、PX4 降级策略。