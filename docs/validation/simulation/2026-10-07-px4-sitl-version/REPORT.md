# 本机 PX4 SITL 版本冻结

日期：2026-10-07。用户要求本机使用 PX4 1.16 系列最新版；采用该系列最新稳定发布 **v1.16.2**。

- 官方仓库：`https://github.com/PX4/PX4-Autopilot.git`
- 官方发布：[v1.16.2 Stable Release](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.16.2)
- 精确 commit：`54f0455ffcd755534539a7cf33a09a20bf71d29d`
- annotated tag object：`cec6f597235c27f8d42d900e5c5fc8e4d792dbd6`
- 计划源码位置：`.deps/PX4-Autopilot`；构建目标 `px4_sitl_default`，初始模型 `gz_x500`。

通过官方远端 `git ls-remote --tags ... 'refs/tags/v1.16*'` 查询该系列全部 tag，按正式版本 patch 号排序，排除 alpha/beta/rc；最大正式版本为 v1.16.2。官方发布页标记 Stable Release，tag 的 peeled commit 与发布页 commit 一致。保存 [远端 refs](remote-tags.txt) 和 [机器可读冻结记录](sitl.json)，主版本锁见 `simulation/px4/versions.lock.yaml`。

“最新版”按本次核验时点解析，后续构建始终用以上精确 commit；不在每次启动时自动跟随 release 分支或移动 tag。若后续出现新的 1.16.x，显式更新锁并重跑受影响回归。

本次完成版本选择与冻结，尚未准备 SITL 源码/子模块、编译或启动 x500；也未完成 Agent 和 px4_msgs 对应验证。这些待办仍列在锁文件 pending 中，不将版本冻结当作运行通过。

DM-FC01 厂商文件继续保留自己的构建身份 `v1.16.0-7-g78a512995e`。本机上游 v1.16.2 与板卡固件分别记录，后续硬件联调再比较实际接口和行为。Jetson 仍延期。
