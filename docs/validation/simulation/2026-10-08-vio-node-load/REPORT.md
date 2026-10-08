# 第二阶段前置：本机 cuVSLAM 节点构建与加载（2026-10-08）

BT 四项实飞通过后，单独选择构建 `isaac_ros_visual_slam` 成功，首次约 39.8 s，
增量构建约 0.41 s。没有编译可选 image_proc 流程；此前缺少 CV-CUDA 头文件的
图像预处理包问题仍未修复，workspace setup 仍会打印其缺失提示。

实际启动 cuVSLAM 可执行程序，run `14da3616-b5cf-4b0b-b970-4af5e8b8eacf`：

- cuVSLAM SDK 报告 `15.0.0+74f0e317-modified`，GPU CUDA warm-up 完成。
- 节点处于 VIO（IMU fusion）配置，关闭 SLAM 和 odom→base TF 输出。
- DDS 图确认两个 image、两个 CameraInfo 和 IMU 输入订阅存在。
- 隔离 ROS domain 94；没有 `/fmu/in/` 发布者，不启动相机驱动、PX4 或飞行命令。
- 节点正常存活，验证后发送 SIGINT，退出 0、cleanup_confirmed=true。
- `result.json` 保存实际二进制、组件库、SDK、工具、版本锁 hash 与源码 commit。

**这只证明节点/库/CUDA 和输入接口能加载。没有图像/IMU输入，没有初始化并验证
真实跟踪结果，也没有产生 VIO 融合或悬停证据。** 不算第二阶段完成。

复现（要求此前 NITROS 等依赖已安装）：

```bash
./scripts/build_vio_node.sh
./scripts/with_venv.sh env ROS_DOMAIN_ID=94 ROS_LOCALHOST_ONLY=1 python scripts/run_vio_node_smoke.py
```

下一工作项为固定 Gazebo 双目/IMU模型、标定、安装 TF、时间与有纹理场景，
用实际 VIO 输出验证跟踪、坐标、协方差和 reset，再接独立 PX4 VIO 配置与 BT 悬停。
合成静止 external vision 不得替代这一输入链路。

`node.log.gz` 和 `build-*.log.gz` 保留白名单加载/构建日志；`SHA256SUMS` 校验归档。
