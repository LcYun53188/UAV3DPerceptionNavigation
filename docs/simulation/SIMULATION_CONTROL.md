# 仿真启动与控制脚本

[文档总目录](../README.md) · [启动与构建](LAUNCHERS.md) · [脚本目录](../../scripts/README.md)

## 1. 启动程序

只保留光流纯手动与完整项目两个程序：

```bash
./scripts/sim.sh run manual  # 无 VIO，只由遥控器手动飞行
./scripts/sim.sh run full    # VIO、建图、规划、行为树完整链路
```

二选一运行，默认使用实体 HID。使用模拟输入显式添加 `--input-source virtual`，程序自动打开或复用面板。校准完成并退出后会保存到 `.cache/simulation/qgc-calibration/`，下次自动复用；`--qgc-config <文件>` 优先于自动缓存。实体与模拟设备分开保存，未完成的校准不会覆盖旧校准。默认不安装依赖、不自动构建、不自动解锁或起飞。

完整项目的 SB／SC／SD 控件、任务暂停恢复与手动干预见[控件定义](../flight/VIRTUAL_POCKET_PANEL.md#最新控件定义)。纯手动程序禁用自动任务，且只允许已具备的光流／测距定位能力。QGC 必须校准、启用摇杆并在 Fly 页面发送操纵。

## 2. 在新终端查看运行

```bash
./scripts/sim.sh status
./scripts/sim.sh logs
./scripts/sim.sh logs --follow
```

状态记录位于 `.cache/simulation/operator/current.json`，包含本轮运行目录和通信身份。状态输出同时读取定位准入、遥控器状态、输入健康和已生成的最终结果。启动预热可能需要数分钟；进程存在并不表示定位 READY。`logs --follow` 的 Ctrl+C 只退出日志查看。

两个终端使用同一工作区即可，不需要重新激活虚拟环境、加载 ROS overlay 或手动 export ROS_DOMAIN_ID／GZ_PARTITION。

## 3. 保存地图与导出模型

仅完整项目有地图。先用遥控器降落并上锁，确认 `status` 中实时状态为 `landed=true`、`arming=1`，然后执行：

```bash
./scripts/sim.sh save .cache/maps/warehouse_run1
./scripts/sim.sh export-mesh .cache/maps/warehouse_run1.ply
```

输出路径不能已存在。地图包包含 `manifest.json` 和 `static_map.nvblx`，记录场景、地图身份、分辨率及文件摘要；PLY 来自本轮实际重建的网格。完整项目当前关闭颜色融合，导出的模型不承诺真实 RGB 颜色。

命令连接本轮项目的 `/uav/map/save` 或 `/uav/map/export_mesh`。服务端检查新鲜、唯一的 PX4 落地上锁状态及有效地图，在写入时暂停深度输入并恢复；不会取消遥控器任务、改变飞行模式或伪造 HOLD。定位或地图尚未准入、飞机在空中、状态陈旧、输出已存在时会拒绝。

当前接口不提供静态地图加载、离线摆放飞机扫描或旧真值执行器的 goal/explore 控制；完整项目的飞行任务由遥控器触发。

## 4. 加载地图导航

当前完整项目使用实时 VIO 与在线建图；本次保留地图保存和模型导出，跨会话静态地图加载尚未作为公共操作接入。保存的地图包不会在下一次启动时隐式加载。

## 5. 正常关闭

```bash
./scripts/sim.sh stop
```

命令向当前运行目录写入 `close-ui` 请求；运行器确认飞机落地且上锁后清理自身进程。飞行中请求关闭不会发送自动降落指令，请先用遥控器降落。预热期间的请求在运行器进入操作阶段后处理。会话时长到期也等待地面状态。

同一工作区只允许一个公共会话。关闭后查看 `status` 的 `active` 和运行目录中的 `result.json`／`cleanup_confirmed`；未完成清理或硬件操纵验收时不能报告飞行通过。

监督器已退出但仍有仿真残留时，先预览归属清单，再按需清理：

```bash
./scripts/sim.sh cleanup
./scripts/sim.sh cleanup --apply
```

工具结合工作区路径、会话分区和隔离配置筛选残留，保护仍在运行的监督器及其子进程；普通个人 QGC 不列入清理。默认只预览，`--apply` 分阶段发送 INT、TERM，必要时 KILL，并使用进程身份检查避免 PID 复用误杀。它不发送降落指令，正常会话仍应先落地上锁，再使用 `stop`。

清理脚本也可直接运行 `./scripts/simulation/cleanup.py`。INT／TERM 各阶段默认等待 5 秒，可用 `--timeout 2` 调整；未能完全清理时返回非零退出码。

## 6. 旧接口迁移

旧分层菜单、算法独立场景、自动起飞测试和专项传感器检查已移出公共入口。旧 `sim.sh algorithm ...` 与 `px4-*` 启动命令会提示迁移到 `run manual`／`run full`，不会偷偷启动其他程序。内部开发工具及历史回归仍在 [scripts/simulation/](../../scripts/simulation/README.md) 中维护。
