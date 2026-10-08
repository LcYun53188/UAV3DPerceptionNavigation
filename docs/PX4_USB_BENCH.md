# DM-FC01 USB 台架检查

USB 台架与 PX4 SITL 分开：当前实机只有 USB 供电、未连接遥控器。此阶段检查连接和
主动遥测，不解锁、不发送 Offboard/轨迹/电机控制，不修改参数以绕过 RC 或电池检查。
算法 Gazebo 的 `/cmd_vel` 属于仿真；不要同时启动针对实机的飞控控制桥。

## 被动诊断入口

先关闭 QGC 及其他串口读取程序，然后执行（依赖现有 PX4 Python 环境）：

```bash
.deps/px4-venv/bin/python scripts/inspect_px4_usb.py \
  --device /dev/serial/by-id/usb-DAMIAO_DAMIAO_DM-FC01_0-if00 \
  --duration 10 --output .cache/hardware/usb-passive.json
```

该工具以 O_RDONLY 打开串口，仅解析 MAVLink，不发送心跳、参数/版本请求或命令。
发现已有串口读者时拒绝运行；采集期间使用 TIOCEXCL 禁止新的普通打开，结束恢复
termios 设置并释放独占。它不拉动 DTR/RTS，不刷新接收队列。工具不能约束其他
程序或具有特权的进程；使用前仍须关闭其他串口程序。

结果 OBSERVED 表示收到 PX4 心跳，不代表自检、定位或飞行就绪。无心跳为
INCONCLUSIVE，串口冲突/读取错误为 ERROR，后二者退出码为 2。输出保存每类消息
计数及最新的关键状态；收不到版本/RC 消息不能推导固件版本或接收机硬件故障。
采集后可以重新打开 QGC 查看状态；QGC 有自己的 MAVLink 通信，不能称为被动采集。

## 本次记录与验证

2026-10-08 记录见 [USB 台架报告](validation/hardware/2026-10-08-usb-bench/REPORT.md)。
伪终端测试覆盖真实 MAVLink 编解码、无发送、串口设置恢复、已有读者拒绝与无遥测：

```bash
PYTHONPATH="$PWD/.deps/px4-venv/lib/python3.11/site-packages" \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv/bin/python -m pytest -q scripts/test_inspect_px4_usb.py
```

下一阶段仍在本机推进 PX4 仿真的坐标/时间、感知导航集成；实机外部供电、真实 RC
链路、故障接管及外部定位另行验证，当前 USB 结果不计入飞行验收。
