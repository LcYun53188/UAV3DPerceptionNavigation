# USB 台架与开发会话重启（2026-10-08）

用户条件：DM-FC01 仅 USB 连接，无外部供电，接收机未供电、未连接遥控器。

停止原受管 launch 82166 及独立 RViz launch 112662，核对无残留相关进程后，
重新启动 lab/mapping、domain 68、partition uav_ego_lab，view=both，launch 147565。
Gazebo 顶层启动器 147589 的子进程 147850 / 147851 分别为同一场景的服务端/GUI；
RViz 147605。起点初始化完成 Local launch area observed，地图保持在线更新。
本轮用户要求重启，因此原内存地图重新建立；磁盘地图包和日志保留。

## 实机只接收观测

设备 `/dev/serial/by-id/usb-DAMIAO_DAMIAO_DM-FC01_0-if00` → `/dev/ttyACM0`。
QGC 启动前被动采集 10.001 s，206421 字节，12 条 HEARTBEAT；原始关键状态和
消息计数见 usb-passive.json。PX4 source 1:1 最新心跳 armed=false；
EXTENDED_SYS_STATE.landed_state=1。电池电压 65535、剩余量 -1 为未知/无效值。
未收到 RC_CHANNELS、AUTOPILOT_VERSION 或 STATUSTEXT，不据此声称获得固件精确
版本、RC 硬件诊断或完整健康检查。system_status=0 也不作为就绪证明。

诊断采集无 MAVLink 发送；不向实机发送解锁、模式切换、轨迹、参数修改或电机控制。
随后 QGC 重新打开 PID 149409，用 fuser 确认持有 ttyACM0；QGC 通信独立于该被动
采集。X11 已确认 Gazebo、RViz、QGC 三类窗口存在。此时没有运行 PX4 SITL。

## 软件验证

新增 inspect_px4_usb.py 及伪终端测试，3 passed。伪终端实际注入 MAVLink 帧，检查
正确解码、无发送、termios 恢复，另验证串口读者冲突与静默流的 INCONCLUSIVE。
PX4 Python 环境未安装 pytest，使用工作区 pytest 与 PX4 环境的纯 Python MAVLink
模块完成测试，没有修改锁定运行依赖。最终实现还仅在成功取得独占后释放独占。
最终工具在 QGC 持有实机串口时实际拒绝读取，见 usb-busy.json。
仿真状态复核 ready=true、executor=HOLD、map.valid=true。
managed vendor/ego 补丁检查通过；未更改或提交 vendor 子模块。

这些是 USB 遥测和软件测试结果，不是解锁、遥控器链路、外部定位或实机飞行验证。
操作入口见 [台架说明](../../../PX4_USB_BENCH.md)。
