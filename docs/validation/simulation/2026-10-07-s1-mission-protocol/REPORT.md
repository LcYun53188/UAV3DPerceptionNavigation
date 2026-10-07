# S1 任务身份、暂停协议与 FlightSession mock

日期：2026-10-07。本机 Jazzy；分支 `feat/local-simulation-s0-s1`。
接口提交 `a590a11`，模型/服务端提交 `76da450`。本批仅为协议验证，未连接 PX4、
Gazebo、规划器或执行器。所有 ROS 状态/结果明确标记 mock，不作为实际飞行证据。

## 交付

- ExecuteMission / NavigateToPose3D Action、TaskStatus、ControlSession / ControlStatus、PauseMission / ResumeMission 服务。
- 串行所有者协议模型：根/子 UUID、协调器实例、控制世代隔离；暂停/恢复、取消优先、完成竞态、原生降落取消守卫。
- FlightSession mock：有界制动/停稳/新世代保持 ACK 后才结束根任务；根任务结束后保持可继续续租，但固定最终期限不滑动。
- 独立 `/uav/mock/*` ROS Action/服务端及普通客户端冒烟运行器。

协议字段、单位、候选期限、mock 合成事件与未实现能力见
[协议说明](../../../../src/uav_mission/PROTOCOL.md)。本批没有修改既有算法轨迹消息，
接口与 mock 在 `.deps/mission-install` 独立构建。

## 验证

独立 colcon 构建：`uav_nav_interfaces` 和 `uav_mission` 两包成功。
聚合器 36 项、协议模型 32 项、已有仿真工具 46 项：**114 passed**。
Python 编译、Git 空白检查、受管 vendor patch 校验通过。

最终 ROS 会话 `120a85e2-284a-4cb3-a98a-81097ca6a71c`，ROS_DOMAIN_ID=79：
**24 项检查全部通过**，收到 294 条 TaskStatus，服务端退出码 0，无本次残留。
见 [观测与源码 hash](observation.json)、[去重状态转换](transitions.json)、
[服务端日志](server.log)。原始完整事件保留于对应 `.cache/simulation/mission-protocol/`
会话目录；归档仅对连续相同事件去重，未改变字段。

关键检查：BUSY 拒绝并行根任务；MOCK 拒绝真实后端；暂停接受先于停稳确认；
暂停超过合成运动预算时父 Action 仍活动；重复请求返回原决策；旧实例拒绝；
恢复更新子 UUID/控制世代；取消需确认保持交接；保持会话跨根结果存活；新授权
可开始新根任务，旧根暂停/迟到取消不能作用于新根；交接失败返回
ABORTED/CANCEL_TIMEOUT 且 cleanup_confirmed=false；普通 Navigate 客户端收到
明确标记的合成结果。模型回归另覆盖固定保持期限、暂停/总预算、进展租约、
原生降落观察失联/超时不能转回保持、人工/failsafe 接管及终态唯一。

首次 ROS 冒烟 `07056fdf-4f65-4120-933b-20c1fa11f977` 因节点属性 `handle`
与 rclpy 保留属性冲突而启动失败，日志保留在缓存；已改为 active_goal。后续
`6be5eb8e-c82c-4be5-bcec-8a60e23b524d` 和 `e603726e-7fcb-40d1-a4f3-da56087105be`
亦通过；加入最终降落守卫及真实后端拒绝后，重新运行得到本目录最终证据。

复现：按包 README 独立构建，再执行：

```bash
./scripts/with_px4_sim.sh bash -e -c \
  'source .deps/mission-install/local_setup.bash; python scripts/run_mission_protocol_smoke.py'
PYTHONPATH=src/uav_mission PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 SKIP_WS_SETUP=true \
  ./scripts/with_venv.sh python -m pytest -q src/uav_mission/test \
  scripts/test_sim_control.py scripts/test_sim_validation.py scripts/test_px4_sim_tools.py
```

## 剩余边界与下一步

mock 的停稳/网关 ACK 是合成事件，未证明实际制动、参考连续性或网关唯一输出。
Navigate fixture 是独立根任务，不连接父子 ROS Action、实际目标容差/速度检查、
旧规划 token/轨迹撤销或地图/TF。S2 需要在现有执行器所有者中接入真实 Navigate
Action，然后实现 MissionServer/两航点 BT 和航点检查点，不能直接把 mock 成功
接到实际任务成功。

AircraftState 的 Agent/电池/导航健康提供者，以及定位 reset 后显式故障确认与重新
授权仍待补齐；AcquireControl/ResetFault 未实现。S0 cuVSLAM/nvblox 固定样例仍待
CV-CUDA 依赖，S3/S4 坐标、时钟、W0、控制与飞行动力学门槛未完成。
因此本批完成的是 S1 最小接口与 mock 协议，不声称全链路或 S1 全部验收通过。
Jetson 继续延期，无实机操作。
