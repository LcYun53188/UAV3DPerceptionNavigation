# RGB 人员检测与地图定位

可选人员感知模块：RGB 检测 → 配准深度取样 → 采集时刻的相机到地图变换 → 临时轨迹 → RViz 实时与历史标注。输出点表示 **可见躯干附近表面代表点**，不是人体几何中心或脚底位置。检测置信度和深度支持指标分别输出；横卧人体仍采用框内深度区域取样，不提供解剖学关键点。

模块不发布导航目标、轨迹或飞控命令；人员标注不作为动态避障资格。默认两个仿真程序及菜单保持原行为，通过 full 的 `--people` 显式开启。单人静态 Gazebo 场景已测试，实机 OAK-D 彩色流、动态人物和多人仍需独立验证。

## 准备与构建

在工作区根目录执行。可选依赖保持 ROS Jazzy 的 NumPy 1.x ABI：

```bash
uv pip install --python .venv/bin/python -r requirements/person-perception.txt
./scripts/with_venv.sh colcon build \
  --base-paths src/uav_nav_interfaces src/uav_target_perception \
  --build-base build_uav --install-base install_uav --symlink-install \
  --packages-select uav_nav_interfaces uav_target_perception \
  --cmake-args -DBUILD_TESTING=OFF
```

使用官方 [YOLOX-Tiny ONNX 发布模型](https://github.com/Megvii-BaseDetection/YOLOX/tree/main/demo/ONNXRuntime)，只输出 COCO 的 person 类。固定输入为 416×416、BGR、左上对齐 padding=114，CPU 推理默认两个线程。运行时不下载模型；启动前检查权重 SHA-256。

```bash
./scripts/with_venv.sh ros2 run uav_target_perception person_model prepare \
  .cache/models/person/yolox_tiny.onnx

# 离线文件也必须通过同一摘要校验；已有不同模型不会被覆盖
./scripts/with_venv.sh ros2 run uav_target_perception person_model prepare \
  .cache/models/person/yolox_tiny.onnx --source /绝对路径/yolox_tiny.onnx
```

固定模型 `0.1.1rc0/yolox_tiny.onnx` 的 SHA-256：
`427cc366d34e27ff7a03e2899b5e3671425c262ea2291f88bb942bc1cc70b0f7`。
上游许可见 [YOLOX LICENSE](https://github.com/Megvii-BaseDetection/YOLOX/blob/main/LICENSE)。模型保留在忽略的缓存目录。

## 完整项目运行

先完成[完整项目构建](../../docs/simulation/LAUNCHERS.md#构建准备)。预览不创建输入设备或启动仿真：

```bash
./scripts/sim.sh run full --people --dry-run
./scripts/sim.sh run full --people
# 需要模拟遥控器时显式选择；同样不自动解锁或起飞
./scripts/sim.sh run full --people --input-source virtual
```

`--person-model /绝对路径/yolox_tiny.onnx` 指定模型；保存的 full JSON 可设置 `people: true`、`person_model`。菜单不增加配置步骤，加载 JSON 时保留人员开关。纯光流 manual 不提供人员地图定位。

完整项目额外启动同机 RGB-D 的 RGB 桥和人员节点：`/px4_rgb/image`、`/px4_depth/image`、`/px4_depth/camera_info`。定位绑定 `/planning/context` 的地图、定位会话、对齐世代和 reset；不依赖仅在 mapping 模式转发的图像。nvblox 保持原深度配置，不额外开启彩色融合。

启用人员功能时，仿真 RGB-D 使用独立的固定相机 link，保持原安装位姿，避免 Gazebo 将其与 VIO 左右相机合并为同一双目组并写入额外投影平移。VIO 相机和安装参数保持原值；辅助 link 质量为 1 mg，生成信息写入运行清单。实际相机仍须满足配准和标定契约。

需要着色地图时，显式增加 `--color-map`，使同一 RGB 流经过地图输入门控后以 5 Hz 融合；默认配置仍关闭彩色建图。必须在开启此选项后重新扫描，旧灰色地图不能通过重新导出获得实际 RGB。

```bash
./scripts/sim.sh run full --people --color-map
# 扫描结束、飞机落地上锁后，在另一终端执行；输出必须为新文件
./scripts/sim.sh export-mesh .cache/maps/people_scan_rgb.ply
```

人员显示是单独的 RViz 标记层，PLY 保存实际重建的几何和顶点颜色，不把临时人员编号绘入静态模型。人员历史记录通过单独的快照服务保存。

RViz 增加 People in Map 与 Person Detections 显示。场景中必须实际存在可见人物；默认仓库主要是静态障碍，打开开关不等于人物场景验收通过。

## RViz 持续显示最后观测位置

默认启用 **Last Observed People** 历史图层：确认过的人体以橙色点和 `history H… last=…s` 标签保留，标记不设过期时间。相机离开或实时轨迹超过 1.5 s 后，绿色实时标记消失，橙色历史点继续显示；它表示最后观测位置，不能据此判断人员仍在原地。

历史标记发布在 `/uav/people/history_markers`，使用可靠、Transient Local、深度 1 的完整快照；感知节点运行期间，RViz 重连可收到已有记录。自定义 RViz 配置需增加该话题的 MarkerArray 显示，并选择 Transient Local。历史图层不输入规划器，也不改变原有语义地图仅关联实时人员的行为。

默认最多保留 200 个历史位置，0.6 m 空间关联可合并再次看到的相近位置，不是身份识别。超过容量时移除最旧记录。相同坐标会话中，短暂上下文失效、图像中断或仿真暂停不清除历史；地图 ID/世代、定位会话、对齐或 reset 改变，以及时钟回退时会清空。独立 launch 可设置 `retain_history:=false`、`history_radius_m:=0.6`、`history_max_records:=200`。

同一通信域手动清除历史：

```bash
./scripts/with_venv.sh ros2 service call /uav/people/clear_history std_srvs/srv/Trigger '{}'
```

清除后缓存的旧帧不会恢复历史点；新的有效观测仍可重新标记。跨进程/地图加载不自动恢复历史，快照保存见下文。

## 人体仿真模型与定位测试

可选场景使用 [OpenRobotics Standing person](https://app.gazebosim.org/OpenRobotics/fuel/models/Standing%20person)，作者 Marina Kollmitz，MakeHuman 制作，CC0-1.0。固定 Fuel 第 3 版；26 MB 资产保存在忽略的缓存目录，运行时不联网下载。准备与校验：

```bash
./scripts/with_venv.sh python -m uav_target_perception.gazebo prepare \
  .cache/models/gazebo/standing_person/v3
# 离线包可在 prepare 命令后增加 --source /绝对路径/v3.zip
./scripts/with_venv.sh python -m uav_target_perception.gazebo check \
  .cache/models/gazebo/standing_person/v3
./scripts/sim.sh run full --people --person-scene standing
```

固定 ZIP 的 SHA-256 为 `385c4c518662f75b6c719849a4f47100bac7617058407c7d6db41bf802fc5c5e`。启动逐文件校验解压内容，修改或缺失时拒绝运行。原始资产保持不变，生成场景把人体设为静态并使用绝对网格路径；在 warehouse/avoidance 的世界坐标 (3, 3.5, 0) m 放置一个人体，朝向约 -45°，检查网格范围不侵入膨胀后的起飞/扫描保护体积。默认 `person_scene=none`，仅选择场景不会自动开启人员功能，菜单步骤保持不变。此模型用于观测，尚不代表动态人员避障资格。

新增坐姿与仰卧测试：坐姿采用 [OpenRobotics Rescue Randy Sitting 第 8 版](https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue%20Randy%20Sitting)，CC0-1.0，ZIP SHA-256 `611ef9ddadfba16be25963f2b79967fec6493c638ca77d0652f18fbbc5202fb3`。只使用静态视觉/碰撞几何，生成场景移除其外部接近传感器和热成像插件。仰卧为原站立网格刚体旋转并按网格最低点贴地的静态变体，不是关节姿态或跌倒动画。

```bash
./scripts/with_venv.sh python -m uav_target_perception.gazebo prepare \
  .cache/models/gazebo/rescue_randy_sitting/v8 --kind sitting
./scripts/sim.sh run full --people --person-scene postures --person-rotations
```

`--person-scene` 可选 `standing`、`sitting`、`lying`、`postures`；最后一种同时放置站立 (3,3.5,0)、坐姿 (-3,3.5,0)、仰卧 (-0.8,4.6,贴地高度) m，均检查扫描保护体积。姿态选择仅改变测试资产，不向检测器提供目标位置、类别或真值姿态。

`--person-rotations` 额外推理原图的 ±90° 方向，检测框映回原始像素网格后跨方向去重，再使用原始注册深度定位。门限仍为 0.5，默认关闭；开启会增加 CPU 开销。本轮原方向未检出侧向仰卧人体，开启后实际 Gazebo RGB-D 测试通过，不能据此保证所有姿态、视角或遮挡均可识别。独立 launch 参数为 `rotation_augmentation:=true`。

独立固定相机测试不启动 PX4，使用 Gazebo 实际渲染的 RGB、深度和 CameraInfo，以及明确配置的静态相机 TF 和测试地图上下文。`front`、`offset`、`far` 分别覆盖正面、偏侧和 5 m 距离，`empty` 为无人对照；每次使用新证据目录：

```bash
ROS_DOMAIN_ID=194 ROS_LOCALHOST_ONLY=1 \
  ROS_LOG_DIR="$PWD/.cache/person-scene-check/ros-logs" \
  ./scripts/with_venv.sh python scripts/simulation/validation/run_person_localization.py \
  --case front --output .cache/person-scene-check/front
```

正样本要求至少 20 个不同采集时刻的确认点、Marker 和快照，到原始人体三角网格的距离 ≤0.20 m、到模型原点的水平距离 ≤0.60 m。网格距离核验可见表面语义，不是人体中心误差；多次静态观测也不是独立精度样本。无人对照同时检查有效相机输入和零检测/零标记。固定相机测试不覆盖移动定位误差或 VIO 资格。

新增用例可增加 `--posture sitting`，或 `--posture lying --person-rotations`。水平范围按网格包围盒半对角线加 0.2 m、至少 0.6 m 设限，以覆盖横卧人体长度；主要定位指标仍为真实三角网格表面距离 ≤0.20 m。确认定位后，测试器停止图像与时钟桥并等待 3 s，检查实时人数归零、历史标记继续保留。本轮姿态和历史回放证据见[持续标记与姿态验证（本地）](../../docs/validation/simulation/2026-10-25-person-history-postures/REPORT.md)。

四个固定相机用例和一次含人体的完整 VIO/建图/导航任务已通过；完整任务记录 37 次人员观测，表面距离中位数 2.92 cm、最大 15.60 cm。具体范围、检测图、地图位置图及原始结果见[人体模型定位记录（本地）](../../docs/validation/simulation/2026-10-23-person-model-localization/REPORT.md)。这些数据限定为单人静态仿真，不代表实机精度。

## 接入已有 RGB-D 数据

对已有算法真值坐标地图，在相同 ROS_DOMAIN_ID 下独立启动：

```bash
./scripts/with_venv.sh ros2 launch uav_target_perception person_mapping.launch.py \
  model_path:="$PWD/.cache/models/person/yolox_tiny.onnx" \
  context_source:=map_snapshot context_topic:=/uav/map/snapshot
```

`map_snapshot` 模式采用算法仿真的 `gazebo_world_identity` 对齐；其他定位链路使用 `planning_context`，不能用此模式绕过定位对齐。独立启动不接管已有会话或改变 TF。

首版输入契约：

- RGB/深度已配准到相同光学坐标、尺寸和像素网格；CameraInfo 对应该网格。
- RGB 为 rgb8/bgr8/rgba8/bgra8，深度为 32FC1 米或 16UC1 毫米。
- 图像已校正，无非零畸变、ROI 或额外 binning；不能靠修改 frame_id 或拉伸深度冒充配准。
- 默认同步差 ≤30 ms、源年龄 ≤500 ms；采集时间非零且不在未来。
- 上下文有效、新鲜且发布源唯一；TF 覆盖图像采集时刻。

中央躯干区域的深度取样检查有效比例、样本量和四分位距。背景混入严重时可能拒绝定位；背景占主导时仍可能得到错误距离，尚不具备实例分割边界能力。

## 输出与有效期

| 话题 | 类型 | 含义 |
| --- | --- | --- |
| `/uav/people/detections` | `vision_msgs/Detection2DArray` | 原图人员框、检测分数及源时间 |
| `/uav/people/image` | `sensor_msgs/Image` | 带框图像 |
| `/uav/people/observations` | `uav_nav_interfaces/PersonObservationArray` | 地图坐标、上下文、临时编号、深度支持及各自观测时间 |
| `/uav/people/markers` | `visualization_msgs/MarkerArray` | 人员专用实时标记层 |
| `/uav/people/history_markers` | `visualization_msgs/MarkerArray` | 持续保留的最后观测位置，橙色历史层 |
| `/uav/people/diagnostics` | `std_msgs/String` | JSON 原因码、上下文资格及目标数量 |

数组 `valid` 表示当前坐标上下文有效，不代表存在人员或位置达到特定精度。每个点保留 `last_observed`；深度离散度不是总定位误差或协方差。无人机位姿误差会传递到地图坐标。

默认至少 3 个不同有效采集帧确认、一对一 0.8 m 关联、1.5 s 有效期、最多 50 条轨迹。编号是临时空间关联，不是身份识别；交叉和遮挡可能换号。只保留最新待处理帧、同步队列有界；历史 TF 暂缺时在期限内重试，不重复推理或代用最新 TF。

地图/定位上下文变化、失效、超时或标定变化会清空旧实时轨迹。源时间与单调时间共同限制有效期，仿真暂停后也会清除实时标注；历史层按上文规则保留或清空。时钟回退后如不能重新建立上下文时间，应重启人员节点。

## 保存历史快照

独立启动可设置 `snapshot_path:=/绝对路径/新文件.json`。full 配置为本轮运行目录的 `person-observations.json`。在该会话通信域调用：

```bash
./scripts/with_venv.sh ros2 service call /uav/people/save_snapshot std_srvs/srv/Trigger '{}'
```

快照包含当前有效观测、坐标上下文、模型和相机标定摘要，另以 `history` 数组保存最后观测位置、捕获时间和来源临时编号；`history_kind` 明确标为非实时位置。保存仍要求当前有效的唯一坐标上下文，独占创建新文件，不覆盖已有记录。文件不在加载地图时恢复成实时人员。

## 验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 ./scripts/with_venv.sh python -m pytest -q \
  src/uav_target_perception/test

# 实际模型推理；使用本地准备的人员图片，输出路径须不存在
./scripts/with_venv.sh ros2 run uav_target_perception person_model detect \
  .cache/models/person/yolox_tiny.onnx /绝对路径/person.jpg \
  --output .cache/person-detections.jpg

# 隔离 ROS 域，实际 RGB 推理 + 显式合成深度/TF，无飞控输出
ROS_DOMAIN_ID=193 ROS_LOCALHOST_ONLY=1 \
  UAV_PERSON_MODEL="$PWD/.cache/models/person/yolox_tiny.onnx" \
  UAV_PERSON_IMAGE=/绝对路径/person.jpg PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  ./scripts/with_venv.sh python -m pytest -q \
  src/uav_target_perception/test/test_ros_pipeline.py
```

离线几何、真实图片推理、ROS 合成几何链路与实际 Gazebo 人物几何分别解释。无人物仓库场景的联合扫描、导航和着色导出已运行，失败及产物见[联合仿真记录（本地）](../../docs/validation/simulation/2026-10-23-full-people-color/REPORT.md)。单人静态人物测试范围见上节；动态/多人物、OAK-D RGB-D 配准与运动误差尚需独立验收，既有导航验收结论保持原适用范围。

## RViz 地图颜色选择

统一启动入口支持 `./scripts/sim.sh run full --map-color semantic`，会同时开启人员检测，将可见躯干表面点附近网格标为紫色，其余区域保持灰色未分类。该显示是空间关联，不是逐像素语义分割；关联半径用 `--map-semantic-radius 0.4`（m）调整，过期人员不会永久染色。

高度、RGB、法线、ESDF、融合权重、更新年龄及可通行性等八种选择和参数见[启动脚本的 RViz 地图着色说明](../../docs/simulation/LAUNCHERS.md#rviz-地图着色)。
