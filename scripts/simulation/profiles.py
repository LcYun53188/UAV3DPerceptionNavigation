"""仿真配置注册表：菜单、命令行和 JSON 配置共用的选项与参数规则。"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

from .backends import ROOT, command as backend_command

SIM = str(ROOT / 'scripts/sim.sh')


@dataclass(frozen=True)
class Option:
    key: str
    label: str
    default: object
    choices: tuple = ()
    limits: tuple = ()

    def validate(self, value):
        if self.choices:
            if type(value) is not type(self.default) or value not in dict(self.choices):
                raise ValueError(f'{self.label}：请选择列出的选项')
        elif self.limits:
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or not self.limits[0] <= value <= self.limits[1]
                    or (type(self.default) is int and type(value) is not int)):
                raise ValueError(f'{self.label}：范围 {self.limits[0]}–{self.limits[1]}，单位见提示')
        elif not isinstance(value, str) or any(c in value for c in ('\0', '\n', '\r')):
            raise ValueError(f'{self.label}：需要单行文本路径')
        if self.key == 'device' and value != 'auto' and (not value or not value.startswith('/dev/input/')):
            raise ValueError('摇杆设备需要 auto 或 /dev/input/ 下的绝对路径')
        return value


@dataclass(frozen=True)
class Profile:
    key: str
    title: str
    note: str
    options: tuple = ()


YES_NO = ((True, '是'), (False, '否'))
UI = Option('ui', '显示 Gazebo／QGC 界面', True, YES_NO)
DEVICE = Option('device', '虚拟 Pocket 实际设备路径', '/dev/input/js0')
QGC = Option('qgc_config', '继承 QGC 校准文件（留空则不导入）', '')
ARENAS = tuple((key, label) for key, label in (
    ('uav_obstacle_course', '障碍穿越场地'), ('uav_harmonic_demo', '基础演示'),
    ('rmuc_2024', 'RMUC 2024'), ('rmuc_2025', 'RMUC 2025'),
    ('rmul_2024', 'RMUL 2024'), ('rmul_2025', 'RMUL 2025')))
PROFILES = (
    Profile('algorithm', '算法建图／探索／RViz 导航',
            '使用真值定位和简化速度执行器。首次构建：scripts/build/build_algorithm_sim.sh。', (
                Option('layout', '场景', 'expanded', (('expanded', '扩展场景 20×20 m'), ('lab', '实验室 10×10 m'))),
                Option('mode', '地图模式', 'mapping', (('mapping', '在线建图'), ('localization', '加载静态地图'))),
                Option('view', '界面', 'both', (('both', 'Gazebo＋RViz'), ('rviz', '仅 RViz'), ('none', '无界面'))),
                Option('goal_height', 'RViz 目标高度（米）', 1.2, limits=(0.1, 3.5)),
            )),
    Profile('scene', 'Gazebo 场景与传感器查看',
            '只启动场景和传感器桥，不运行导航／PX4。需已构建 uav_bringup。', (
                Option('arena', '场景', 'uav_obstacle_course', ARENAS),
                Option('mid360', '启用 MID360 桥接', False, YES_NO),
            )),
    Profile('px4-flight', 'PX4＋BT 旧 GNSS 自动任务［已停用］',
            '全局无 GNSS：旧 W0 任务尚未接入无 GNSS 定位，禁止启动；使用 VIO／光流入口。', (
                Option('scenario', '任务', 'full', (('full', '起飞→航点→悬停→返航→降落'),
                       ('pause-resume', '运动中暂停／恢复'), ('cancel', '运动中取消并降落'))), UI,
            )),
    Profile('vio-pocket', '无 GNSS VIO／深度／EGO＋Pocket 操作',
            '仓库 avoidance，固定 SDK Schur64＋VIO 主定位。由遥控器开关发起任务，不自动起飞。\n'
            '实体 Pocket 选择 pocket-rc-v2（SB/SC/SD 新规划）；虚拟面板选择 uav-aetr-v1。', (
                Option('device', 'Pocket 实际设备路径', '/dev/input/js0'),
                Option('input_profile', 'Pocket 开关配置', 'uav-aetr-v1',
                       (('uav-aetr-v1', '虚拟面板（9轴／3按钮）'), ('pocket-rc-v2', '实体遥控器 SB/SC/SD（8轴／24按钮）'),
                        ('legacy-usb', '兼容旧实体 SA/SB/SD 二态配置'))),
                Option('task', '自动任务', 'avoidance', (('avoidance', '扫描／静态避障任务'),
                    ('intervention', '起点附近手动介入验证（水平速度上限 0.5 m/s）'))),
                QGC, Option('session_duration', '操作会话时长（秒）', 3600, limits=(60, 14400)),
            )),
    Profile('flow-pocket', '无 GNSS 纯光流仓库＋Pocket 操作',
            '仅光流／测距定位，不提供地图避障。支持实体与虚拟 Pocket；不自动起飞。', (
                DEVICE, Option('input_profile', 'Pocket 开关配置', 'uav-aetr-v1',
                    (('uav-aetr-v1', '虚拟面板'), ('pocket-rc-v2', '实体 SB/SC/SD 新规划'), ('legacy-usb', '旧实体配置'))),
                QGC, Option('duration', '地面审计时长（秒）', 35, limits=(20, 120)),
                Option('session_duration', '操作会话时长（秒）', 3600, limits=(20, 3600)),
            )),
    Profile('flow-local', '纯光流自动起飞／悬停／降落测试',
            '地面审计通过后自动起飞。原生 Position 使用模拟接收机，不是实体 Pocket 验证。', (
                Option('control', '控制方式', 'native', (('native', '原生 Position＋模拟接收机'),
                       ('mission', 'FLOW_LOCAL 任务网关'))),
                Option('duration', '地面审计时长（秒）', 35, limits=(20, 120)), UI,
            )),
    Profile('vio-audit', 'VIO／光流联合融合审计（未解锁）',
            '实际 VIO／光流／测距进入 PX4 EKF，结束前停源审计；不启动飞行任务。', (
                Option('variant', '仓库变体', 'baseline', (('baseline', '基础仓库'), ('complex', '复杂几何／纹理'))),
                Option('duration', '观察时长（秒）', 120, limits=(60, 240)),
                Option('fault', '优先停止的源', 'vio', (('vio', 'VIO'), ('flow', '光流'), ('range', '测距'), ('all', '全部'))),
                Option('resolution', '双目分辨率', '640x400', (('640x400', '640×400'), ('480x300', '480×300'))),
                Option('rtf', '仿真实时倍率', 0.8, ((0.8, '0.8 倍'), (1.0, '1.0 倍'))),
                Option('renderer', '渲染设备', 'nvidia', (('nvidia', 'NVIDIA'), ('default', '系统默认'), ('intel', 'Intel'))),
                UI,
            )),
    Profile('depth', 'PX4 深度／建图／规划检查（未解锁）',
            '无 GNSS、无飞行任务；当前可检查相机。独立建图／规划尚未接入无 GNSS 定位；请使用 VIO 建图链路。', (
                Option('level', '检查层级', 'camera', (('camera', '深度相机'), ('map', '深度＋nvblox［无 GNSS 定位待接入］'),
                       ('plan', '深度＋nvblox＋EGO［无 GNSS 定位待接入］'))),
                Option('duration', '观察时长（秒）', 45, limits=(20, 120)), UI,
            )),
    Profile('panel', '虚拟 Pocket 控制面板',
            '面板独立运行；显示设备路径后，在另一终端启动 Pocket 仿真。', (
                Option('uinput', '创建虚拟摇杆，供 QGC／仿真使用', True, YES_NO),
            )),
    Profile('isaac', 'Isaac Sim 4.5 UI', '需已安装 Isaac Sim 4.5；沿用 ISAAC_SIM_* 环境变量。'),
    Profile('doctor', '离线仿真环境检查', '只检查依赖，不表示在线话题、TF、时钟或飞行就绪。'),
    Profile('sitl', 'PX4 基础状态检查（未解锁）', '只检查 x500／DDS／时钟／QGC，不运行飞行任务。', (
                Option('duration', '观察时长（秒）', 45, limits=(10, 180)), UI,
            )),
    Profile('manual', 'Gazebo＋RViz 手动飞行',
            '由实体 Pocket 或电脑虚拟面板经 QGC 操纵；自动任务关闭，不自动解锁。当前仅接通 VIO＋建图仓库链路。', (
                Option('vio', '启动 VIO', True, YES_NO),
                Option('mapping', '启动建图', True, YES_NO),
                Option('scene', '场景', 'warehouse-avoidance', (('warehouse-avoidance', '仓库静态障碍场景'),
                       ('px4-default', 'PX4 基础场景'))),
                Option('input_source', '手动控制输入', 'auto', (('auto', '自动：优先实体 Pocket，否则使用虚拟设备'),
                       ('physical', '仅实体 Pocket（8轴／24按钮）'),
                       ('virtual', '电脑虚拟 Pocket（9轴／3按钮）'))),
                Option('device', 'Pocket 设备路径（auto 自动选择）', 'auto'),
                QGC, Option('session_duration', '操作会话时长（秒）', 3600, limits=(60, 14400)),
            )),
    Profile('autonomous-mapping', 'Gazebo＋RViz 自主探索建图',
            '算法仿真：自动初始化起点，再启动自主探索。使用真值定位与简化速度模型；全场覆盖仍以探索结果为准。', (
                Option('layout', '场景', 'expanded', (('expanded', '扩展场景 20×20 m'), ('lab', '实验室 10×10 m'))),
            )),
    Profile('navigation', 'Gazebo＋RViz 任务点与行为树配置',
            '算法仿真：选择地图来源与场景，再选择任务点和行为树配置。当前可运行 RViz 单点导航。', (
                Option('load_map', '加载现有地图', False, YES_NO),
                Option('layout', '场景', 'expanded', (('expanded', '扩展场景 20×20 m'), ('lab', '实验室 10×10 m'))),
                Option('map_bundle', '地图包目录', ''),
                Option('task_source', '任务点与行为树配置', 'rviz', (('rviz', 'RViz 单点导航（默认导航流程）'),
                       ('algorithm-waypoints', '多航点＋算法行为树（待接入交互选择）'),
                       ('custom-tree', '自定义行为树 XML（待接入）'))),
            )),
)
BY_KEY = {profile.key: profile for profile in PROFILES}


def validate_config(config):
    if not isinstance(config, dict) or set(config) - {'schema', 'profile', 'options'}:
        raise ValueError('配置只接受 schema、profile、options 字段')
    if type(config.get('schema')) is not int or config['schema'] != 1:
        raise ValueError('配置 schema 必须为 1')
    key = config.get('profile')
    if not isinstance(key, str) or key not in BY_KEY:
        raise ValueError('未知仿真 profile；使用 menu --list 查看')
    profile = BY_KEY[key]
    options = config.get('options', {})
    if not isinstance(options, dict) or set(options) - {item.key for item in profile.options}:
        raise ValueError('options 包含未知配置项')
    options = {item.key: item.validate(options.get(item.key, item.default)) for item in profile.options}
    if key == 'manual':
        reason = manual_unavailable(options['vio'], options['mapping'], options['scene'])
        if reason:
            raise ValueError(reason)
    if key == 'navigation':
        if options['load_map'] and not options['map_bundle'].strip():
            raise ValueError('加载现有地图需要填写地图包目录')
        if options['task_source'] != 'rviz':
            raise ValueError('多航点交互选择和自定义行为树尚未接入该启动流程；请使用专项 BT 工具')
    return dict(schema=1, profile=key, options=options)


def manual_unavailable(vio, mapping, scene):
    if not vio:
        return '待接入：无 VIO 的手动链路尚未统一 RViz／可选建图；可使用无 GNSS 光流专项入口'
    if not mapping:
        return '待接入：当前 VIO 手动链路仍与深度建图绑定，尚不能独立关闭建图'
    if scene != 'warehouse-avoidance':
        return '待接入：VIO 手动飞行目前只绑定仓库静态障碍场景'
    return ''


def path_arg(value):
    path = Path(value).expanduser()
    return str((ROOT / path).resolve())


def command(config):
    config = validate_config(config)
    key, o = config['profile'], config['options']
    target, args = key, []
    if key == 'manual':
        target = 'manual'
        args = ['--input-source', o['input_source'], '--device', o['device'],
                '--session-duration', str(o['session_duration'])]
        if o['qgc_config']:
            args += ['--qgc-config-from', path_arg(o['qgc_config'])]
    elif key in ('autonomous-mapping', 'navigation'):
        target = 'algorithm'
        mode = 'localization' if o.get('load_map') else 'mapping'
        args = ['start', '--layout', o['layout'], '--mode', mode, '--view', 'both']
        if key == 'autonomous-mapping':
            args += ['--startup', 'explore']
        elif o['load_map']:
            args += ['--map-bundle', path_arg(o['map_bundle'])]
        else:
            args += ['--startup', 'init']
    elif key == 'algorithm':
        args = ['start', '--layout', o['layout'], '--mode', o['mode'], '--view', o['view'],
                '--goal-height', str(o['goal_height'])]
    elif key == 'scene':
        args = ['arena:=' + o['arena'], 'launch_mid360:=' + str(o['mid360']).lower()]
    elif key == 'px4-flight':
        target, args = 'flight', ['--bt', '--flight-scenario', o['scenario']]
    elif key in ('vio-pocket', 'flow-pocket'):
        if key == 'vio-pocket':
            args += ['--sdk-schur64', '--vio-primary', '--input-profile', o['input_profile']]
            if o['task']!='avoidance':args += ['--pocket-task',o['task']]
        else:
            args += ['--duration', str(o['duration']), '--input-profile', o['input_profile']]
        args += ['--device', o['device'], '--session-duration', str(o['session_duration'])]
        if o['qgc_config']:
            args += ['--qgc-config-from', path_arg(o['qgc_config'])]
    elif key == 'flow-local':
        args = ['--duration', str(o['duration'])]
        if o['control'] == 'native':
            args += ['--native-manual']
    elif key == 'vio-audit':
        target = 'vio-joint'
        args = ['--warehouse-variant', o['variant'], '--duration', str(o['duration']),
                '--joint-fault', o['fault'], '--quality-policy', 'bounded_gap',
                '--real-time-factor', str(o['rtf']), '--render-device', o['renderer'],
                '--image-resolution', o['resolution'], '--sdk-image-depth', '1',
                '--headless-rendering', '--sdk-executor', 'single_thread', '--sdk-cpu-list', '0-15']
        if o['ui']:
            args += ['--diagnostic-ui']
    elif key == 'depth':
        target = dict(camera='depth-camera', map='depth-map', plan='depth-plan')[o['level']]
        args = ['--duration', str(o['duration'])]
    elif key == 'panel':
        args = ['--uinput'] if o['uinput'] else []
    elif key == 'doctor':
        target, args = 'algorithm', ['doctor']
    elif key == 'sitl':
        args = ['--duration', str(o['duration'])]
    if o.get('ui'):
        args += ['--ui']
    return backend_command(target, args)


def read_config(path):
    return validate_config(json.loads(path.read_text()))


def save_config(config, path):
    config = validate_config(config)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as handle:
        handle.write(json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(f'已保存：{path}')
