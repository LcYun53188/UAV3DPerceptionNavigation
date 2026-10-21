"""唯一的运行器与环境映射；保留旧入口、参数顺序、信号和退出码。"""
from dataclasses import dataclass
import hashlib
import os
from pathlib import Path
import sys

from .policy import require_backend

ROOT = Path(__file__).resolve().parents[2]
ENVIRONMENT = ROOT / 'scripts/simulation/environment.sh'


@dataclass(frozen=True)
class Backend:
    title: str
    environment: str
    runner: str
    arguments: tuple = ()


# Fixed presets live here once. User options are appended so the original
# runners keep their last-option-wins CLI and their own admission checks.
VIO_POCKET = (
    '--experimental-chain', '--flight', 'sequence', '--pocket-session', '--normalize',
    '--mtf01', '--scene', 'warehouse', '--warehouse-variant', 'avoidance',
    '--warehouse-floor-texture', '--quality-policy', 'bounded_gap',
    '--real-time-factor', '0.8', '--render-device', 'nvidia', '--sdk-image-depth', '1',
    '--headless-rendering', '--sdk-executor', 'single_thread', '--sdk-cpu-list', '0-15',
    '--image-resolution', '480x300', '--ui', '--diagnostic-ui', '--duration', '120',
)
BACKENDS = {
    'manual': Backend('自动选择实体／虚拟 Pocket 的纯手动会话', 'host', 'scripts/simulation/manual.py'),
    'algorithm': Backend('算法会话控制', 'host', 'scripts/sim_control.py'),
    'scene': Backend('Gazebo 场景与传感器桥', 'shell', 'scripts/simulation/launch_scene.sh'),
    'isaac': Backend('Isaac Sim 4.5 UI', 'shell', 'scripts/simulation/launch_isaac.sh'),
    'panel': Backend('虚拟 Pocket 面板', 'host', 'scripts/pocket_panel.py'),
    'sitl': Backend('PX4 未解锁基础检查', 'px4', 'scripts/run_px4_sitl_smoke.py'),
    'flight': Backend('PX4 自动飞行任务', 'mission', 'scripts/run_px4_sitl_smoke.py',
                      ('--duration', '20', '--flight')),
    'observe': Backend('PX4 EGO 观察飞行', 'mission', 'scripts/run_px4_sitl_smoke.py',
                       ('--duration', '20', '--flight', '--bt', '--navigation-backend', 'EGO', '--depth-observation')),
    'depth-camera': Backend('PX4 深度相机检查', 'px4', 'scripts/run_px4_sitl_smoke.py', ('--depth-camera',)),
    'depth-map': Backend('PX4 深度建图检查', 'px4', 'scripts/run_px4_sitl_smoke.py', ('--depth-reference', '--depth-mapping')),
    'depth-plan': Backend('PX4 深度规划检查', 'px4', 'scripts/run_px4_sitl_smoke.py',
                          ('--depth-reference', '--depth-mapping', '--depth-planning')),
    'vision-audit': Backend('PX4 合成 EV 融合检查', 'mission', 'scripts/run_px4_sitl_smoke.py', ('--vision-fusion-smoke',)),
    'flow': Backend('光流／测距未解锁审计', 'px4', 'scripts/run_px4_mtf01.py'),
    'flow-local': Backend('光流自动飞行', 'mission', 'scripts/run_px4_mtf01.py', ('--flight',)),
    'flow-pocket': Backend('光流 Pocket 操作', 'mission', 'scripts/run_px4_mtf01.py', ('--usb-joystick', '--ui')),
    'vio': Backend('VIO 原始运行器', 'vio', 'scripts/run_px4_vio_sensors.py'),
    'vio-joint': Backend('VIO 联合未解锁审计', 'vio', 'scripts/run_px4_vio_sensors.py',
                         ('--mtf01', '--normalize', '--fuse-pose', '--scene', 'warehouse', '--warehouse-floor-texture', '--duration', '120')),
    'vio-pocket': Backend('VIO 仓库 Pocket 操作', 'vio', 'scripts/run_px4_vio_sensors.py', VIO_POCKET),
}

LEGACY_COMMANDS = {
    'pocket-panel': 'panel', 'px4-vio-pocket': 'vio-pocket', 'px4-flow-pocket': 'flow-pocket',
    'px4-vio-mtf01': 'vio-joint', 'px4-flow-local': 'flow-local', 'px4-mtf01': 'flow',
    'px4-vio-sensors': 'vio', 'px4-flight': 'flight', 'px4-observe': 'observe',
    'px4-depth-plan': 'depth-plan', 'px4-depth-map': 'depth-map', 'px4-depth': 'depth-camera',
    'px4-vision-audit': 'vision-audit',
}
LEGACY_SCRIPTS = {
    'run_px4_flight.sh': ('flight', ()),
    'run_px4_vio_sensors.sh': ('vio', ()),
    'run_gazebo_harmonic_nav.sh': ('scene', ()),
    'run_uav_obstacle_course.sh': ('scene', ('arena:=uav_obstacle_course',)),
    'run_isaac_sim_45_ui.sh': ('isaac', ()),
}
ALGORITHM_COMMANDS = ('start', 'stop', 'status', 'init', 'cancel', 'explore', 'doctor',
                      'logs', 'survey', 'save', 'load', 'export-mesh', 'goal')


def source_hashes():
    """Record launch code in new run manifests, including calls via old symlinks."""
    files = [ROOT / name for name in ('scripts/sim.sh', 'scripts/with_venv.sh', 'scripts/with_px4_sim.sh', 'scripts/pocket_input.py', 'scripts/usb_joystick.py')]
    files += sorted(path for path in Path(__file__).parent.iterdir() if path.suffix in ('.py', '.sh'))
    return {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest() for path in files}


def command(key, arguments=()):
    require_backend(key)
    backend = BACKENDS[key]
    runner = str(ROOT / backend.runner)
    if backend.environment == 'host':
        prefix = [sys.executable, runner]
    elif backend.environment == 'shell':
        prefix = [runner]
    else:
        prefix = [str(ENVIRONMENT), backend.environment, 'python', runner]
    return [*prefix, *backend.arguments, *arguments]


def execute(arguments):
    os.chdir(ROOT)
    sys.stdout.flush()
    # No intermediate subprocess: supervisors receive terminal signals directly.
    os.execv(arguments[0], arguments)
