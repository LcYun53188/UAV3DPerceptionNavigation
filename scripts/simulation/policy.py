"""Workspace simulation policy: no satellite measurements or GNSS aiding."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
SERVER_CONFIG = ROOT / 'simulation/px4/server_control.config'
DISABLED_BACKENDS = frozenset(('flight', 'observe', 'depth-map', 'depth-plan'))
DISABLED_REASON = ('全局无 GNSS：此旧入口依赖 GNSS 定位，尚未接入无 GNSS 定位源。'
                   '请使用 sim.sh run manual（光流纯手动）或 sim.sh run full（完整项目）')


def require_backend(key):
    if key in DISABLED_BACKENDS:
        raise ValueError(DISABLED_REASON)


def no_gnss_environment(env, world):
    """Apply last, after presets and inherited variables, before any process starts."""
    for path in (SERVER_CONFIG, Path(world)):
        root = ET.parse(path).getroot()
        for plugin in root.iter('plugin'):
            identity = (plugin.get('filename', '') + ' ' + plugin.get('name', '')).casefold()
            if 'navsat' in identity or 'gnss' in identity or 'gps' in identity:
                raise ValueError(f'全局无 GNSS：禁止卫星传感器插件 {path}')
    env.update(PX4_PARAM_EKF2_GPS_CTRL='0', GZ_SIM_SERVER_CONFIG_PATH=str(SERVER_CONFIG))
    return dict(gnss_enabled=False, gps_fusion=False, navsat_system=False,
                policy='no_gnss_v1')
