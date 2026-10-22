"""Preserve device-specific QGC calibration and repair signed Pocket throttle ranges."""
import configparser
import hashlib
import os
from pathlib import Path
from urllib.parse import quote

ROOT = Path(__file__).resolve().parents[3]
CACHE = ROOT/'.cache/simulation/qgc-calibration'


def read(path):
    config = configparser.ConfigParser(interpolation=None, strict=False)
    config.optionxform = str
    with Path(path).open() as stream:
        config.read_file(stream)
    return config


def device_prefix(identity):
    return quote(identity['name'], safe='') + '\\'


def ready(config, identity):
    """A cache never replaces calibration: all four functions must already exist."""
    prefix = device_prefix(identity)
    values = config['JoystickSettingsV2'] if config.has_section('JoystickSettingsV2') else {}
    if values.get(prefix+'calibrated', '').lower() != 'true':
        return False
    functions = []
    try:
        for axis in range(identity['axes']):
            base = prefix+f'JoystickAxisSettingsArray\\{axis}\\'
            if base+'function' not in values:
                continue
            function = int(values[base+'function'])
            if function not in range(4):
                continue
            low, high, center = (int(values[base+key]) for key in ('min','max','center'))
            if not -32768 <= low < high <= 32767 or not low <= center <= high:
                return False
            functions.append(function)
    except (KeyError, ValueError):
        return False
    return sorted(functions) == [0,1,2,3]


def normalize_throttle(config, identity):
    """QGC down-zero mode needs a signed center, not a one-sided min/max center.

    QGC 5.1.5 _adjustRange(center=min) yields [0,1]; down-zero conversion
    then yields [0.5,1], so PX4 sees [0,1] instead of [-1,1]. Preserve the
    measured endpoints, axis assignment, reversal, and calibrated flag.
    """
    from pocket_input import PHYSICAL_NAMES
    if (identity['name'] not in PHYSICAL_NAMES or identity.get('axes') != 8
            or identity.get('buttons') != 24 or not ready(config, identity)):
        return None
    prefix = device_prefix(identity)
    values = config['JoystickSettingsV2']
    if values.get(prefix+'throttleModeCenterZero', 'false').lower() != 'false':
        return None
    for axis in range(identity['axes']):
        base = prefix+f'JoystickAxisSettingsArray\\{axis}\\'
        if values.get(base+'function') != '3':
            continue
        low, high, center = (int(values[base+key]) for key in ('min','max','center'))
        if low not in (-32768,-32767) or high != 32767 or center not in (low, high):
            return None
        values[base+'center'] = '0'
        return dict(axis=axis, previous_center=center, center=0, min=low, max=high)
    return None


def cache_path(identity):
    key = identity['name']+'\0'+identity['input_profile']
    return CACHE/(hashlib.sha256(key.encode()).hexdigest()[:24]+'.ini')


def cached(identity):
    path = cache_path(identity)
    try:
        return path if ready(read(path), identity) else None
    except (OSError, configparser.Error):
        return None


def remember(source, identity):
    config = read(source)
    if not ready(config, identity):
        return None
    normalize_throttle(config, identity)
    path = cache_path(identity)
    path.parent.mkdir(parents=True, exist_ok=True)
    pending = path.with_suffix('.pending.ini')
    with pending.open('w') as stream:
        config.write(stream, space_around_delimiters=False)
    os.replace(pending, path)
    return path
