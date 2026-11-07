"""Preserve QGC calibration, seed fixed virtual controls, and repair Pocket ranges."""
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


def virtual_defaults(config, identity):
    """Seed a new software device from its fixed AETR contract, not a HID measurement.

    Only the project's 9-axis/3-button uinput device qualifies. Existing device
    settings (including incomplete user calibration) remain authoritative.
    QGC sends the four sticks; the sole project gateway still owns switches.
    """
    from pocket_virtual_joystick import NAME
    if (identity.get('name') != NAME or identity.get('input_profile') != 'uav-aetr-v1'
            or (identity.get('axes'), identity.get('buttons')) != (9, 3)):
        return False
    prefix = device_prefix(identity)
    section = 'JoystickSettingsV2'
    if config.has_section(section) and any(key.startswith(prefix) for key in config[section]):
        return False
    if not config.has_section(section):
        config.add_section(section)
    values = config[section]
    values.update({prefix+'calibrated': 'true', prefix+'transmitterMode': '2',
                   prefix+'throttleModeCenterZero': 'false', prefix+'negativeThrust': 'false',
                   prefix+'throttleSmoothing': 'false', prefix+'exponentialPct': '0',
                   prefix+'circleCorrection': 'false'})
    for axis, function in enumerate((0, 1, 3, 2)):
        base = prefix+f'JoystickAxisSettingsArray\\{axis}\\'
        values.update({base+'function': str(function), base+'min': '-32768',
                       base+'max': '32767', base+'center': '0', base+'deadband': '0',
                       base+'reversed': 'false'})
    if not config.has_section('JoystickManager'):
        config.add_section('JoystickManager')
    manager = config['JoystickManager']
    vehicles = [value for value in manager.get('joystickEnabledVehiclesIds', '').split(',') if value]
    if '8' not in vehicles:  # The owned PX4 SITL instance, never a hardware vehicle.
        vehicles.append('8')
    manager['joystickEnabledVehiclesIds'] = ','.join(vehicles)
    return True


def ready(config, identity):
    """Cached settings must already contain all four complete axis functions."""
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
