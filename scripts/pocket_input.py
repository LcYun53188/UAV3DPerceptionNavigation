"""Read-only Pocket identity checks before starting owned simulation processes."""
from pathlib import Path

from pocket_virtual_joystick import NAME as VIRTUAL_NAME
from usb_joystick import Joystick, devices

PHYSICAL_NAME = 'EdgeTX Radiomaster Pocket Joystick'
PHYSICAL_NAMES = (PHYSICAL_NAME, 'OpenTX Radiomaster Pocket Joystick')
PHYSICAL_DEVICE = '/dev/input/by-id/usb-EdgeTX_Radiomaster_Pocket_Joystick_00000000001B-joystick'
INPUT_PROFILES = {'physical': 'pocket-rc-v2', 'virtual': 'uav-aetr-v1'}


def validate_identity(name, axes, buttons, profile):
    if profile == 'uav-aetr-v1':
        expected_name, expected_shape = VIRTUAL_NAME, (9, 3)
        remedy = '先运行 ./scripts/sim.sh run panel，并使用面板输出的设备路径；实体 Pocket 请选“实体 Pocket”'
    elif profile in ('legacy-usb', 'pocket-rc-v2'):
        expected_name, expected_shape = ' / '.join(PHYSICAL_NAMES), (8, 24)
        remedy = '请选择实体 RadioMaster Pocket 的 USB Joystick 设备；虚拟面板请选“电脑虚拟 Pocket”'
    else:
        raise ValueError('未知 Pocket 输入配置：' + str(profile))
    names = PHYSICAL_NAMES if profile in ('legacy-usb', 'pocket-rc-v2') else (expected_name,)
    if name.casefold() not in {item.casefold() for item in names} or (axes, buttons) != expected_shape:
        raise ValueError(f'Pocket 设备类型不匹配：需要 {expected_name} {expected_shape[0]}轴/{expected_shape[1]}按钮，'
                         f'实际为 {name or "未知设备"} {axes}轴/{buttons}按钮。{remedy}')


def preflight_device(device, profile):
    """Inspect a separate read-only fd; never consume another reader or send input."""
    try:
        resolved = Path(device).resolve(strict=True)
        joystick = Joystick(str(resolved))
    except OSError as exc:
        raise ValueError(f'无法读取摇杆 {device}：{exc}；检查 USB Joystick 模式或先启动虚拟面板') from exc
    try:
        name = (Path('/sys/class/input') / resolved.name / 'device/name').read_text().strip()
        validate_identity(name, joystick.axes, joystick.buttons, profile)
        return dict(device=str(device), resolved_device=str(resolved), name=name,
                    axes=joystick.axes, buttons=joystick.buttons, input_profile=profile)
    finally:
        joystick.close()


def select_device(source='auto', device='auto'):
    """Prefer a physical Pocket. None means an owned virtual panel is needed.

    An unreadable/malformed physical device is an error, not a reason to silently
    switch controls. An explicit device path is an intentional user override.
    """
    if source not in ('auto', *INPUT_PROFILES):
        raise ValueError('未知手动输入来源：' + str(source))
    if device != 'auto':
        if source == 'auto':
            resolved = Path(device).resolve()
            try:
                name = (Path('/sys/class/input') / resolved.name / 'device/name').read_text().strip()
            except OSError as exc:
                raise ValueError(f'无法识别摇杆 {device}：{exc}') from exc
            source = 'physical' if name.casefold() in {item.casefold() for item in PHYSICAL_NAMES} else 'virtual'
        return dict(preflight_device(device, INPUT_PROFILES[source]), input_source=source)
    available = devices()
    for candidate in (('physical', 'virtual') if source == 'auto' else (source,)):
        names = PHYSICAL_NAMES if candidate == 'physical' else (VIRTUAL_NAME,)
        expected = ' / '.join(names)
        matches = [item for item in available if item['name'].casefold() in {name.casefold() for name in names}]
        if len(matches) > 1:
            raise ValueError(f'发现多个 {expected}，请用 --device 明确选择')
        if matches:
            path = matches[0]['path']
            if candidate == 'physical' and Path(PHYSICAL_DEVICE).exists() and Path(PHYSICAL_DEVICE).resolve() == Path(path).resolve():
                path = PHYSICAL_DEVICE
            elif candidate == 'physical':
                stable = Path('/dev/input/by-id') / ('usb-' + matches[0]['name'].replace(' ', '_') + '_00000000001B-joystick')
                if stable.exists() and stable.resolve() == Path(path).resolve():
                    path = str(stable)
            return dict(preflight_device(path, INPUT_PROFILES[candidate]), input_source=candidate)
    if source == 'physical':
        raise ValueError('未检测到实体 Pocket，请连接数据 USB 并选择 USB Joystick')
    return None
