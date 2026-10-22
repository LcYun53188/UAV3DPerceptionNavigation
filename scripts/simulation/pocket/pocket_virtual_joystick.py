"""Linux uinput Pocket transport using only the standard library.

joydev: CH1–CH9 -> axes 0–8; CH10/11 -> buttons 0/1; button 2 is reserved.
This is a separate device/profile, never an impersonation of the physical Pocket.
"""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[2])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import fcntl
import os
from pathlib import Path
import struct

NAME = 'Pocket Virtual UAV AETR'
INPUT_EVENT = struct.Struct('@llHHi')
EV_KEY, EV_ABS = 1, 3
BUTTON_BASE = 0x120
AXES = tuple(range(9))


class VirtualJoystick:
    def __init__(self, path='/dev/uinput'):
        self.fd = os.open(path, os.O_WRONLY | os.O_NONBLOCK)
        self.previous = {}
        try:
            fcntl.ioctl(self.fd, 0x40045564, EV_ABS)  # UI_SET_EVBIT
            fcntl.ioctl(self.fd, 0x40045564, EV_KEY)
            for code in AXES:
                fcntl.ioctl(self.fd, 0x40045567, code)  # UI_SET_ABSBIT
            for code in range(BUTTON_BASE, BUTTON_BASE + 3):
                fcntl.ioctl(self.fd, 0x40045565, code)  # UI_SET_KEYBIT
            maxima, minima = [0] * 64, [0] * 64
            for code in AXES:
                maxima[code], minima[code] = 32767, -32767
            descriptor = struct.pack('=80sHHHHI' + 'i' * 256,
                NAME.encode(), 3, 0x1209, 0x5050, 1, 0,
                *maxima, *minima, *([0] * 128))
            os.write(self.fd, descriptor)
            fcntl.ioctl(self.fd, 0x5501)  # UI_DEV_CREATE
            # Bind discovery to this uinput instance (UI_GET_SYSNAME), not a
            # device-name search which could select a second panel's joystick.
            name = bytearray(128)
            fcntl.ioctl(self.fd, 0x8080552c, name, True)
            self.sysname = name.split(b'\0', 1)[0].decode()
        except BaseException:
            self.close()
            raise

    def write(self, channels):
        values = [(EV_ABS, code, round(channels[i] * 32767)) for i, code in enumerate(AXES)]
        values += [(EV_KEY, BUTTON_BASE, int(channels[9] > 0)),
                   (EV_KEY, BUTTON_BASE + 1, int(channels[10] > 0)),
                   (EV_KEY, BUTTON_BASE + 2, 0)]
        payload = bytearray()
        for kind, code, value in values:
            if self.previous.get((kind, code)) != value:
                payload += INPUT_EVENT.pack(0, 0, kind, code, value)
        if payload:
            payload += INPUT_EVENT.pack(0, 0, 0, 0, 0)  # SYN_REPORT
            if os.write(self.fd, payload) != len(payload):
                raise OSError('Incomplete virtual joystick frame')
            for kind, code, value in values:
                self.previous[kind, code] = value

    def device_path(self):
        paths = sorted((Path('/sys/devices/virtual/input') / self.sysname).glob('js*'))
        return str(Path('/dev/input') / paths[0].name) if paths else None

    def close(self):
        if self.fd is not None:
            try:
                fcntl.ioctl(self.fd, 0x5502)  # UI_DEV_DESTROY: real input loss
            except OSError:
                pass
            finally:
                os.close(self.fd)
                self.fd = None
