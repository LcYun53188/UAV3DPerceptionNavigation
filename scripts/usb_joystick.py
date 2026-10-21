#!/usr/bin/env python3
"""Read Linux USB joystick events; inspection is the default, no flight output."""
import argparse
import array
import fcntl
import json
import os
from pathlib import Path
import select
import struct
import time

EVENT = struct.Struct('<IhBB')
INIT, BUTTON, AXIS = 0x80, 1, 2


def qgc_environment(environment):
    """Keep physical Pocket axes signed instead of using SDL trigger scaling.

    QGC exposes these four signed axes followed by unmapped USB axes 4..7.
    Existing QGC calibration must be redone after changing this mapping.
    """
    result = dict(environment)
    mapping = ('0300fcb609120000544f000011010000,EdgeTX Radiomaster Pocket Joystick,'
               'a:b0,b:b1,x:b3,y:b4,back:b10,guide:b12,start:b11,'
               'leftstick:b13,rightstick:b14,leftshoulder:b6,rightshoulder:b7,'
               'leftx:a0,lefty:a1,rightx:a2,righty:a3,platform:Linux,')
    existing = result.get('SDL_GAMECONTROLLERCONFIG', '').strip()
    opentx_mapping = mapping.replace('0300fcb609120000544f000011010000',
                                    '0300bf1e09120000544f000011010000').replace('EdgeTX', 'OpenTX')
    result['SDL_GAMECONTROLLERCONFIG'] = '\n'.join(filter(None, (existing, mapping, opentx_mapping)))
    return result


def devices():
    result = []
    for device in sorted(Path('/dev/input').glob('js*')):
        name = Path('/sys/class/input') / device.name / 'device/name'
        result.append(dict(path=str(device), name=name.read_text().strip() if name.exists() else '',
                           readable=os.access(device, os.R_OK)))
    return result


class Joystick:
    def __init__(self, path):
        self.fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
        axes, buttons = array.array('B', [0]), array.array('B', [0])
        try:
            fcntl.ioctl(self.fd, 0x80016a11, axes, True)
            fcntl.ioctl(self.fd, 0x80016a12, buttons, True)
        except BaseException:
            os.close(self.fd)
            raise
        self.axes, self.buttons = axes[0], buttons[0]

    def read(self, timeout=0.):
        if not select.select([self.fd], [], [], timeout)[0]:
            return None
        data = os.read(self.fd, EVENT.size)
        if len(data) != EVENT.size:
            raise OSError('Joystick disconnected or incomplete event')
        _, value, kind, index = EVENT.unpack(data)
        return dict(kind='axis' if kind & AXIS else 'button', index=index,
                    value=value, initial=bool(kind & INIT))

    def close(self):
        os.close(self.fd)


class SwitchEdges:
    """Stable, release-before-use edges. Initial/held/repeated values never command."""
    def __init__(self, mapping, debounce_s=.15):
        if not isinstance(mapping, list) or not mapping:
            raise ValueError('An explicit, nonempty mapping list is required')
        self.mapping, self.state = [], {}
        seen = set()
        for item in mapping:
            kind, index, value, command = (item[k] for k in ('kind', 'index', 'value', 'command'))
            if kind not in ('axis', 'button') or type(index) is not int or not 0 <= index <= 255:
                raise ValueError('Invalid joystick kind/index')
            if type(value) is not int or not -32767 <= value <= 32767 or (kind == 'button' and value != 1):
                raise ValueError('Invalid switch value')
            if command not in ('START', 'PAUSE', 'RESUME', 'CANCEL', 'RETURN', 'RESUME_MISSION', 'END_MISSION') or (kind, index, value) in seen:
                raise ValueError('Invalid or conflicting command mapping')
            seen.add((kind, index, value))
            self.mapping.append(dict(item))
        self.debounce_s = debounce_s

    def receive(self, event, now):
        key = event['kind'], event['index']
        previous = self.state.get(key)
        if previous is None or previous['value'] != event['value']:
            # A value seen on attach becomes a baseline, not a new press.
            self.state[key] = dict(value=event['value'], since=now,
                                   emitted=previous is None or event['initial'],
                                   previous=previous['value'] if previous else event['value'])

    def ready(self, now):
        commands = []
        for item in self.mapping:
            state = self.state.get((item['kind'], item['index']))
            if (state and not state['emitted'] and state['value'] == item['value'] and
                    now-state['since'] >= self.debounce_s):
                state['emitted'] = True
                commands.append(item['command'])
        # Stop preempts simultaneous resume/start. Never queue motion behind stop.
        if 'CANCEL' in commands:
            return ['CANCEL']
        if 'END_MISSION' in commands:
            return ['END_MISSION']
        return commands[:1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device')
    parser.add_argument('--duration', type=float, default=30.)
    args = parser.parse_args()
    print(json.dumps(dict(devices=devices()), ensure_ascii=False), flush=True)
    if not args.device:
        return 0 if devices() else 2
    if not 0 < args.duration <= 3600:
        parser.error('duration must be within (0,3600]')
    joystick = Joystick(args.device)
    try:
        print(json.dumps(dict(axes=joystick.axes, buttons=joystick.buttons)), flush=True)
        until = time.monotonic()+args.duration
        while time.monotonic() < until:
            event = joystick.read(.1)
            if event is not None:
                print(json.dumps(event), flush=True)
    finally:
        joystick.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
