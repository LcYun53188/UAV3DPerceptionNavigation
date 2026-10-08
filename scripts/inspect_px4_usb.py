#!/usr/bin/env python3
"""Passive USB MAVLink capture. Never sends heartbeat, requests or commands."""
import argparse
from collections import Counter
import fcntl
import json
import os
from pathlib import Path
import select
import termios
import time

from pymavlink.dialects.v20 import common as mavlink

TYPES = {'HEARTBEAT', 'SYS_STATUS', 'EXTENDED_SYS_STATE', 'RC_CHANNELS',
         'AUTOPILOT_VERSION', 'STATUSTEXT', 'BATTERY_STATUS'}


def capture(device, duration):
    if not 0 < duration <= 60:
        raise ValueError('duration must be 0..60 seconds')
    device = Path(device).resolve(strict=True)
    # Refuse an existing reader; TIOCEXCL alone cannot evict an already open FD.
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            descriptors = list((proc / 'fd').iterdir())
        except (PermissionError, FileNotFoundError):
            continue
        for descriptor in descriptors:
            try:
                if descriptor.resolve() == device:
                    raise RuntimeError(f'{device} already open by PID {proc.name}; close QGC/serial readers')
            except (FileNotFoundError, PermissionError):
                continue
    fd = os.open(device, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    saved = None
    exclusive = False
    counts, vehicles, status_text = Counter(), {}, []
    raw_bytes = 0
    parser = mavlink.MAVLink(None)
    parser.robust_parsing = True
    started = time.monotonic()
    try:
        fcntl.ioctl(fd, termios.TIOCEXCL)
        exclusive = True
        saved = termios.tcgetattr(fd)
        settings = termios.tcgetattr(fd)
        settings[0] = settings[1] = settings[3] = 0
        settings[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        settings[4] = settings[5] = termios.B115200
        settings[6][termios.VMIN] = settings[6][termios.VTIME] = 0
        termios.tcsetattr(fd, termios.TCSANOW, settings)
        while time.monotonic() - started < duration:
            ready, _, _ = select.select([fd], [], [], min(.2, max(0, duration-(time.monotonic()-started))))
            if not ready:
                continue
            try:
                data = os.read(fd, 65536)
            except BlockingIOError:
                continue
            if not data:
                raise RuntimeError('USB disconnected or serial stream closed')
            raw_bytes += len(data)
            for message in parser.parse_buffer(data) or []:
                kind = message.get_type()
                counts[kind] += 1
                if kind not in TYPES:
                    continue
                key = f'{message.get_srcSystem()}:{message.get_srcComponent()}'
                vehicle = vehicles.setdefault(key, {})
                vehicle[kind] = message.to_dict()
                if kind == 'STATUSTEXT' and len(status_text) < 100:
                    status_text.append(dict(source=key, **message.to_dict()))
    finally:
        try:
            if saved is not None:
                termios.tcsetattr(fd, termios.TCSANOW, saved)
        finally:
            try:
                if exclusive:
                    fcntl.ioctl(fd, termios.TIOCNXCL)
            finally:
                os.close(fd)
    autopilots = []
    for source, messages in vehicles.items():
        hb = messages.get('HEARTBEAT', {})
        if hb.get('autopilot') == mavlink.MAV_AUTOPILOT_PX4:
            autopilots.append(dict(source=source,
                armed=bool(hb['base_mode'] & mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
                system_status=hb['system_status'], custom_mode=hb['custom_mode']))
    return dict(result='OBSERVED' if autopilots else 'INCONCLUSIVE', device=str(device),
                receive_only=True, transmitted_bytes=0, duration_s=time.monotonic()-started,
                raw_bytes=raw_bytes, message_counts=dict(counts), autopilots=autopilots,
                latest=vehicles, status_text=status_text,
                scope='Passive telemetry only; no parameter changes or hardware flight validation')


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--device', type=Path, required=True)
    cli.add_argument('--duration', type=float, default=10)
    cli.add_argument('--output', type=Path, required=True)
    args = cli.parse_args()
    try:
        result = capture(args.device, args.duration)
    except (OSError, RuntimeError, ValueError) as error:
        result = dict(result='ERROR', receive_only=True, transmitted_bytes=0, error=str(error))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    return 0 if result['result'] == 'OBSERVED' else 2


if __name__ == '__main__':
    raise SystemExit(main())
