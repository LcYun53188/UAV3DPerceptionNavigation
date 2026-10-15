#!/usr/bin/env python3
"""Owned uinput stimulus for explicit computer-only flight acceptance cases."""
import argparse
import json
import math
import signal
import time
from pathlib import Path
from pocket_virtual_joystick import VirtualJoystick


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session',type=Path);args=parser.parse_args()
    session=args.session.resolve()
    manifest=json.loads((session/'manifest.json').read_text())
    if manifest.get('flight_case') not in ('pilot-intervention','local-land-manual','local-land-position'):
        raise ValueError('Explicit owned computer-only acceptance case required')
    if any(p.read_text().strip()=='Pocket Virtual UAV AETR' for p in Path('/sys/class/input').glob('js*/device/name')):
        raise ValueError('Close the interactive virtual Pocket before automated input acceptance')
    stopping=False
    def stop(*_):
        nonlocal stopping
        stopping=True
    signal.signal(signal.SIGTERM,stop);signal.signal(signal.SIGINT,stop)
    device=VirtualJoystick();channels=[0.,0.,-1.,0.,-1.,-1.,-1.,-1.,-1.,-1.,-1.]+[0.]*5
    try:
        while not stopping:
            try:
                command=json.loads((session/'validation-input.json').read_text())
                values=command['channels']
                if len(values)!=16 or any(not math.isfinite(v) or not -1<=v<=1 for v in values):
                    raise ValueError('Invalid virtual input frame')
                channels=values
            except FileNotFoundError:pass
            device.write(channels)
            time.sleep(.02)
    finally:device.close()


if __name__=='__main__':main()
