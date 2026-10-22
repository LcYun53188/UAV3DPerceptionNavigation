"""Pocket SA/SB/SD stable edges, bound to the inspected USB axes."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[2])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import time


class PocketSwitches:
    AXES = {'arm': 4, 'navigation': 5, 'run': 7}
    # Buttons 1 is a latching wheel, not a momentary return key. Buttons 0
    # remains unassigned until its function and Linux input mapping are chosen.
    def __init__(self, return_button=None):
        if return_button is not None and (type(return_button) is not int or return_button != 0):
            raise ValueError('Only the inspected momentary button 0 may be enabled; wheel 1 is not a key')
        self.return_button = return_button
        self.raw, self.levels = {}, {}

    def receive(self, event, now):
        if event['kind'] == 'axis' and event['index'] in self.AXES.values():
            key, high = event['index'], event['value'] > 20000
        elif event['kind'] == 'button' and event['index'] == self.return_button:
            key, high = 256+event['index'], bool(event['value'])
        else:
            return
        old = self.raw.get(key)
        if old is None or old[0] != high:
            self.raw[key] = high, now
        if event['initial']:
            self.levels[key] = high  # Attach/held state never creates a command.

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        changed = set()
        for key, (high, since) in self.raw.items():
            if key not in self.levels:
                self.levels[key] = high
            elif self.levels[key] != high and now-since >= .15:
                self.levels[key] = high
                changed.add(key)
        if not all(key in self.levels for key in self.AXES.values()):
            return []
        arm, nav, run = (self.levels[self.AXES[name]] for name in ('arm', 'navigation', 'run'))
        # Loss of arm or navigation wins; do not dispatch START in the same tick.
        if self.AXES['arm'] in changed and not arm:
            return ['DISARM']
        if self.AXES['navigation'] in changed and not nav:
            return ['MANUAL']
        commands = []
        if self.AXES['arm'] in changed and arm:
            commands.append('ARM')
        if (self.return_button is not None and 256+self.return_button in changed
                and self.levels[256+self.return_button]):
            commands.append('TOGGLE_RETURN' if arm and nav else 'BLOCKED_RUN')
            return commands
        if self.AXES['run'] in changed:
            commands.append('RUN' if run and nav and arm else 'PAUSE' if not run else 'BLOCKED_RUN')
        return commands

    @property
    def arm(self):
        return self.levels.get(self.AXES['arm'], False)

    @property
    def navigation(self):
        return self.levels.get(self.AXES['navigation'], False)
