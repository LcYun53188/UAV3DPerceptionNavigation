"""UAV AETR virtual profile, deliberately separate from the legacy USB mapping."""
import time


class PocketUavSwitches:
    def __init__(self):
        self.raw, self.levels = {}, {}
        self.prepared_edge = False

    def receive(self, event, now):
        kind, index, value = event['kind'], event['index'], event['value']
        if kind == 'axis' and index in (4, 5, 6, 7):
            key = index
            level = (1 if value > 20000 else -1 if value < -20000 else 0) if index in (5, 6) else value > 20000
        elif kind == 'button' and index in (0, 1, 2):
            key, level = 256 + index, bool(value)
        else:
            return
        if key not in self.raw or self.raw[key][0] != level:
            self.raw[key] = level, now
        if event['initial']:
            self.levels[key] = level

    def tick(self, now=None):
        now = time.monotonic() if now is None else now
        changed = set()
        for key, (level, since) in self.raw.items():
            if key not in self.levels:
                self.levels[key] = level
            elif self.levels[key] != level and now - since >= .15:
                self.levels[key] = level
                changed.add(key)
        if not all(key in self.levels for key in (4, 5, 6, 7)):
            return []
        if 4 in changed and not self.arm:
            self.prepared_edge = False
            return ['DISARM']
        if 5 in changed:
            self.prepared_edge = False
        # Manual takeover has priority over task pulses. SB position always
        # requests VIO, regardless of the previous SC position.
        if self.main_mode != 1 and (5 in changed or self.main_mode == -1 and 6 in changed):
            return [self.requested_mode]
        if 257 in changed and self.levels[257]:
            self.prepared_edge = False
            return ['CANCEL']
        commands = ['ARM'] if 4 in changed and self.arm else []
        if 7 in changed and self.levels[7]:
            self.prepared_edge = False
            return commands + ['END_MISSION' if self.navigation else 'LAND']
        if self.navigation and (5 in changed or 6 in changed):
            if self.aux_mode == -1:
                self.prepared_edge = True
                commands.append('PREPARE')
            elif self.aux_mode == 0:
                commands.append('RUN' if self.arm and self.prepared_edge else 'BLOCKED_PREPARE')
            else:
                commands.append('INTERVENE')
        if 256 in changed and self.levels[256]:
            return commands + ['TOGGLE_PAUSE']
        return commands

    @property
    def arm(self):
        return bool(self.levels.get(4, False))

    @property
    def main_mode(self):
        return self.levels.get(5, -1)

    @property
    def aux_mode(self):
        return self.levels.get(6, -1)

    @property
    def requested_mode(self):
        if self.main_mode == 0:
            return 'VIO_POSITION'
        return {-1: 'ACRO', 0: 'FLOW_POSITION', 1: 'VIO_POSITION'}[self.aux_mode]

    @property
    def navigation(self):
        return self.main_mode == 1
