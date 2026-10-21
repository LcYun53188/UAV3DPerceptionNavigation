"""UAV Model 5 channel model; independent of Tk, ROS and Linux input."""
import math


CHANNELS = (
    ('Roll', 'I1:Ail'), ('Pitch', 'I2:Ele'), ('Throttle', 'I3:Thr'),
    ('Yaw', 'I4:Rud'), ('Arm', 'SA:L1'), ('Main', 'SB:L2'),
    ('Auxi', 'SC:R2'), ('RTH', 'SD:R1'), ('Speed', 'S1:RB'),
    ('TPaRe', 'L01 / SE:LB'), ('TCanc', 'L02 / SE:LB'),
)


class PocketModel:
    SHORT_MAX = .9
    LONG_MIN = 1.
    PULSE = .5

    @property
    def main_mode(self):
        return {-1: 'MANUAL', 0: 'POSITION', 1: 'AUTO'}[self.values[5]]

    @property
    def requested_mode(self):
        if self.main_mode == 'AUTO':
            return 'AUTO'
        return {-1: 'ASSISTED_POSITION', 0: 'FLOW_POSITION',
                1: 'ALTITUDE' if self.main_mode == 'POSITION' else 'STABILIZED'}[self.values[6]]

    @property
    def sc_labels(self):
        if self.main_mode == 'AUTO':
            return ('起飞准备', '起飞并执行任务', '手动干预')
        if self.main_mode == 'POSITION':
            return ('VIO＋光流定点', '纯光流定点', '仅定高')
        return ('VIO＋光流自稳', '纯光流自稳', '纯姿态自稳')

    def __init__(self):
        self.values = [0., 0., -1., 0., -1., -1., -1., -1., -1., -1., -1.] + [0.] * 5
        self.se_since = None
        self.long_fired = False
        self.pulse_until = {10: float('-inf'), 11: float('-inf')}
        self.last_gesture = '等待 SE 操作'

    def set_channel(self, channel, value):
        if not 1 <= channel <= 9 or not math.isfinite(value):
            raise ValueError('Only finite CH1–CH9 inputs are editable')
        if channel in (5, 8) and value not in (-1, 1):
            raise ValueError('SA/SD require two positions')
        if channel in (6, 7) and value not in (-1, 0, 1):
            raise ValueError('SB/SC require three positions')
        self.values[channel - 1] = max(-1., min(1., value))

    def press_se(self, now):
        if self.se_since is None:
            self.se_since, self.long_fired = now, False

    def release_se(self, now):
        if self.se_since is None:
            return
        self.tick(now)  # A delayed GUI frame still classifies >=1 s as long.
        duration = now - self.se_since
        if not self.long_fired and 0 <= duration <= self.SHORT_MAX + 1e-9:
            self.pulse_until[10] = now + self.PULSE
            self.last_gesture = f'短按 {duration:.2f}s → CH10 暂停 / 继续'
        elif not self.long_fired:
            self.last_gesture = f'{duration:.2f}s → 阈值间隙，无命令'
        self.se_since = None
        self.tick(now)

    def release_controls(self):
        """Focus loss must not synthesize a short press or cut throttle."""
        self.se_since = None
        for channel in (1, 2, 4):
            self.values[channel - 1] = 0.

    def tick(self, now):
        if self.se_since is not None and not self.long_fired and now - self.se_since >= self.LONG_MIN:
            self.long_fired = True
            self.pulse_until[11] = self.se_since + self.LONG_MIN + self.PULSE
            self.last_gesture = '长按 ≥1.00s → CH11 取消（每次按住只触发一次）'
        for channel, until in self.pulse_until.items():
            self.values[channel - 1] = 1. if now < until else -1.
        return tuple(self.values)

    def snapshot(self, now):
        return dict(model='UAV', model_number=5, profile='uav-aetr-v1',
                    main_mode=self.main_mode, requested_mode=self.requested_mode,
                    auxiliary_action=self.sc_labels[int(self.values[6]) + 1],
                    channels={f'CH{i+1}': value for i, value in enumerate(self.tick(now))},
                    scope='software channel simulation; no RF or EdgeTX firmware emulation')
