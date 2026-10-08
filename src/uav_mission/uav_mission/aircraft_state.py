"""PX4 facts with independent ROS source and monotonic receive freshness.

This observer is not a control arbiter or a navigation readiness provider.
Clock/source resets latch until a new aggregator instance is created.
"""
from dataclasses import dataclass
import math
import uuid


@dataclass(frozen=True)
class Dimension:
    value: str = 'UNKNOWN'
    valid: bool = False
    reason: str = 'NO_SAMPLE'
    source_time_s: float = -1.0
    source_age_s: float = -1.0
    receive_age_s: float = -1.0


class AircraftStateAggregator:
    SOURCES = ('status', 'land', 'local')

    def __init__(self, max_age_s=.5, clock_stall_s=.5, future_tolerance_s=.05):
        if (not all(math.isfinite(x) for x in
                    (max_age_s, clock_stall_s, future_tolerance_s))
                or min(max_age_s, clock_stall_s) <= 0 or future_tolerance_s < 0):
            raise ValueError('Invalid freshness thresholds')
        self.max_age = max_age_s
        self.clock_stall = clock_stall_s
        self.future_tolerance = future_tolerance_s
        self.instance = str(uuid.uuid4())
        self.sequence = 0
        self.samples = {}
        self.source_faults = {}
        self.clock_fault = None
        self.last_ros = None
        self.last_mono = None
        self.last_advance = None

    def clock(self, ros_s, mono_s):
        if not math.isfinite(ros_s) or not math.isfinite(mono_s):
            self.clock_fault = 'NONFINITE_CLOCK'
            return
        if self.last_mono is not None and mono_s < self.last_mono:
            self.clock_fault = 'MONOTONIC_TIME_RESET'
        if self.last_ros is not None and ros_s < self.last_ros:
            self.clock_fault = 'ROS_TIME_RESET'
        if ros_s > 0:
            if self.last_ros is None or ros_s > self.last_ros:
                self.last_advance = mono_s
            if self.last_advance is not None and mono_s - self.last_advance > self.clock_stall:
                self.clock_fault = 'ROS_TIME_STALLED'
        self.last_ros, self.last_mono = ros_s, mono_s

    def observe(self, source, timestamp_s, data, ros_s, mono_s):
        if source not in self.SOURCES:
            raise ValueError(source)
        self.clock(ros_s, mono_s)
        if self.clock_fault or source in self.source_faults:
            return False
        if (not math.isfinite(timestamp_s) or timestamp_s <= 0
                or ros_s <= 0 or not math.isfinite(ros_s) or not math.isfinite(mono_s)):
            return False
        previous = self.samples.get(source)
        if previous and timestamp_s < previous[0]:
            self.source_faults[source] = 'SOURCE_TIME_RESET'
            return False
        if previous and timestamp_s == previous[0]:
            return False  # Replaying one sample cannot renew receive freshness.
        if source == 'local' and previous and any(
                previous[2].get(k) != data.get(k) for k in
                ('xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter', 'vz_reset_counter', 'heading_reset_counter')):
            self.source_faults[source] = 'LOCAL_POSITION_RESET'
            return False
        if not -self.future_tolerance <= ros_s - timestamp_s <= self.max_age:
            return False
        self.samples[source] = (timestamp_s, mono_s, dict(data))
        return True

    def _dimension(self, source, value, ros_s, mono_s, reason=''):
        sample = self.samples.get(source)
        if sample is None:
            return Dimension(reason=self.clock_fault or 'NO_SAMPLE')
        stamp, receive, _ = sample
        source_age, receive_age = ros_s - stamp, mono_s - receive
        fault = self.clock_fault or self.source_faults.get(source)
        if not fault:
            if not all(math.isfinite(x) for x in (source_age, receive_age)):
                fault = 'NONFINITE_AGE'
            elif source_age < -self.future_tolerance or receive_age < 0:
                fault = 'INVALID_AGE'
            elif source_age > self.max_age or receive_age > self.max_age:
                fault = 'STALE_SAMPLE'
        if fault:
            return Dimension(reason=fault, source_time_s=stamp,
                             source_age_s=source_age, receive_age_s=receive_age)
        return Dimension(value, value not in ('UNKNOWN', 'INVALID'), reason,
                         stamp, source_age, receive_age)

    def snapshot(self, ros_s, mono_s):
        self.clock(ros_s, mono_s)
        self.sequence += 1
        data = lambda key: self.samples.get(key, (None, None, {}))[2]
        status, land, local = (data(key) for key in self.SOURCES)
        dim = lambda key, value, reason='': self._dimension(key, value, ros_s, mono_s, reason)
        arming = {1: 'DISARMED', 2: 'ARMED'}.get(status.get('arming_state'), 'UNKNOWN')
        raw_mode = status.get('nav_state', -1)
        mode = ('OFFBOARD' if raw_mode == 14 else
                'AUTO_LAND' if raw_mode in (18, 20) else
                'PILOT' if raw_mode in (0, 1, 2, 6, 10, 15) else
                'AUTO_OTHER' if raw_mode in (3, 4, 5, 12, 17, 19, 21, 22) else 'UNKNOWN')
        # Conflicting detector/motion facts cannot confirm landing.
        moving = any(land.get(key, False) for key in
                     ('vertical_movement', 'horizontal_movement', 'rotational_movement'))
        ground = ('UNKNOWN' if not land else 'TRANSITION' if land.get('freefall') or (land.get('landed') and moving)
                  or land.get('ground_contact') and not land.get('landed')
                  or land.get('maybe_landed') and not land.get('landed') else
                  'ON_GROUND' if land.get('landed') else 'IN_AIR')
        numeric = all(math.isfinite(local.get(k, math.nan)) for k in ('x', 'y', 'z'))
        quality = all(math.isfinite(local.get(k, math.inf)) and 0 <= local[k] <= 1.0
                      for k in ('eph', 'epv'))
        localization = ('LOCAL_POSITION' if numeric and quality and local.get('xy_valid')
                        and local.get('z_valid') and not local.get('dead_reckoning') else
                        'ALTITUDE' if math.isfinite(local.get('z', math.nan)) and local.get('z_valid')
                        and math.isfinite(local.get('epv', math.inf)) and 0 <= local['epv'] <= 1.0
                        else 'INVALID')
        result = dict(
            arming=dim('status', arming), ground=dim('land', ground), mode=dim('status', mode),
            localization=dim('local', localization, 'PX4_ESTIMATE_ONLY'),
            autopilot_exception=dim('status', 'UNKNOWN' if not status else
                                    'FAILSAFE' if status.get('failsafe') or status.get('failure_detector_status')
                                    else 'NOMINAL'),
            ownership=Dimension(reason='NO_ARBITER'),
            navigation=Dimension('NOT_READY', False, 'NO_MAP_TF_OR_CONTROL_SESSION'),
            battery=Dimension(reason='NO_BATTERY_OBSERVER'))
        link = dim('status', 'CONNECTED')
        if not link.valid and link.reason == 'STALE_SAMPLE':
            age = max(link.source_age_s, link.receive_age_s)
            link = Dimension('LOST' if age > 3 * self.max_age else 'STALE', False,
                             link.reason, link.source_time_s, link.source_age_s, link.receive_age_s)
        result['link'] = link
        result['raw_nav_state'] = raw_mode if link.valid else -1
        result['aggregator_instance'], result['sequence'] = self.instance, self.sequence
        result['reasons'] = sorted({d.reason for d in result.values()
                                    if isinstance(d, Dimension) and d.reason})
        return result
