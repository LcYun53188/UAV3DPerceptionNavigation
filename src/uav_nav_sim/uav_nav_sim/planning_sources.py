"""Bind an owned online map to one localization session and explicit transform.

A reset requires a new map session and a new binder; heartbeat never refreshes
map or odometry source timestamps. This module has no control publishers.
"""
from copy import deepcopy
import math
from .planning_context import stamp, transform_odometry, transform_parts
from uav_nav_interfaces.msg import LocalizationAlignment


class SourceBinding:
    def __init__(self, session, alignment_id, transform, settle_seconds=0.):
        if not session or not alignment_id:
            raise ValueError('EXPLICIT_SOURCE_ID_REQUIRED')
        if not math.isfinite(settle_seconds) or not 0<=settle_seconds<=10:raise ValueError('INVALID_SETTLE_DURATION')
        self.settle_seconds=settle_seconds
        self.pending_resets=None
        self.reset_stable_since=None
        transform_parts(transform)
        self.session, self.alignment_id = session, alignment_id
        self.transform = deepcopy(transform)
        self.inputs = {}
        self.identity = None
        self.retired = ''
        self.reason = 'NO_INPUTS'
        self.activated = False
        self.map_high_water = None
        self.last_clock = None
        self.last_advance = None

    def retire(self, reason):
        self.retired = self.reason = reason

    def update(self, name, message, mono):
        if self.retired:
            return False
        if name not in ('map', 'odom'):
            raise ValueError('UNKNOWN_SOURCE')
        previous = self.inputs.get(name)
        if previous and stamp(message.header.stamp) <= stamp(previous[0].header.stamp):
            # A repeated sample cannot refresh receive freshness.
            return False
        if name == 'map' and self.map_high_water and message.valid:
            version, source = self.map_high_water
            if message.version <= version:
                self.retire('MAP_VERSION_REGRESSION'); return False
            if stamp(message.source_stamp) < source:
                self.retire('MAP_SOURCE_CLOCK_REGRESSION'); return False
        if name == 'map' and previous:
            before = previous[0]
            if (message.map_id, message.epoch) != (before.map_id, before.epoch):
                self.retire('MAP_SESSION_CHANGED'); return False
            if message.valid and before.valid and message.version <= before.version:
                self.retire('MAP_VERSION_REGRESSION'); return False
            if message.valid and stamp(message.source_stamp) < stamp(before.source_stamp):
                self.retire('MAP_SOURCE_CLOCK_REGRESSION'); return False
        if name == 'odom':
            if message.localization_session != self.session:
                self.retire('LOCALIZATION_SESSION_CHANGED'); return False
            if self.identity and tuple(message.reset_counters) != self.identity[2]:
                self.retire('LOCALIZATION_RESET'); return False
        if name == 'map' and message.valid:
            self.map_high_water = (message.version, stamp(message.source_stamp))
        if name=='odom' and tuple(message.reset_counters)!=self.pending_resets:
            self.pending_resets=tuple(message.reset_counters);self.reset_stable_since=mono
        self.inputs[name] = (deepcopy(message), mono)
        return True

    def ready(self, now, mono):
        if self.retired:
            self.reason = self.retired; return False
        if not all(math.isfinite(v) for v in (now, mono)) or now <= 0:
            self.reason = 'CLOCK_UNAVAILABLE'; return False
        if self.activated and self.last_clock is not None and now < self.last_clock:
            self.retire('CLOCK_REGRESSION'); return False
        if not self.activated or self.last_clock is None or now > self.last_clock:
            self.last_advance = mono
        self.last_clock = now
        if self.activated and mono - self.last_advance > .5:
            self.retire('CLOCK_STALL'); return False
        for name, maximum in (('map', 2.), ('odom', .5)):
            if name not in self.inputs:
                self.reason = 'MISSING_' + name.upper(); return False
            message, received = self.inputs[name]
            if not (0 <= mono - received <= maximum and -.05 <= now - stamp(message.header.stamp) <= maximum):
                self.reason = 'STALE_' + name.upper(); return False
        m, o = self.inputs['map'][0], self.inputs['odom'][0]
        if (not m.valid or m.static_map or m.header.frame_id != 'map' or not m.map_id or m.epoch == 0
                or not -.05 <= now - stamp(m.source_stamp) <= 2.):
            self.reason = 'INVALID_OR_STALE_ONLINE_MAP'; return False
        if o.header != o.odometry.header:
            self.retire('ODOMETRY_HEADER_MISMATCH'); return False
        try:
            transform_odometry(o.odometry, self.transform)
        except ValueError as error:
            self.retire(str(error)); return False
        if mono-self.reset_stable_since<self.settle_seconds:
            self.reason='LOCALIZATION_SETTLING';return False
        identity = (m.map_id, m.epoch, tuple(o.reset_counters))
        if self.identity is not None and identity != self.identity:
            self.retire('SOURCE_IDENTITY_CHANGED'); return False
        self.activated = True
        self.identity = identity
        self.reason = 'READY'
        return True

    def alignment(self, now, mono, current_stamp):
        valid = self.ready(now, mono)
        result = LocalizationAlignment(localization_session=self.session,
                                       alignment_id=self.alignment_id, generation=1 if self.identity else 0, valid=valid)
        result.header.frame_id = 'map'; result.header.stamp = current_stamp
        result.map_to_odom = deepcopy(self.transform)
        if self.identity:
            result.map_id, result.map_epoch, counters = self.identity
            result.reset_counters = list(counters)
        return result
