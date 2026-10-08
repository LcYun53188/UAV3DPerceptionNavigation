"""Fail-closed VIO admission using source health AND successful EKF aid samples.

EstimatorStatusFlags.cs_ev_* expresses intent and is insufficient on its own.
Default PX4 DDS does not export aid sources; missing telemetry means NOT READY.
"""
import math


class VioGate:
    required = ('source', 'flags', 'selector', 'ev_pos', 'ev_hgt', 'ev_vel', 'ev_yaw')

    def __init__(self, calibration_id, stable_s=2.):
        self.calibration_id = calibration_id
        self.stable_s = stable_s
        self.samples = {}
        self.received = {}
        self.stable_since = None
        self.bound_identity = None
        self.candidate_identity = None
        self.bound_selector_count = None
        self.fault = ''
        self.reason = 'VIO_MISSING'
        self.last_clock = None
        self.clock_advance = None

    def receive(self, name, message, now):
        # Observe identity/tracking loss in the callback, even if a newer message
        # arrives before the next control tick. A bound mission must not auto-recover.
        if name == 'source' and self.bound_identity is not None:
            identity = (message.localization_session, message.reset_counter, message.calibration_id)
            if identity != self.bound_identity:
                self.fault = 'VIO_SESSION_CHANGED'
            elif not message.valid:
                self.fault = 'VIO_SOURCE_LOST'
        if name == 'selector' and self.bound_selector_count is not None:
            if message.primary_instance != 0 or message.instance_changed_count != self.bound_selector_count:
                self.fault = 'VIO_EKF_INSTANCE_CHANGED'
        self.samples[name] = message
        self.received[name] = now

    @staticmethod
    def stamp(stamp):
        return stamp.sec + stamp.nanosec / 1e9

    def fail(self, reason):
        self.stable_since = None
        self.reason = reason
        return False

    def ready(self, now, ros, writers):
        if self.last_clock is not None and ros < self.last_clock:
            self.fault = 'VIO_CLOCK_RESET'
        if ros > 0 and (self.last_clock is None or ros > self.last_clock):
            self.clock_advance = now
        self.last_clock = ros
        if self.clock_advance is not None and now-self.clock_advance > .5:
            self.fault = 'VIO_CLOCK_STALLED'
        if self.fault:
            return self.fail(self.fault)
        if any(writers.get(n) != 1 for n in self.required):
            return self.fail('VIO_TELEMETRY_WRITER_COUNT')
        if any(n not in self.samples for n in self.required):
            return self.fail('VIO_TELEMETRY_MISSING')
        source = self.samples['source']
        for name in self.required:
            message = self.samples[name]
            stamp = self.stamp(message.header.stamp) if name == 'source' else message.timestamp/1e6
            # Flags and selector publish at ~1 Hz or on changes in pinned PX4 1.16.2.
            max_age = 1.5 if name in ('flags', 'selector') else .5
            if not (0 <= now-self.received[name] <= max_age and stamp > 0 and -.05 <= ros-stamp <= max_age):
                return self.fail('VIO_TELEMETRY_STALE:'+name)
        if (not source.valid or source.header.frame_id != 'odom' or not source.localization_session
                or source.calibration_id != self.calibration_id or not self.calibration_id
                or not -.05 <= ros-self.stamp(source.sample_stamp) <= .2):
            return self.fail('VIO_SOURCE_INVALID')
        identity = (source.localization_session, source.reset_counter, source.calibration_id)
        if self.bound_identity is not None and identity != self.bound_identity:
            self.fault = 'VIO_SESSION_CHANGED'
            return self.fail(self.fault)
        if identity != self.candidate_identity:
            self.candidate_identity = identity
            self.stable_since = None
        selector = self.samples['selector']
        if (selector.primary_instance != 0 or selector.instances_available < 1 or not selector.healthy[0]
                or selector.gyro_fault_detected or selector.accel_fault_detected):
            return self.fail('VIO_EKF_SELECTOR_INVALID')
        if self.bound_selector_count is not None and selector.instance_changed_count != self.bound_selector_count:
            self.fault = 'VIO_EKF_INSTANCE_CHANGED'
            return self.fail(self.fault)
        flags = self.samples['flags']
        if (not all(getattr(flags, 'cs_'+n) for n in ('ev_pos','ev_hgt','ev_vel','ev_yaw'))
                or any(getattr(flags, n) for n in ('cs_ev_yaw_fault','cs_inertial_dead_reckoning',
                        'cs_fake_pos','cs_fake_hgt','reject_hor_pos','reject_ver_pos','reject_hor_vel',
                        'reject_ver_vel','reject_yaw','fs_bad_hdg','fs_bad_acc_vertical','fs_bad_acc_clipping'))):
            return self.fail('VIO_EKF_FLAGS_INVALID')
        for name in ('ev_pos', 'ev_hgt', 'ev_vel', 'ev_yaw'):
            aid = self.samples[name]
            ratios = aid.test_ratio if hasattr(aid.test_ratio, '__len__') else [aid.test_ratio]
            if (aid.estimator_instance != 0 or not aid.fused or aid.innovation_rejected
                    or aid.timestamp_sample <= 0 or aid.time_last_fuse <= 0
                    or not -.05 <= ros-aid.timestamp_sample/1e6 <= .5
                    or not -.05 <= ros-aid.time_last_fuse/1e6 <= .5
                    or abs(aid.timestamp_sample/1e6-self.stamp(source.sample_stamp)) > .2
                    or not all(math.isfinite(float(v)) and 0 <= v < 1 for v in ratios)):
                return self.fail('VIO_EKF_NOT_FUSED:'+name)
        if self.stable_since is None:
            self.stable_since = now
        if now-self.stable_since < self.stable_s:
            self.reason = 'VIO_STABILIZING'
            return False
        self.bound_identity = identity
        self.bound_selector_count = selector.instance_changed_count
        self.reason = 'READY'
        return True
