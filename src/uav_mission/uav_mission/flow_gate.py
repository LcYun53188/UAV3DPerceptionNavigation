"""Native flow/range admission without VIO, ROS companion health or EV aids.

This gate belongs to the SITL mission observer. Real manual Position control and
sensor-loss handling run inside PX4 and do not require this Python process.
"""
import math


class FlowGate:
    required = ('flags', 'selector', 'local', 'optical_flow', 'rng_hgt', 'range_sensor', 'flow_sensor')
    pose_only = True
    joint_sim = True
    pure_flow = True

    def __init__(self, stable_s=2.):
        self.samples, self.received = {}, {}
        self.stable_s, self.stable_since = stable_s, None
        self.fault, self.reason = '', 'FLOW_TELEMETRY_MISSING'
        self.identity = self.candidate = None
        self.last_clock = self.clock_advance = None

    def receive(self, name, message, now):
        if name not in self.required:
            return  # VIO lifetime cannot affect the standalone path.
        self.samples[name], self.received[name] = message, now
        if self.identity is not None and name in ('selector', 'local'):
            if self.frame_identity() != self.identity:
                self.fault = 'FLOW_ESTIMATOR_RESET'

    def frame_identity(self):
        s, p = self.samples.get('selector'), self.samples.get('local')
        if s is None or p is None:
            return None
        return (s.primary_instance, s.instance_changed_count) + tuple(getattr(p, n) for n in
            ('xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter', 'vz_reset_counter', 'heading_reset_counter'))

    def fail(self, reason):
        self.reason, self.stable_since = reason, None
        return False

    def ready(self, now, ros, writers, *, native_landing=False):
        if self.last_clock is not None and ros < self.last_clock:
            self.fault = 'FLOW_CLOCK_RESET'
        if ros > 0 and (self.last_clock is None or ros > self.last_clock):
            self.clock_advance = now
        self.last_clock = ros
        if self.clock_advance is not None and now-self.clock_advance > .5:
            self.fault = 'FLOW_CLOCK_STALLED'
        if self.fault:
            return self.fail(self.fault)
        if any(writers.get(n) != 1 for n in self.required):
            return self.fail('FLOW_TELEMETRY_WRITER_COUNT')
        if any(n not in self.samples for n in self.required):
            return self.fail('FLOW_TELEMETRY_MISSING')
        p, f, s = (self.samples[n] for n in ('local', 'flags', 'selector'))
        measured_range = self.samples['range_sensor']
        near_landing = (native_landing and math.isfinite(measured_range.current_distance) and
                        .02 <= measured_range.current_distance <= .4)
        for name in self.required:
            if near_landing and name in ('optical_flow', 'flow_sensor'):
                continue  # Native LAND owns final descent; never admits a mission.
            m = self.samples[name]
            age = 1.5 if name in ('flags', 'selector') else .5
            if not (0 <= now-self.received[name] <= age and m.timestamp > 0 and
                    -.05 <= ros-m.timestamp/1e6 <= age):
                return self.fail('FLOW_TELEMETRY_STALE:'+name)
        if (s.primary_instance != 0 or s.instances_available < 1 or not s.healthy[0] or
                s.gyro_fault_detected or s.accel_fault_detected):
            return self.fail('FLOW_SELECTOR_INVALID')
        if any(getattr(f, n) for n in ('cs_gnss_pos', 'cs_gnss_vel', 'cs_gnss_yaw', 'cs_gps_hgt',
                'cs_ev_pos', 'cs_ev_vel', 'cs_ev_hgt', 'cs_ev_yaw', 'cs_fake_pos', 'cs_fake_hgt',
                'cs_inertial_dead_reckoning', 'reject_ver_pos', 'reject_ver_vel', 'reject_yaw',
                'cs_mag_fault', 'cs_mag_field_disturbed', 'fs_bad_hdg', 'fs_bad_acc_vertical', 'fs_bad_acc_clipping')):
            return self.fail('FLOW_UNEXPECTED_OR_INVALID_AIDING')
        if not f.cs_rng_hgt or not (f.cs_mag_hdg or f.cs_mag_3d) or not (
                f.cs_yaw_align and f.cs_tilt_align and f.cs_mag_heading_consistent):
            return self.fail('FLOW_HEIGHT_OR_HEADING_MISSING')
        if not near_landing and (not f.cs_opt_flow or f.reject_hor_vel or f.reject_hor_pos):
            return self.fail('FLOW_HORIZONTAL_AID_MISSING')
        # PX4's heading_good_for_control means *final in-air* magnetic alignment,
        # not ground yaw alignment. Native manual Position can use aligned yaw
        # before that; resets still revoke this local session immediately.
        if (not all(getattr(p, n) for n in ('z_valid', 'v_z_valid')) or
                not near_landing and (not p.xy_valid or not p.v_xy_valid or p.dead_reckoning) or
                not -.05 <= ros-p.timestamp_sample/1e6 <= .5 or
                not all(math.isfinite(getattr(p, n)) for n in ('x', 'y', 'z', 'vx', 'vy', 'vz',
                    'heading', 'heading_var', 'eph', 'epv', 'evh', 'evv')) or
                not 0 <= p.eph <= 1 or not 0 <= p.epv <= 1 or not 0 < p.heading_var <= .25):
            return self.fail('FLOW_LOCAL_INVALID')
        # dist_bottom_valid describes terrain-state observability. Range-primary
        # height without an absolute position aid may not observe terrain state.
        # Require the actual downward sensor and accepted height aid instead.
        if (measured_range.orientation != 25 or not math.isfinite(measured_range.current_distance) or
                not .02 <= measured_range.current_distance <= 8.):
            return self.fail('FLOW_RANGE_INVALID')
        raw_flow = self.samples['flow_sensor']
        if not near_landing and (raw_flow.quality <= 0 or raw_flow.integration_timespan_us <= 0):
            return self.fail('FLOW_SENSOR_INVALID')
        for name in ('optical_flow', 'rng_hgt'):
            if near_landing and name == 'optical_flow':
                continue
            aid = self.samples[name]
            ratios = aid.test_ratio if hasattr(aid.test_ratio, '__len__') else [aid.test_ratio]
            if (aid.estimator_instance != 0 or aid.innovation_rejected or aid.timestamp_sample <= 0 or
                    aid.time_last_fuse <= 0 or not -.05 <= ros-aid.timestamp_sample/1e6 <= .5 or
                    not -.05 <= ros-aid.time_last_fuse/1e6 <= .5 or
                    not all(math.isfinite(float(v)) and 0 <= v < 1 for v in ratios)):
                return self.fail('FLOW_NOT_FUSED:'+name)
        identity = self.frame_identity()
        if identity != self.candidate:
            self.candidate, self.stable_since = identity, None
        if self.stable_since is None:
            self.stable_since = now
        if now-self.stable_since < self.stable_s:
            self.reason = 'FLOW_STABILIZING'
            return False
        self.identity, self.reason = identity, 'READY'
        return True
