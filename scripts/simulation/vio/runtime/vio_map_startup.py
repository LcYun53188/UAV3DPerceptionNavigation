"""Ground-only startup stability before enabling depth integration.

This observes original VIO/PX4 samples. It never changes poses, resets a source,
or authorizes flight; the existing source and sensor checks remain mandatory.
"""
from collections import deque
import math
import time


def initialized_window(states, covariance_stamps, ros, writer_valid, window_s=30.):
    """Recent matched SDK evidence for ground startup, not SDK qualification.

    The separate candidate audit still requires 120 s. This window is used
    only together with equally long original VIO/PX4 position stability.
    """
    states = states[-2000:]
    covariance_stamps = covariance_stamps[-2000:]
    checks = dict(writer=bool(writer_valid), present=bool(states and covariance_stamps))
    if not checks['present']:
        return dict(passed=False, window_s=window_s, checks=checks)
    if not all(row and math.isfinite(row[0]) for row in states):
        return dict(passed=False, window_s=window_s, checks=dict(checks, finite_time=False))
    end = min(states[-1][0], covariance_stamps[-1])
    begin = end-window_s*1e9
    rows = [row for row in states if begin <= row[0] <= end]
    stamps = [s for s in covariance_stamps if begin <= s <= end]
    checks['fresh'] = all(-.05 <= ros-t/1e9 <= .2 for t in (states[-1][0], covariance_stamps[-1]))
    checks['initialized'] = bool(rows) and all(
        len(row) == 14 and row[1] == 1 and all(math.isfinite(v) for v in row)
        and 8. <= math.sqrt(sum(v*v for v in row[2:5])) <= 11. for row in rows)
    times = [row[0] for row in rows]
    for name, values in (('state', times), ('covariance', stamps)):
        checks[name+'_continuous'] = bool(len(values) > 1
            and (values[-1]-values[0])/1e9 >= window_s-.1
            and all(0 < b-a <= .2e9 for a, b in zip(values, values[1:])))
    state_times = {round(t) for t in times}
    checks['matched'] = bool(stamps) and all(round(s) in state_times for s in stamps)
    return dict(passed=all(checks.values()), window_s=window_s, checks=checks,
                state_samples=len(rows), covariance_samples=len(stamps))


class MapStartupStability:
    window_s = 10.
    max_position_span_m = .03
    max_angle_rad = math.radians(1.)

    def __init__(self, window_s=10., max_drift_rate_m_s=None):
        if not math.isfinite(window_s) or not 10. <= window_s <= 30.:
            raise ValueError('Startup stability window must be within [10,30] seconds')
        self.window_s = window_s
        if max_drift_rate_m_s is not None and (not math.isfinite(max_drift_rate_m_s) or max_drift_rate_m_s <= 0):
            raise ValueError('Drift rate limit must be finite and positive')
        self.max_drift_rate_m_s = max_drift_rate_m_s
        self.samples = {key: deque(maxlen=4000) for key in ('vio', 'px4')}
        self.local_resets = None

    def observe(self, source, stamp, position, quaternion, mono):
        rows = self.samples[source]
        values = (stamp, *position, *quaternion, mono)
        norm = math.sqrt(sum(v*v for v in quaternion))
        if not all(math.isfinite(v) for v in values) or norm < .5:
            rows.clear()
            return
        if rows and stamp <= rows[-1][0]:
            if stamp == rows[-1][0]:
                return  # A duplicate cannot renew freshness or fill the window.
            rows.clear()
        rows.append((stamp, tuple(position), tuple(v/norm for v in quaternion), mono))
        while rows and rows[0][0] < stamp-self.window_s:
            rows.popleft()

    def result(self, ros, mono, ground):
        sources = {}
        for name, rows in self.samples.items():
            span = rows[-1][0]-rows[0][0] if rows else 0.
            position_span = (math.sqrt(sum((max(r[1][i] for r in rows)-min(r[1][i] for r in rows))**2
                                          for i in range(3))) if rows else None)
            angle = (max(2*math.acos(min(1., abs(sum(a*b for a, b in zip(r[2], rows[0][2])))))
                         for r in rows) if rows else None)
            fresh = bool(rows and -.05 <= ros-rows[-1][0] <= .2 and 0 <= mono-rows[-1][3] <= .2)
            gaps = [b[0]-a[0] for a, b in zip(rows, list(rows)[1:])]
            # Compare two 2 s means separated by ~8 s. Unlike endpoint speed,
            # this tolerates stationary sample noise but detects slow creep
            # that still fits inside the 30 s / 3 cm envelope.
            drift_rate = None
            if rows and span >= 9.9:
                end = rows[-1][0]
                first = [r for r in rows if end-10. <= r[0] <= end-8.]
                last = [r for r in rows if r[0] >= end-2.]
                if first and last:
                    dt = sum(r[0] for r in last)/len(last)-sum(r[0] for r in first)/len(first)
                    drift_rate = math.sqrt(sum((sum(r[1][i] for r in last)/len(last)
                                               -sum(r[1][i] for r in first)/len(first))**2 for i in range(3)))/dt
            drift_stable = (self.max_drift_rate_m_s is None or
                            (drift_rate is not None and drift_rate <= self.max_drift_rate_m_s))
            passed = bool(ground and fresh and span >= self.window_s-.1
                          and gaps and max(gaps) <= .2
                          and position_span <= self.max_position_span_m and angle <= self.max_angle_rad and drift_stable)
            sources[name] = dict(passed=passed, samples=len(rows), span_s=span,
                                 position_span_m=position_span, angle_rad=angle, fresh=fresh,
                                 drift_rate_m_s=drift_rate, drift_stable=drift_stable)
        return dict(passed=bool(ground and all(v['passed'] for v in sources.values())), ground=ground,
                    window_s=self.window_s, max_position_span_m=self.max_position_span_m,
                    max_angle_rad=self.max_angle_rad, max_drift_rate_m_s=self.max_drift_rate_m_s, sources=sources)

    def subscribe(self, node):
        from nav_msgs.msg import Odometry
        from px4_msgs.msg import VehicleLocalPosition
        from rclpy.qos import qos_profile_sensor_data

        def raw(message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            s = message.header.stamp
            self.observe('vio', s.sec+s.nanosec/1e9, (p.x, p.y, p.z), (q.x, q.y, q.z, q.w), time.monotonic())

        def local(message):
            resets = (message.xy_reset_counter, message.z_reset_counter, message.heading_reset_counter)
            if resets != self.local_resets:
                self.samples['px4'].clear()
                self.local_resets = resets
            yaw = message.heading/2
            self.observe('px4', message.timestamp/1e6, (message.x, message.y, message.z),
                         (0., 0., math.sin(yaw), math.cos(yaw)), time.monotonic())

        self.subscriptions = [
            node.create_subscription(Odometry, '/visual_slam/tracking/odometry', raw, qos_profile_sensor_data),
            node.create_subscription(VehicleLocalPosition, '/px4_7/fmu/out/vehicle_local_position', local, qos_profile_sensor_data)]

    def close(self, node):
        for subscription in self.subscriptions:
            node.destroy_subscription(subscription)


class MapStartupReady:
    """Observe the promoted map binding; never manufacture or publish readiness."""
    topic = '/planning/source/alignment'

    def __init__(self, session):
        self.session = session
        self.rows = deque(maxlen=50)
        self.writer = None

    def observe(self, message, mono):
        stamp = message.header.stamp.sec+message.header.stamp.nanosec/1e9
        identity = (message.map_id, message.map_epoch, message.generation, tuple(message.reset_counters))
        valid = (message.valid and message.header.frame_id == 'map'
                 and message.localization_session == self.session
                 and message.alignment_id == 'depth-map-is-ekf-odom-v1'
                 and message.map_id and message.map_epoch > 0 and message.generation > 0
                 and math.isfinite(stamp) and math.isfinite(mono))
        if not valid:
            self.rows.clear()
            return
        if self.rows and (identity != self.rows[-1][2] or not 0 < stamp-self.rows[-1][0] <= .2):
            self.rows.clear()
        self.rows.append((stamp, mono, identity))

    def result(self, ros, mono, admitted_ros, unique_writer):
        rows = [r for r in self.rows if r[0] >= admitted_ros]
        fresh = bool(rows and -.05 <= ros-rows[-1][0] <= .2 and 0 <= mono-rows[-1][1] <= .25)
        continuous = bool(len(rows) >= 3 and rows[-1][0]-rows[0][0] >= 1.
                          and all(0 < b[1]-a[1] <= .25 for a, b in zip(rows, rows[1:])))
        return dict(passed=bool(unique_writer and fresh and continuous), fresh=fresh,
                    continuous=continuous, unique_writer=unique_writer, samples=len(rows),
                    map_id=rows[-1][2][0] if rows else None, scope='Fresh same-session promoted map binding')

    def subscribe(self, node):
        from uav_nav_interfaces.msg import LocalizationAlignment
        self.subscription = node.create_subscription(LocalizationAlignment, self.topic,
            lambda message: self.observe(message, time.monotonic()), 10)

    def unique(self, node):
        endpoints = node.get_publishers_info_by_topic(self.topic)
        if len(endpoints) != 1:
            self.rows.clear()
            return False
        writer = bytes(endpoints[0].endpoint_gid)
        if self.writer is not None and writer != self.writer:
            self.rows.clear()
            return False
        self.writer = writer
        return True

    def close(self, node):
        node.destroy_subscription(self.subscription)
