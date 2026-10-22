"""Ground-only startup stability before enabling depth integration.

This observes original VIO/PX4 samples. It never changes poses, resets a source,
or authorizes flight; the existing source and sensor checks remain mandatory.
"""
from collections import deque
import math
import time


class MapStartupStability:
    window_s = 10.
    max_position_span_m = .03
    max_angle_rad = math.radians(1.)

    def __init__(self):
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
            passed = bool(ground and fresh and span >= self.window_s-.1
                          and gaps and max(gaps) <= .2
                          and position_span <= self.max_position_span_m and angle <= self.max_angle_rad)
            sources[name] = dict(passed=passed, samples=len(rows), span_s=span,
                                 position_span_m=position_span, angle_rad=angle, fresh=fresh)
        return dict(passed=bool(ground and all(v['passed'] for v in sources.values())), ground=ground,
                    window_s=self.window_s, max_position_span_m=self.max_position_span_m,
                    max_angle_rad=self.max_angle_rad, sources=sources)

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
