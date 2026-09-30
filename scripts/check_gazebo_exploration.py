#!/usr/bin/env python3
"""Online exploration regression for uav_ego_lab (sends a simulation goal).

Uses independent lab obstacle geometry and captures task state, map growth,
segment count, and the stopped position after terminal success/failure.
"""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_nav_interfaces.msg import MapSnapshot, TimedTrajectory
from uav_nav_sim.core import grid_from_message, spline


def clearance(p):
    return min(math.hypot(p[0], p[1])-.45, 4.51-abs(p[0]),
               4.21-abs(p[1]), p[2])-.3


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', nargs=3, type=float, default=[3., 0., 1.2])
    parser.add_argument('--expect', choices=['REACHED', 'BLOCKED'], default='REACHED')
    parser.add_argument('--timeout', type=float, default=180.)
    parser.add_argument('--min-segments', type=int, default=1)
    parser.add_argument('--output', default='/tmp/uav_exploration_result.json')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('gazebo_exploration_check')
    report = {'states': [], 'events': [], 'trajectories': []}
    latest = {'map': None, 'position': None}
    active = False
    samples = []
    def on_map(msg):
        if msg.valid:
            latest['map'] = msg
    def on_odom(msg):
        p = msg.pose.pose.position
        latest['position'] = [p.x, p.y, p.z]
        if active:
            samples.append(latest['position'])
    def on_state(msg):
        if active and (not report['states'] or report['states'][-1] != msg.data):
            report['states'].append(msg.data)
            print(msg.data, flush=True)
    def on_event(msg):
        if active:
            report['events'].append(msg.data)
    def on_trajectory(msg):
        if not active:
            return
        curve = spline([[p.x,p.y,p.z] for p in msg.control_points], msg.knot_interval)
        duration = float(curve.t[-4])
        truth = min(clearance(p) for p in curve(np.linspace(0, duration, max(100, int(duration*20)))))
        report['trajectories'].append({'id': msg.trajectory_id, 'duration': duration,
            'map_version': msg.map_version, 'endpoint': curve(duration).tolist(), 'truth_clearance': float(truth)})
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(MapSnapshot, '/uav/map/snapshot', on_map, qos)
    node.create_subscription(Odometry, '/uav/localization/odometry', on_odom, qos_profile_sensor_data)
    node.create_subscription(String, '/uav/navigation/state', on_state, qos)
    node.create_subscription(String, '/uav/executor/event', on_event, 10)
    node.create_subscription(TimedTrajectory, '/uav/trajectory', on_trajectory, 10)
    pub = node.create_publisher(PoseStamped, '/uav/goal', 10)
    cancel = node.create_client(Trigger, '/uav/cancel')
    try:
        deadline = time.monotonic()+20
        while time.monotonic() < deadline and (latest['map'] is None or latest['position'] is None or pub.get_subscription_count() == 0):
            rclpy.spin_once(node, timeout_sec=.05)
        if latest['map'] is None or latest['position'] is None:
            raise RuntimeError('No map or odometry')
        grid = grid_from_message(latest['map'])
        report['initial_observed'] = int(np.count_nonzero(grid.observed))
        report['initial_goal_state'] = grid.state(args.goal).name
        report['initial_goal_volume_blocked'] = bool(grid.collision(args.goal, .3))
        report['initial_start_blocked'] = bool(grid.collision(latest['position'], .3))
        report['start'] = latest['position']
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = args.goal
        msg.pose.orientation.w = 1.
        active = True
        pub.publish(msg)
        deadline = time.monotonic()+args.timeout
        terminal = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if report['states'] and report['states'][-1].split(':')[0] in ('REACHED', 'BLOCKED', 'STOPPED'):
                terminal = report['states'][-1].split(':')[0]
                break
        stopped = np.array(latest['position'])
        after_stop = len(samples)
        accepted_before = sum(e.startswith('ACCEPTED:') for e in report['events'])
        # Continue listening after a terminal state: a late plan must not restart it.
        hold_deadline = time.monotonic()+3
        while time.monotonic() < hold_deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        drift = max((float(np.linalg.norm(np.array(p)-stopped)) for p in samples[after_stop:]), default=0.)
        accepted_after = sum(e.startswith('ACCEPTED:') for e in report['events'])
        report.update(goal=args.goal, terminal=terminal, final_position=latest['position'],
            final_error=float(np.linalg.norm(np.array(latest['position'])-args.goal)),
            final_observed=int(np.count_nonzero(latest['map'].observed)),
            samples=len(samples), min_truth_clearance=min(map(clearance, samples)) if samples else None,
            accepted_segments=accepted_after, hold_drift=drift)
        report['passed'] = bool(terminal == args.expect and samples and
            report['min_truth_clearance'] > 0 and drift < .05 and accepted_before == accepted_after and
            accepted_after >= args.min_segments and
            all(t['truth_clearance'] > 0 for t in report['trajectories']) and
            (args.expect != 'REACHED' or report['final_error'] < .15))
        Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report, indent=2), flush=True)
    finally:
        # Also stop the task if the test times out or raises.
        if cancel.service_is_ready():
            future = cancel.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if report.get('passed') else 1)


if __name__ == '__main__':
    main()
