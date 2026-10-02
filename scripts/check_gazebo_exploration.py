#!/usr/bin/env python3
"""Online exploration regression for lab/expanded (sends a simulation goal).

Uses independent world obstacle geometry and captures task state, map growth,
segment count, and the stopped position after terminal success/failure.
"""
import argparse
import json
import math
from pathlib import Path
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.duration import Duration
from std_msgs.msg import String
from std_srvs.srv import Trigger
from uav_nav_interfaces.msg import MapSnapshot, TimedTrajectory
from uav_nav_sim.core import grid_from_message, spline


def clearance(p):
    return min(math.hypot(p[0], p[1])-.45, 4.51-abs(p[0]),
               4.21-abs(p[1]), p[2])-.3


def expanded_clearance():
    """Independent distances to the expanded world's static collision shapes."""
    world_path = Path(__file__).resolve().parents[1]/'src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf'
    world = ET.parse(world_path).getroot().find('world')
    shapes = []
    for model in world.findall('model'):
        pose = list(map(float, model.findtext('pose', '0 0 0 0 0 0').split()))
        if pose[3:5] != [0., 0.]:
            raise ValueError('Validation geometry requires upright static models')
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                if link.find('pose') is not None or collision.find('pose') is not None:
                    raise ValueError('Validation geometry requires model-relative shapes')
                geometry = collision.find('geometry')
                box, cylinder = geometry.find('box'), geometry.find('cylinder')
                if box is not None:
                    shapes.append(('box', pose, list(map(float, box.findtext('size').split()))))
                elif cylinder is not None:
                    shapes.append(('cylinder', pose, [float(cylinder.findtext('radius')),
                                                       float(cylinder.findtext('length'))]))
                elif (geometry.find('plane') is None or pose != [0.]*6 or
                      geometry.findtext('plane/normal') != '0 0 1'):
                    raise ValueError('Unsupported validation collision geometry')

    def distance(point):
        closest = point[2]  # ground plane
        for kind, pose, size in shapes:
            x, y, z = (point[i]-pose[i] for i in range(3))
            c, s = math.cos(pose[5]), math.sin(pose[5])
            x, y = c*x+s*y, -s*x+c*y
            q = ([abs(x)-size[0]/2, abs(y)-size[1]/2, abs(z)-size[2]/2]
                 if kind == 'box' else [math.hypot(x, y)-size[0], abs(z)-size[1]/2])
            signed = math.sqrt(sum(max(v, 0)**2 for v in q))+min(max(q), 0)
            closest = min(closest, signed)
        return closest-.3
    return distance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goal', nargs=3, type=float, default=[3., 0., 1.2])
    parser.add_argument('--layout', choices=['lab', 'expanded'], default='lab')
    parser.add_argument('--expect', choices=['REACHED', 'BLOCKED'], default='REACHED')
    parser.add_argument('--timeout', type=float, default=180.)
    parser.add_argument('--min-segments', type=int, default=1)
    parser.add_argument('--output', default='/tmp/uav_exploration_result.json')
    parser.add_argument('--stop-on-unsafe', action='store_true',
                        help='Cancel immediately when independent geometry rejects a published curve')
    args = parser.parse_args()
    truth_clearance = expanded_clearance() if args.layout == 'expanded' else clearance
    rclpy.init()
    node = rclpy.create_node('gazebo_exploration_check')
    report = {'layout': args.layout, 'goal': args.goal, 'completed': False,
              'states': [], 'events': [], 'trajectories': []}
    latest = {'map': None, 'position': None}
    active = False
    task_started = False
    samples = []
    def checkpoint():
        report['latest_position'] = latest['position']
        path = Path(args.output)
        temporary = path.with_suffix(path.suffix+'.tmp')
        temporary.write_text(json.dumps(report, indent=2)+'\n')
        temporary.replace(path)
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
            checkpoint()
    def on_event(msg):
        nonlocal task_started
        if active:
            report['events'].append(msg.data)
            if msg.data == 'GOAL_REPLACED':
                task_started = True
    def on_trajectory(msg):
        if not active:
            return
        curve = spline([[p.x,p.y,p.z] for p in msg.control_points], msg.knot_interval)
        duration = float(curve.t[-4])
        points = curve(np.linspace(0, duration, max(100, int(duration*20))))
        clearances = np.array([truth_clearance(p) for p in points])
        closest = points[int(np.argmin(clearances))]
        entry = {'id': msg.trajectory_id, 'parent_id': msg.parent_trajectory_id, 'duration': duration,
            'map_version': msg.map_version, 'endpoint': curve(duration).tolist(),
            'truth_clearance': float(np.min(clearances)), 'truth_closest_point': closest.tolist()}
        if latest['map'] is not None:
            grid = grid_from_message(latest['map'])
            index = grid.index(closest)
            entry['receipt_map_version'] = latest['map'].version
            entry['receipt_map_collision_at_closest'] = bool(grid.collision(closest, .3))
            entry['receipt_map_distance_at_closest'] = float(grid.distance[tuple(index)]) if index is not None else None
            lo = np.maximum(0,np.floor((closest-.3-grid.origin)/grid.resolution).astype(int))
            hi = np.minimum(grid.distance.shape,np.floor((closest+.3-grid.origin)/grid.resolution).astype(int)+1)
            region = tuple(slice(a,b) for a,b in zip(lo,hi))
            body_distance = grid.distance[region]
            entry['min_body_distance'] = float(np.min(body_distance))
            entry['nonpositive_body_voxels'] = int(np.count_nonzero(body_distance <= 0))
            entry['unknown_body_voxels'] = int(np.count_nonzero(~grid.observed[region]))
        report['trajectories'].append(entry)
        checkpoint()
        if args.stop_on_unsafe and entry['truth_clearance'] <= 0:
            report['unsafe_abort'] = True
            cancel.call_async(Trigger.Request())
            checkpoint()
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
        if pub.get_subscription_count() == 0:
            raise RuntimeError('No navigation goal subscriber')
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
        if not pub.wait_for_all_acked(Duration(seconds=3.)):
            raise RuntimeError('Navigation goal delivery was not acknowledged')
        started = time.monotonic()
        deadline = started+args.timeout
        terminal = None
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.05)
            if report.get('unsafe_abort'):
                break
            if task_started and report['states'] and report['states'][-1].split(':')[0] in ('REACHED', 'BLOCKED', 'STOPPED'):
                terminal = report['states'][-1].split(':')[0]
                break
        if terminal is None and cancel.service_is_ready():
            future = cancel.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, future, timeout_sec=3.)
        report['duration'] = time.monotonic()-started
        stopped = np.array(latest['position'])
        after_stop = len(samples)
        accepted_before = sum(e.startswith('ACCEPTED:') for e in report['events'])
        # Continue listening after a terminal state: a late plan must not restart it.
        hold_deadline = time.monotonic()+3
        while time.monotonic() < hold_deadline:
            rclpy.spin_once(node, timeout_sec=.05)
        drift = max((float(np.linalg.norm(np.array(p)-stopped)) for p in samples[after_stop:]), default=0.)
        accepted_after = sum(e.startswith('ACCEPTED:') for e in report['events'])
        report.update(completed=True, goal=args.goal, terminal=terminal, final_position=latest['position'],
            final_error=float(np.linalg.norm(np.array(latest['position'])-args.goal)),
            final_observed=int(np.count_nonzero(latest['map'].observed)),
            samples=len(samples), min_truth_clearance=min(map(truth_clearance, samples)) if samples else None,
            accepted_segments=accepted_after, hold_drift=drift)
        report['passed'] = bool(terminal == args.expect and samples and
            report['min_truth_clearance'] > 0 and drift < .05 and accepted_before == accepted_after and
            accepted_after >= args.min_segments and
            all(t['truth_clearance'] > 0 for t in report['trajectories']) and
            (args.expect != 'REACHED' or report['final_error'] < .15))
        checkpoint()
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
