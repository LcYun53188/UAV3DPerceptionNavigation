#!/usr/bin/env python3
"""Ordinary Action-client regression for the algorithm velocity-model profile.

This is not a BT or PX4 flight test. Raw Gazebo odometry independently checks
motion and the final hold; filtered executor odometry is also recorded.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import uuid

from action_msgs.msg import GoalStatus
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from uav_nav_interfaces.action import NavigateToPose3D
from uav_nav_interfaces.msg import MapSnapshot
from sim_control import CACHE, lock, read_session


def xyz(msg):
    p = msg.pose.pose.position
    return [p.x, p.y, p.z]


def distance(a, b):
    return math.dist(a, b)


def clearance(p):
    # Independent geometry for gazebo/worlds/uav_ego_lab.sdf, body radius .3 m.
    return min(math.hypot(p[0], p[1])-.45, 4.51-abs(p[0]),
               4.21-abs(p[1]), p[2])-.3


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--waypoint', nargs=3, type=float, action='append')
    parser.add_argument('--cancel', action='store_true')
    parser.add_argument('--timeout', type=float, default=90.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.timeout <= 600 or not math.isfinite(args.timeout):
        parser.error('timeout must be finite and 1..600')
    waypoints = args.waypoint or [[-2.2, .6, 1.2], [-3.2, -.6, 1.2]]
    if args.cancel:
        waypoints = waypoints[:1]
    args.output.mkdir(parents=True, exist_ok=True)
    root = Path(__file__).resolve().parents[1]
    session = read_session()
    if (session['layout'] != 'lab' or session['mode'] != 'mapping' or
            int(os.environ.get('ROS_DOMAIN_ID', '0')) != session['domain'] or
            os.environ.get('GZ_PARTITION') != session['partition']):
        parser.error('Require the managed lab/mapping session and matching domain/partition')
    world = root/'src/uav_bringup/gazebo/worlds/uav_ego_lab.sdf'
    world_hash = hashlib.sha256(world.read_bytes()).hexdigest()
    if world_hash != '343d0cef432f45b0bbd854af71104a8908ee534de5c2bcb96eeef3dfc34f0424':
        parser.error('Lab geometry changed; review the independent clearance oracle')
    source_hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest()
        for p in [Path(__file__).resolve(), root/'src/uav_nav_sim/uav_nav_sim/navigation_action.py',
                  root/'src/uav_nav_sim/uav_nav_sim/executor.py', root/'src/uav_nav_sim/uav_nav_sim/navigation.py',
                  root/'src/uav_nav_sim/uav_nav_sim/autonomous.py']}
    rclpy.init()
    node = rclpy.create_node('navigation_action_check', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    client = ActionClient(node, NavigateToPose3D, '/uav/algorithm/navigate')
    state = {'map': None, 'truth': None, 'odom': None}
    samples, feedback, results = [], [], []
    begun = time.monotonic()
    active = None
    trace = (args.output/'trace.jsonl').open('w')

    def record(kind, value):
        value = {'t': time.monotonic()-begun, 'kind': kind, **value}
        trace.write(json.dumps(value, allow_nan=False)+'\n')
        trace.flush()
        return value

    def truth(msg):
        state['truth'] = xyz(msg)
        sample = record('truth', {'position': xyz(msg), 'stamp':
            msg.header.stamp.sec+msg.header.stamp.nanosec/1e9})
        samples.append(sample)

    def odom(msg):
        state['odom'] = xyz(msg)
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        record('odom', {'position': xyz(msg), 'speed': math.hypot(v.x, v.y, v.z),
                        'angular_speed': math.hypot(w.x, w.y, w.z)})

    def progress(msg):
        f = msg.feedback
        feedback.append(record('feedback', {
            'uuid': bytes(msg.goal_id.uuid).hex(), 'phase': f.status.phase,
            'sequence': f.status.event_sequence, 'remaining_distance_m': f.remaining_distance_m,
            'segments': f.segment_count, 'map_session': f.status.map_session}))

    node.create_subscription(Odometry, '/visual_slam/tracking/odometry', truth, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/uav/localization/odometry', odom, qos_profile_sensor_data)
    node.create_subscription(MapSnapshot, '/uav/map/snapshot', lambda m: state.update(map=m),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def spin_until(predicate, timeout):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.02)
        return predicate()

    report = {'profile': 'algorithm', 'client': 'ordinary_action_client',
              'mock': False, 'cancel': args.cancel, 'waypoints': waypoints, 'passed': False,
              'managed_session': session, 'world_sha256': world_hash}
    try:
        if not client.wait_for_server(timeout_sec=5.) or not spin_until(
                lambda: state['truth'] is not None and state['odom'] is not None and
                state['map'] is not None and state['map'].valid, 5.):
            raise RuntimeError('Action server, valid map and Gazebo odometry required')
        report['command_publishers'] = [
            info.node_namespace.rstrip('/')+'/'+info.node_name
            for info in node.get_publishers_info_by_topic('/cmd_vel')]
        if len(report['command_publishers']) != 1:
            raise RuntimeError('Require exactly one cmd_vel publisher')
        report['initial_truth'] = state['truth']
        for index, point in enumerate(waypoints):
            goal = NavigateToPose3D.Goal()
            goal.goal.header.frame_id = 'map'
            goal.goal.header.stamp = node.get_clock().now().to_msg()
            goal.goal.pose.position.x, goal.goal.pose.position.y, goal.goal.pose.position.z = point
            goal.goal.pose.orientation.w = 1.
            goal.position_tolerance_m = .2
            goal.stopped_speed_mps = .05
            goal.stable_duration_s = .6
            goal.timeout_s = args.timeout
            goal.control_session.session_id.uuid = list(uuid.uuid4().bytes)
            goal.control_session.owner = 'ordinary-action-regression'
            goal.control_session.generation = index+1
            goal.map_session = f'{state["map"].map_id}:{state["map"].epoch}'
            start_truth = list(state['truth'])
            begin_index = len(samples)
            sent = client.send_goal_async(goal, feedback_callback=progress)
            if not spin_until(sent.done, 5.):
                raise RuntimeError('Goal response timeout')
            active = sent.result()
            if not active.accepted:
                raise RuntimeError('Goal rejected')
            record('accepted', {'uuid': bytes(active.goal_id.uuid).hex(), 'index': index})
            terminal = active.get_result_async()
            cancel_future = None
            end = time.monotonic()+args.timeout+8.
            while not terminal.done() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
                if args.cancel and cancel_future is None and distance(start_truth, state['truth']) >= .35:
                    cancel_future = active.cancel_goal_async()
                    record('cancel_requested', {'displacement_m': distance(start_truth, state['truth'])})
            if not terminal.done():
                raise RuntimeError('Terminal result timeout')
            wrapped = terminal.result()
            value = wrapped.result
            outcome = record('result', {
                'uuid': bytes(active.goal_id.uuid).hex(), 'status': wrapped.status,
                'result_code': value.result_code, 'reason': value.reason,
                'cleanup_confirmed': value.cleanup_confirmed, 'mock': value.mock,
                'truth_position': list(state['truth']), 'goal_error_m': distance(point, state['truth']),
                'displacement_m': max(distance(start_truth, s['position']) for s in samples[begin_index:])})
            active = None
            hold_start = len(samples)
            spin_until(lambda: False, 1.)
            hold = samples[hold_start:]
            outcome['post_result_hold_drift_m'] = max(
                (distance(outcome['truth_position'], s['position']) for s in hold), default=None)
            expected_status = GoalStatus.STATUS_CANCELED if args.cancel else GoalStatus.STATUS_SUCCEEDED
            outcome['passed'] = (
                wrapped.status == expected_status and value.result_code == ('CANCELED' if args.cancel else 'SUCCEEDED') and
                value.cleanup_confirmed and not value.mock and outcome['displacement_m'] >= .3 and
                outcome['post_result_hold_drift_m'] is not None and outcome['post_result_hold_drift_m'] < .02 and
                (args.cancel or outcome['goal_error_m'] <= .2) and
                (not args.cancel or (cancel_future is not None and cancel_future.done() and
                                     bool(cancel_future.result().goals_canceling))))
            results.append(outcome)
            if not outcome['passed']:
                break
        report['min_truth_clearance_m'] = min(clearance(s['position']) for s in samples)
        report['passed'] = (len(results) == len(waypoints) and all(r['passed'] for r in results) and
                            report['min_truth_clearance_m'] > 0.)
    except Exception as exc:
        report['error'] = str(exc)
    finally:
        if active is not None:
            canceled = active.cancel_goal_async()
            spin_until(canceled.done, 3.)
            terminal = active.get_result_async()
            spin_until(terminal.done, 8.)
        report['results'] = results
        report['feedback_count'] = len(feedback)
        report['source_hashes'] = source_hashes
        (args.output/'result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(report, indent=2, allow_nan=False), flush=True)
        trace.close()
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    CACHE.mkdir(parents=True, exist_ok=True)
    try:
        with lock(CACHE/'operation.lock'):
            raise SystemExit(main())
    except RuntimeError as exc:
        print(str(exc), file=sys.stderr)
        raise SystemExit(1) from None
