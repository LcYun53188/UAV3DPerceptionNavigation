#!/usr/bin/env python3
"""Owned C++ BT/MissionServer regression in the managed algorithm lab."""
import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from action_msgs.msg import GoalStatusArray
from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from uav_nav_interfaces.action import ExecuteMission, NavigateToPose3D
from uav_nav_interfaces.msg import ControlStatus, MapSnapshot, MissionProgress, TimedTrajectory
from uav_nav_interfaces.srv import PauseMission, ResumeMission
from check_gazebo_navigation_action import clearance, xyz
from sim_control import CACHE, lock, read_session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', choices=['pause-resume', 'cancel-pausing', 'runner-stall'], default='pause-resume')
    parser.add_argument('--timeout', type=float, default=120.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 20. <= args.timeout <= 600.:
        parser.error('timeout must be finite and 20..600')
    root = Path(__file__).resolve().parents[1]
    session = read_session()
    if (session['layout'] != 'lab' or session['mode'] != 'mapping' or
            int(os.environ.get('ROS_DOMAIN_ID', '0')) != session['domain'] or
            os.environ.get('GZ_PARTITION') != session['partition']):
        parser.error('Require managed lab/mapping and matching domain/partition')
    world_hash = hashlib.sha256((root/'src/uav_bringup/gazebo/worlds/uav_ego_lab.sdf').read_bytes()).hexdigest()
    if world_hash != '343d0cef432f45b0bbd854af71104a8908ee534de5c2bcb96eeef3dfc34f0424':
        parser.error('Review the independent clearance oracle after geometry changes')
    binary = Path(get_package_prefix('uav_bt'))/'lib/uav_bt/algorithm_mission_server'
    files = [Path(__file__).resolve(), root/'src/uav_bt/src/algorithm_mission_server.cpp',
             root/'src/uav_bt/trees/algorithm_waypoints.xml',
             root/'src/uav_nav_sim/uav_nav_sim/control_reservation.py',
             root/'src/uav_nav_sim/uav_nav_sim/navigation_action.py',
             root/'src/uav_nav_sim/uav_nav_sim/executor.py',
             root/'src/uav_nav_sim/uav_nav_sim/navigation.py',
             root/'src/uav_nav_sim/uav_nav_sim/autonomous.py']
    hashes = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    binary_hash = hashlib.sha256(binary.read_bytes()).hexdigest()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = rclpy.create_node('algorithm_mission_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    client = ActionClient(node, ExecuteMission, '/uav/algorithm/execute_mission')
    trace = (args.output/'trace.jsonl').open('w')
    log = (args.output/'server.log').open('w')
    begun = time.monotonic()
    state = {'truth': None, 'map': None, 'feedback': None, 'trajectory': None, 'control': None}
    samples, children, child_results, tokens, controls, progress = [], set(), {}, [], [], []
    process = handle = None
    waypoint_errors = {}
    report = {'profile': 'algorithm', 'scenario': args.scenario, 'passed': False, 'mock': False,
              'source_hashes': hashes, 'binary_sha256': binary_hash, 'world_sha256': world_hash,
              'managed_session': session}

    def record(kind, value):
        value = {'t': time.monotonic()-begun, 'kind': kind, **value}
        trace.write(json.dumps(value, allow_nan=False)+'\n')
        trace.flush()
        return value

    def truth(msg):
        q = msg.pose.pose.orientation
        state['truth'] = xyz(msg)
        samples.append(record('truth', {'position': xyz(msg), 'quaternion': [q.x, q.y, q.z, q.w],
            'stamp': msg.header.stamp.sec+msg.header.stamp.nanosec/1e9}))

    def odom(msg):
        v, w = msg.twist.twist.linear, msg.twist.twist.angular
        record('odom', {'position': xyz(msg), 'speed': math.hypot(v.x, v.y, v.z),
                        'angular_speed': math.hypot(w.x, w.y, w.z)})

    def feedback(msg):
        f = msg.feedback
        state['feedback'] = f
        child = bytes(f.status.child_uuid.uuid).hex() if f.status.has_child else None
        if child:
            children.add(child)
        index = int(f.tree_node.split('[')[-1].rstrip(']'))
        if index > 0 and index-1 not in waypoint_errors and state['truth'] is not None:
            waypoint_errors[index-1] = math.dist(waypoints[index-1], state['truth'])
        record('root_feedback', {'uuid': bytes(msg.goal_id.uuid).hex(), 'phase': f.status.phase,
            'sequence': f.status.event_sequence, 'child_uuid': child, 'waypoint_index': index,
            'remaining_s': f.status.total_remaining_s, 'instance': f.status.coordinator_instance,
            'control_generation': f.status.control_session.generation, 'reason': f.status.reason})

    def trajectory(msg):
        state['trajectory'] = copy.deepcopy(msg)
        token = msg.goal_stamp.sec*1_000_000_000+msg.goal_stamp.nanosec
        tokens.append(record('trajectory', {'goal_stamp': token, 'trajectory_id': msg.trajectory_id}))

    def control(msg):
        state['control'] = msg
        controls.append(record('control', {'uuid': bytes(msg.session.session_id.uuid).hex(),
            'owner': msg.session.owner, 'generation': msg.session.generation,
            'reason': msg.reason, 'lease_remaining_s': msg.lease_remaining_s}))

    def tick_progress(msg):
        progress.append(record('bt_progress', {'uuid': bytes(msg.mission_uuid.uuid).hex(),
            'instance': msg.coordinator_instance, 'tick': msg.tick_sequence}))

    get_result = node.create_client(NavigateToPose3D.Impl.GetResultService,
                                   '/uav/algorithm/navigate/_action/get_result')
    queried = set()

    def child_status(msg):
        for status in msg.status_list:
            child = bytes(status.goal_info.goal_id.uuid).hex()
            if status.status not in (4, 5, 6) or child in queried:
                continue
            queried.add(child)
            request = NavigateToPose3D.Impl.GetResultService.Request(goal_id=status.goal_info.goal_id)
            future = get_result.call_async(request)

            def completed(future, child=child):
                value = future.result()
                child_results[child] = record('child_result', {'uuid': child, 'status': value.status,
                    'result_code': value.result.result_code, 'reason': value.result.reason,
                    'cleanup_confirmed': value.result.cleanup_confirmed, 'mock': value.result.mock})
            future.add_done_callback(completed)

    node.create_subscription(Odometry, '/visual_slam/tracking/odometry', truth, qos_profile_sensor_data)
    node.create_subscription(Odometry, '/uav/localization/odometry', odom, qos_profile_sensor_data)
    node.create_subscription(MapSnapshot, '/uav/map/snapshot', lambda m: state.update(map=m),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(TimedTrajectory, '/uav/trajectory', trajectory, 10)
    node.create_subscription(ControlStatus, '/uav/algorithm/control_status', control, 10)
    node.create_subscription(MissionProgress, '/uav/algorithm/mission_progress', tick_progress, 10)
    node.create_subscription(GoalStatusArray, '/uav/algorithm/navigate/_action/status', child_status,
        QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    events = []
    node.create_subscription(String, '/uav/executor/event', lambda m: events.append(
        record('executor_event', {'event': m.data})), 10)
    pause_client = node.create_client(PauseMission, '/uav/algorithm/pause_mission')
    resume_client = node.create_client(ResumeMission, '/uav/algorithm/resume_mission')
    trajectory_pub = node.create_publisher(TimedTrajectory, '/uav/trajectory', 10)
    legacy_goal_pub = node.create_publisher(PoseStamped, '/uav/goal', 10)
    waypoints = [[-2.2, .6, 1.2], [-3.2, -.6, 1.2]]

    def until(predicate, timeout=5.):
        deadline = time.monotonic()+timeout
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not predicate():
            raise RuntimeError('Timed out waiting for mission state')

    def rpc(service, request):
        until(service.service_is_ready)
        future = service.call_async(request)
        until(future.done)
        response = future.result()
        record('service_response', {'service': service.srv_name,
            'accepted': getattr(response, 'accepted', getattr(response, 'success', None)),
            'reason': getattr(response, 'reason', getattr(response, 'message', '')),
            'phase': getattr(response, 'phase', '')})
        return response

    def pause_request():
        request = PauseMission.Request()
        request.mission_uuid = handle.goal_id
        request.coordinator_instance = state['feedback'].status.coordinator_instance
        request.request_id.uuid = list(uuid.uuid4().bytes)
        return request

    try:
        if client.wait_for_server(timeout_sec=.3):
            raise RuntimeError('An algorithm MissionServer already exists; refusing a duplicate')
        until(lambda: state['map'] is not None and state['map'].valid and state['truth'] is not None)
        report['command_publishers'] = [i.node_name for i in node.get_publishers_info_by_topic('/cmd_vel')]
        if report['command_publishers'] != ['gazebo_trajectory_executor']:
            raise RuntimeError('Require exactly one known velocity publisher')
        process = subprocess.Popen([str(binary), '--ros-args', '-p', 'use_sim_time:=true',
            '-p', f'checkpoint_file:={args.output.resolve()/"checkpoint.json"}'],
            stdout=log, stderr=log)
        until(client.server_is_ready)
        # A rejected request verifies the SendGoal response path before any
        # valid mission can reserve the backend. Discovery alone is not a bidirectional check.
        connected = False
        for attempt in range(3):
            probe = client.send_goal_async(ExecuteMission.Goal(mission_type='DISCOVERY_PROBE'))
            try:
                until(probe.done, 3.)
            except RuntimeError:
                record('discovery_probe', {'attempt': attempt+1, 'response_received': False})
                continue
            if probe.result().accepted:
                raise RuntimeError('Invalid discovery probe was accepted')
            record('discovery_probe', {'attempt': attempt+1, 'response_received': True})
            connected = True
            break
        if not connected:
            raise RuntimeError('SendGoal response channel unavailable; no valid mission submitted')
        identity_client = node.create_client(GetParameters, '/algorithm_mission_server/get_parameters')
        identity = rpc(identity_client, GetParameters.Request(names=['coordinator_instance']))
        instance = identity.values[0].string_value
        if not instance:
            raise RuntimeError('Missing current server identity')
        record('server_identity', {'instance': instance})
        goal = ExecuteMission.Goal(mission_type='WAYPOINTS', backend='ALGORITHM', timeout_s=args.timeout,
            parameters_json=json.dumps({'coordinator_instance': instance,
                                        'map_session': f'{state["map"].map_id}:{state["map"].epoch}',
                                        'waypoints': waypoints, 'navigation_timeout_s': 90.}))
        initial = list(state['truth'])
        report['initial_truth'] = initial
        sent = client.send_goal_async(goal, feedback_callback=feedback)
        until(sent.done)
        handle = sent.result()
        if not handle.accepted:
            raise RuntimeError('Root mission rejected')
        root_result = handle.get_result_async()
        until(lambda: state['feedback'] is not None and state['feedback'].status.has_child and
              state['trajectory'] is not None and math.dist(initial, state['truth']) >= .3, 45.)
        first_child = bytes(state['feedback'].status.child_uuid.uuid).hex()
        old_trajectory = copy.deepcopy(state['trajectory'])
        if args.scenario == 'runner-stall':
            record('runner_stop', {'pid': process.pid})
            process.send_signal(signal.SIGSTOP)
            until(lambda: any(c['reason'] == 'BT_PROGRESS_TIMEOUT' for c in controls), 2.)
            until(lambda: first_child in child_results, 3.)
            process.send_signal(signal.SIGCONT)
            record('runner_continue', {})
        else:
            request = pause_request()
            accepted = rpc(pause_client, request)
            if not accepted.accepted or accepted.phase != 'PAUSING':
                raise RuntimeError('Pause was not accepted')
            if args.scenario == 'cancel-pausing':
                cancel = handle.cancel_goal_async()
                until(cancel.done)
                if not cancel.result().goals_canceling:
                    raise RuntimeError('Root cancellation rejected during PAUSING')
            else:
                until(lambda: state['feedback'].status.phase == 'PAUSED')
                if root_result.done():
                    raise RuntimeError('Pause incorrectly terminated the parent')
                paused_checkpoint = json.loads((args.output/'checkpoint.json').read_text())
                (args.output/'paused_checkpoint.json').write_text(json.dumps(paused_checkpoint, indent=2)+'\n')
                pause_start = len(samples)
                pause_position = list(state['truth'])
                pause_quaternion = list(samples[-1]['quaternion'])
                # Replayed tokens and legacy entry points must not steal a paused parent.
                trajectory_pub.publish(old_trajectory)
                legacy = PoseStamped()
                legacy.header.frame_id = 'map'
                legacy.pose.position.x, legacy.pose.position.y, legacy.pose.position.z = waypoints[0]
                legacy.pose.orientation.w = 1.
                legacy_goal_pub.publish(legacy)
                legacy_cancel = node.create_client(Trigger, '/uav/cancel')
                if rpc(legacy_cancel, Trigger.Request()).success:
                    raise RuntimeError('Legacy cancellation bypassed parent ownership')
                exploration = node.create_client(SetBool, '/uav/exploration/enabled')
                if rpc(exploration, SetBool.Request(data=True)).success:
                    raise RuntimeError('Exploration bypassed parent ownership')
                repeat = rpc(pause_client, request)
                if (repeat.accepted, repeat.phase) != (accepted.accepted, accepted.phase):
                    raise RuntimeError('Duplicate request was not idempotent')
                until(lambda: len(samples) > pause_start and samples[-1]['t']-samples[pause_start]['t'] >= 2., 4.)
                paused_samples = samples[pause_start:]
                report['pause_drift_m'] = max(math.dist(pause_position, s['position']) for s in paused_samples)
                report['pause_rotation_rad'] = max(2*math.acos(min(1., abs(sum(a*b for a, b in
                    zip(pause_quaternion, s['quaternion']))))) for s in paused_samples)
                if report['pause_drift_m'] > .02 or report['pause_rotation_rad'] > .03 or root_result.done():
                    raise RuntimeError('Parent pause was not a stable nonterminal hold')
                resume = ResumeMission.Request(mission_uuid=handle.goal_id,
                    coordinator_instance=request.coordinator_instance)
                resume.request_id.uuid = list(uuid.uuid4().bytes)
                report['resume_t'] = time.monotonic()-begun
                if not rpc(resume_client, resume).accepted:
                    raise RuntimeError('Resume rejected')
                until(lambda: state['feedback'].status.has_child and
                      bytes(state['feedback'].status.child_uuid.uuid).hex() != first_child)
                trajectory_pub.publish(old_trajectory)
        until(root_result.done, args.timeout)
        wrapped = root_result.result()
        value = wrapped.result
        record('root_result', {'status': wrapped.status, 'code': value.result_code,
            'reason': value.reason, 'cleanup_confirmed': value.cleanup_confirmed, 'mock': value.mock})
        report['root_result'] = {'status': wrapped.status, 'code': value.result_code, 'reason': value.reason,
                                 'cleanup_confirmed': value.cleanup_confirmed, 'mock': value.mock}
        final = list(state['truth'])
        hold_start = len(samples)
        until(lambda: len(samples) > hold_start and samples[-1]['t']-samples[hold_start]['t'] >= 1., 3.)
        report['post_result_hold_drift_m'] = max(math.dist(final, s['position']) for s in samples[hold_start:])
        report['final_truth'] = final
        report['displacement_m'] = max(math.dist(initial, s['position']) for s in samples)
        report['min_truth_clearance_m'] = min(clearance(s['position']) for s in samples)
        report['children'] = {child: child_results.get(child) for child in children}
        report['waypoint_errors_m'] = waypoint_errors
        report['obsolete_goal_rejections'] = sum(e['event'] == 'REJECTED:OBSOLETE_GOAL' for e in events)
        expected = {'pause-resume': ('SUCCEEDED', 4), 'cancel-pausing': ('CANCELED', 5),
                    'runner-stall': ('ABORTED', 6)}[args.scenario]
        passed = (value.result_code == expected[0] and wrapped.status == expected[1] and
            value.cleanup_confirmed and not value.mock and report['post_result_hold_drift_m'] < .02 and
            report['min_truth_clearance_m'] > 0. and report['displacement_m'] >= .3 and
            all(c is not None and c['cleanup_confirmed'] and not c['mock'] for c in report['children'].values()))
        if args.scenario == 'pause-resume':
            resumed_tokens = [t['goal_stamp'] for t in tokens if t['t'] >= report['resume_t']]
            old_token = old_trajectory.goal_stamp.sec*1_000_000_000+old_trajectory.goal_stamp.nanosec
            # Our two deliberate replays are recorded too; require a fresh planner token.
            report['fresh_resume_tokens'] = sorted(set(resumed_tokens)-{old_token})
            passed &= (len(children) == 3 and len(waypoint_errors) == 2 and
                max(waypoint_errors.values()) <= .2 and bool(report['fresh_resume_tokens']) and
                report['obsolete_goal_rejections'] >= 2)
        if args.scenario == 'runner-stall':
            passed &= value.reason == 'BT_PROGRESS_TIMEOUT'
            fault = next(c for c in controls if c['reason'] == 'BT_PROGRESS_TIMEOUT')
            previous = [p for p in progress if p['t'] <= fault['t'] and p['uuid'] == bytes(handle.goal_id.uuid).hex()]
            report['last_tick_to_fault_s'] = fault['t']-previous[-1]['t']
            passed &= report['last_tick_to_fault_s'] < .8
        report['passed'] = bool(passed)
    except Exception as error:
        report['error'] = str(error)
    finally:
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGCONT)
        if handle is not None and handle.accepted:
            try:
                canceled = handle.cancel_goal_async()
                until(canceled.done, 2.)
                terminal = handle.get_result_async()
                until(terminal.done, 8.)
            except Exception as error:
                report['cleanup_error'] = str(error)
        if process is not None:
            process.terminate()
            end = time.monotonic()+4.
            while process.poll() is None and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.02)
            if process.poll() is None:
                process.kill()
            process.wait(timeout=2.)
            report['server_exit_code'] = process.returncode
        log.close()
        trace.close()
        (args.output/'result.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print(json.dumps(report, indent=2, allow_nan=False), flush=True)
        client.destroy()
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    CACHE.mkdir(parents=True, exist_ok=True)
    try:
        with lock(CACHE/'operation.lock'):
            raise SystemExit(main())
    except RuntimeError as error:
        print(str(error), file=sys.stderr)
        raise SystemExit(1) from None
