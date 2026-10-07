#!/usr/bin/env python3
"""Owned ROS Action/service protocol smoke. Synthetic data; no vehicle I/O."""
import fcntl
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_services_default
from action_msgs.msg import GoalStatus
from uav_nav_interfaces.action import ExecuteMission, NavigateToPose3D
from uav_nav_interfaces.msg import TaskStatus
from uav_nav_interfaces.srv import PauseMission, ResumeMission
from unique_identifier_msgs.msg import UUID

from run_px4_sitl_smoke import stop
from sim_validation import ROOT, write_json, file_hash


def uid(value=None):
    return UUID(uuid=list((value or uuid.uuid4()).bytes))


def main():
    # Deliberately separate from algorithm domain 68 and SITL domain 78.
    os.environ['ROS_DOMAIN_ID'] = '79'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    run = ROOT / '.cache/simulation/mission-protocol' / str(uuid.uuid4())
    run.mkdir(parents=True)
    result = dict(scope='S1 mock protocol ONLY; no motion/localization/BT validation', passed=False, checks=[])
    process = None
    rclpy.init()
    node = rclpy.create_node('mission_protocol_observer')
    history = []
    subscription = node.create_subscription(TaskStatus, '/uav/mock/task_status', history.append, 100)
    action = ActionClient(node, ExecuteMission, '/uav/mock/execute_mission')
    navigate = ActionClient(node, NavigateToPose3D, '/uav/mock/navigate')
    pause = node.create_client(PauseMission, '/uav/mock/pause', qos_profile=qos_profile_services_default)
    resume = node.create_client(ResumeMission, '/uav/mock/resume', qos_profile=qos_profile_services_default)

    def check(condition, name):
        result['checks'].append(dict(name=name, passed=bool(condition)))
        if not condition:
            raise RuntimeError(name)

    def future(f, timeout=5.):
        deadline = time.monotonic() + timeout
        while not f.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
        if not f.done():
            raise TimeoutError('ROS future timeout')
        return f.result()

    def state(handle, phase, timeout=5.):
        goal = list(handle.goal_id.uuid)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.02)
            for msg in reversed(history):
                if list(msg.mission_uuid.uuid) == goal and msg.phase == phase:
                    return msg
        raise TimeoutError(f'Task phase timeout: {phase}')

    def goal(**options):
        request = ExecuteMission.Goal(mission_type='NAVIGATE', backend='MOCK',
                                      parameters_json=json.dumps(options), timeout_s=8.)
        return future(action.send_goal_async(request))

    def command(client, kind, handle, instance, request_id):
        return future(client.call_async(kind.Request(mission_uuid=handle.goal_id,
                      coordinator_instance=instance, request_id=request_id)))

    try:
        with (run / 'server.log').open('w') as log:
            process = subprocess.Popen([sys.executable, '-c',
                       'from uav_mission.mock_server import main; main()'], cwd=ROOT,
                       stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            check(action.wait_for_server(timeout_sec=5.) and navigate.wait_for_server(timeout_sec=5.)
                  and pause.wait_for_service(timeout_sec=5.) and resume.wait_for_service(timeout_sec=5.),
                  'Action and service discovery')
            invalid = ExecuteMission.Goal(mission_type='NAVIGATE', backend='PX4', timeout_s=8.)
            check(not future(action.send_goal_async(invalid)).accepted, 'Real backend rejected by mock fixture')
            handle = goal(work_s=3.)
            check(handle.accepted, 'Root accepted')
            active = state(handle, 'RUNNING')
            original_child = list(active.child_uuid.uuid)
            busy = goal(work_s=1.)
            check(not busy.accepted, 'BUSY rejects second root')
            request_id = uid()
            decision = command(pause, PauseMission, handle, active.coordinator_instance, request_id)
            check(decision.accepted and decision.phase == 'PAUSING', 'Pause accepted before stop confirmation')
            paused = state(handle, 'PAUSED')
            check(paused.control_session.owner == 'HOLD_CONTROLLER' and not paused.has_child,
                  'Pause keeps parent active and retires child')
            pending_result = handle.get_result_async()
            until = time.monotonic() + 3.2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.02)
            check(not pending_result.done(), 'Pause preserves parent beyond synthetic motion budget')
            duplicate = command(pause, PauseMission, handle, active.coordinator_instance, request_id)
            check((duplicate.accepted, duplicate.reason, duplicate.phase) ==
                  (decision.accepted, decision.reason, decision.phase), 'Pause request idempotency')
            stale = command(resume, ResumeMission, handle, str(uuid.uuid4()), uid())
            check(not stale.accepted and stale.reason == 'STALE_IDENTITY', 'Old instance rejected')
            resumed = command(resume, ResumeMission, handle, active.coordinator_instance, uid())
            check(resumed.accepted, 'Resume accepted')
            # Skip older RUNNING samples retained before pause.
            deadline = time.monotonic() + 3
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.02)
                current = history[-1] if history else None
                if (current and list(current.mission_uuid.uuid) == list(handle.goal_id.uuid)
                        and current.phase == 'RUNNING' and current.control_session.generation > paused.control_session.generation):
                    break
            check(current.phase == 'RUNNING' and list(current.child_uuid.uuid) != original_child
                  and current.control_session.generation > paused.control_session.generation,
                  'Resume creates new child and generation')
            check(bool(future(handle.cancel_goal_async()).goals_canceling), 'Cancel accepted')
            canceled = future(pending_result)
            check(canceled.status == GoalStatus.STATUS_CANCELED and canceled.result.cleanup_confirmed
                  and canceled.result.mock, 'CANCELED only after confirmed mock handoff')
            cancel_state = state(handle, 'CANCELED')
            check(cancel_state.control_session.owner == 'HOLD_CONTROLLER', 'Hold session survives parent result')
            next_handle = goal(work_s=.2)
            check(next_handle.accepted, 'Explicit new root takes over final hold')
            old_pause = command(pause, PauseMission, handle, active.coordinator_instance, uid())
            check(not old_pause.accepted and old_pause.reason == 'STALE_IDENTITY', 'Old root cannot pause new root')
            check(not future(handle.cancel_goal_async()).goals_canceling, 'Late old cancel cannot cancel new root')
            done = future(next_handle.get_result_async())
            check(done.status == GoalStatus.STATUS_SUCCEEDED and done.result.cleanup_confirmed and done.result.mock,
                  'Synthetic success after cleanup')
            failed_handle = goal(work_s=3., handoff_failure=True)
            check(failed_handle.accepted, 'Handoff fault fixture accepted')
            check(bool(future(failed_handle.cancel_goal_async()).goals_canceling), 'Fault fixture cancel accepted')
            failed = future(failed_handle.get_result_async())
            check(failed.status == GoalStatus.STATUS_ABORTED and failed.result.result_code == 'CANCEL_TIMEOUT'
                  and not failed.result.cleanup_confirmed, 'Failed handoff ABORTED without safe-cancel claim')
            nav = NavigateToPose3D.Goal(timeout_s=5., position_tolerance_m=.2,
                                       stopped_speed_mps=.1, stable_duration_s=.2)
            nav.goal.header.frame_id = 'map'
            nav.goal.pose.orientation.w = 1.
            nav_handle = future(navigate.send_goal_async(nav))
            check(nav_handle.accepted, 'Navigate Action ordinary client accepted')
            nav_result = future(nav_handle.get_result_async())
            check(nav_result.status == GoalStatus.STATUS_SUCCEEDED and nav_result.result.mock
                  and nav_result.result.cleanup_confirmed, 'Navigate synthetic protocol result')
            check(all(m.mock for m in history), 'Every published status marked mock')
            result['passed'] = True
    except Exception as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if process:
            stop(process)
        result['server_exit_code'] = process.returncode if process else None
        result['status_samples'] = len(history)
        result['source_sha256'] = {str(p.relative_to(ROOT)): file_hash(p) for p in
                                  [Path(__file__).resolve(), ROOT / 'src/uav_mission/uav_mission/protocol.py',
                                   ROOT / 'src/uav_mission/uav_mission/mock_server.py']}
        write_json(run / 'observation.json', result)
        write_json(run / 'events.json', [dict(mission_uuid=str(uuid.UUID(bytes=bytes(m.mission_uuid.uuid))),
                   instance=m.coordinator_instance, sequence=m.event_sequence, phase=m.phase,
                   generation=m.control_session.generation, owner=m.control_session.owner,
                   child_uuid=str(uuid.UUID(bytes=bytes(m.child_uuid.uuid))), reason=m.reason) for m in history])
        action.destroy()
        navigate.destroy()
        node.destroy_node()
        rclpy.shutdown()
    print(f'Evidence: {run}\nPASS: {result["passed"]}')
    if not result['passed']:
        print(result.get('error'))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    guard = ROOT / '.cache/simulation/mission-protocol.lock'
    guard.parent.mkdir(parents=True, exist_ok=True)
    with guard.open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        raise SystemExit(main())
