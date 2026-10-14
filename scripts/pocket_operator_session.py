#!/usr/bin/env python3
"""SA arm, SB manual/navigation, SD start/pause/resume in an owned USB SITL."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid

from pocket_switches import PocketSwitches
from usb_joystick import Joystick

ROOT = Path(__file__).resolve().parents[1]
DEVICE = '/dev/input/by-id/usb-EdgeTX_Radiomaster_Pocket_Joystick_00000000001B-joystick'


def prepare_manual(node, switches, prepared):
    status, land = node.samples.get('vehicle_status'), node.samples.get('vehicle_land_detected')
    if (not prepared and not switches.navigation and node.active_goal is None and
            status is not None and status.arming_state == 1 and land is not None and land.landed and
            node.operator_input_ready()):
        accepted, _ = node.operator_command('MANUAL')
        return accepted
    return prepared


def return_intent(node):
    steps = getattr(node, 'steps', [])
    index = node.step_index
    returning = node.return_override or (0 <= index < len(steps) and steps[index]['type'] == 'RETURN')
    return ('RESUME' if node.phase == 'PAUSED' else 'PAUSE') if returning else 'RETURN'


def run_intent(node):
    if node.phase != 'PAUSED':
        return None  # A separate, guarded ground START; never restore a moving return.
    return ('RESUME_MISSION' if getattr(node,'return_override',False) and
            getattr(node,'return_checkpoint',None) is not None else 'RESUME')


def bind_session(path):
    session = Path(path).resolve()
    if not session.is_relative_to(ROOT/'.cache/simulation/sitl'):
        raise ValueError('Session must be in the owned SITL cache')
    manifest = json.loads((session/'manifest.json').read_text())
    if (not manifest.get('usb_joystick') or manifest.get('model') != 'x500_7' or
            manifest.get('domain') != 78 or not manifest['partition'].startswith('uav_px4_s0_')):
        raise ValueError('Requires the isolated GNSS x500 USB profile')
    frozen = manifest.get('px4_parameter_overrides',{})
    if (frozen.get('PX4_PARAM_COM_OF_LOSS_T') != '0.5' or frozen.get('PX4_PARAM_COM_OBL_RC_ACT') != '4'):
        raise ValueError('Restart the manual USB profile with the frozen Offboard-loss landing parameters')
    pid = manifest['processes']['px4']['pid']
    command = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
    expected = str(session/'rootfs').encode()
    if expected not in command or not command[0].endswith(b'/bin/px4'):
        raise ValueError('Owned PX4 process is not alive or was replaced')
    return session, manifest


def observe(device, return_button=None):
    joystick, switches = Joystick(device), PocketSwitches(return_button=return_button)
    try:
        while True:
            event = joystick.read(.05)
            if event:
                switches.receive(event, time.monotonic())
            for command in switches.tick():
                print(json.dumps(dict(observe_only=True, command=command,
                    arm=switches.arm, navigation=switches.navigation)), flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        joystick.close()


def control(session, device, return_button=None):
    from copy import deepcopy
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from uav_mission.px4_flight import FlightServer
    from uav_nav_interfaces.srv import SetMissionIntent
    session, manifest = bind_session(session)
    if manifest.get('flight_recipe_sha256'):
        raise ValueError('Pocket owner requires a manual session without another flight supervisor')
    os.environ.update(ROS_DOMAIN_ID='78', ROS_LOCALHOST_ONLY='1', UAV_POCKET_OPERATOR='1',
        UAV_SITL_AUTHORIZATION=str(uuid.uuid4()), UAV_WORKSPACE=str(ROOT), GZ_PARTITION=manifest['partition'])
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'], signal_handler_options=SignalHandlerOptions.NO)
    node = FlightServer()
    executor = SingleThreadedExecutor(); executor.add_node(node)
    joystick, switches = Joystick(device), PocketSwitches(return_button=return_button)
    intent = node.create_client(SetMissionIntent, '/uav/bt/set_intent')
    runner = None
    runner_log = None
    parameters = session/'pocket-bt.private.json'
    requests = []
    pending = None
    outcomes = []
    stopping = False
    manual_prepared = False
    def stop_requested(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop_requested)
    signal.signal(signal.SIGTERM, stop_requested)

    def task_intent(verb):
        nonlocal pending
        if node.active_goal is None or not intent.service_is_ready():
            return False, 'NO_ACTIVE_TASK_OR_RUNNER'
        if pending is not None and not pending.done():
            return False, 'REQUEST_PENDING'
        request = SetMissionIntent.Request(mission_uuid=node.active_goal.goal_id,
            coordinator_instance=node.instance, command=verb)
        request.request_id.uuid = list(uuid.uuid4().bytes)
        pending = intent.call_async(request)
        return True, verb+'_REQUESTED'

    def start_task():
        nonlocal runner, runner_log
        if runner is not None and runner.poll() is None:
            return False, 'RUNNER_STILL_ACTIVE'
        if node.active_goal is not None or node.owner != 'NONE' or node.phase == 'FAULT':
            return False, 'TASK_BUSY_OR_LATCHED_FAULT'
        if not switches.arm or not switches.navigation or not node.operator_input_ready():
            return False, 'SA_SB_OR_QGC_NOT_READY'
        if (not node.healthy(ground=True) or not node.samples['vehicle_land_detected'].landed or
                node.samples['vehicle_status'].arming_state != 2 or node.speed() > .1):
            return False, 'GROUND_ARMED_STILL_REQUIRED'
        recipe = deepcopy(json.loads((ROOT/'simulation/missions/W0_flight_sequence.json').read_text())['steps'])
        home = node.position()
        for step in recipe:
            if 'offset_enu' in step:
                offset = step.pop('offset_enu')
                step['target_map'] = node.alignment.to_map(tuple(home[i]+offset[i] for i in range(3)))
        request = dict(authorization=node.nonce, steps=recipe, navigation_backend='DIRECT',
            runner_progress_required=True, coordinator_instance=node.instance, step_controlled=True)
        parameters.unlink(missing_ok=True)
        fd = os.open(parameters, os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            json.dump(request, stream)
        if runner_log is not None:
            runner_log.close()
        number = len(outcomes)+1
        runner_log = (session/f'pocket-bt-{number}.log').open('w')
        runner = subprocess.Popen([str(ROOT/'.deps/mission-install/uav_bt/lib/uav_bt/mission_runner'),
            '--ros-args', '-p', f'parameters_file:={parameters}', '-p', f'coordinator_instance:={node.instance}',
            '-p', 'timeout_s:=210.0', '-p', f'tree_output_file:={session / ("pocket-tree-"+str(number)+".xml")}'],
            stdout=runner_log, stderr=subprocess.STDOUT)
        return True, 'GROUND_TASK_DISPATCH_REQUESTED'

    try:
        print('Pocket owner ready: SA arm, SB navigation enable, SD run/pause. QGC calibration/fresh input required.', flush=True)
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=.01)
            now = time.monotonic()
            for _ in range(64):
                event = joystick.read()
                if event is None:
                    break
                switches.receive(event, now)
            node.operator_arm_enabled = switches.arm
            node.operator_navigation_enabled = switches.navigation
            # Apply the initial manual mode only after QGC is ready, on the ground
            # and disarmed. This never arms or starts a task.
            prepared = prepare_manual(node, switches, manual_prepared)
            if prepared and not manual_prepared:
                print('Initial manual mode requested; actual PX4 mode confirms the transition', flush=True)
            manual_prepared = prepared
            if pending is not None and pending.done():
                response = pending.result()
                print(f'Task admission: {response.accepted} {response.reason}', flush=True)
                pending = None
            if runner is not None and runner.poll() is not None:
                outcomes.append(dict(exit_code=runner.returncode, result=node.result))
                print(f'Root terminal: exit={runner.returncode} result={node.result}', flush=True)
                runner = None
                parameters.unlink(missing_ok=True)
            for command in switches.tick(now):
                node.operator_arm_enabled = switches.arm
                node.operator_navigation_enabled = switches.navigation
                if command in ('ARM','DISARM','MANUAL'):
                    accepted, reason = node.operator_command(command)
                elif command == 'RUN':
                    verb = run_intent(node)
                    accepted, reason = task_intent(verb) if verb else start_task()
                elif command == 'PAUSE':
                    accepted, reason = task_intent('PAUSE')
                elif command == 'TOGGLE_RETURN':
                    accepted, reason = task_intent(return_intent(node))
                else:
                    accepted, reason = False, 'SA_OR_SB_NOT_ENABLED'
                row = dict(mono=now, command=command, accepted=accepted, reason=reason)
                requests.append(row); print(json.dumps(row), flush=True)
    except (KeyboardInterrupt, OSError) as error:
        print(f'Pocket owner stopping: {error}', flush=True)
    finally:
        if runner is not None and runner.poll() is None:
            runner.send_signal(signal.SIGTERM)
            until = time.monotonic()+10
            while runner.poll() is None and time.monotonic()<until:
                executor.spin_once(timeout_sec=.02)
            if runner.poll() is None:
                node.fault('USB_OPERATOR_STOP_UNCONFIRMED')
                runner.kill(); runner.wait(timeout=3)
        # Keep the sole gateway alive for its bounded post-cancel hold/landing.
        until = time.monotonic()+45
        while node.owner in ('TASK','HOLD_CONTROLLER','AUTOPILOT') and time.monotonic()<until:
            executor.spin_once(timeout_sec=.02)
        snapshot = dict(requests=requests, outcomes=outcomes, final_owner=node.owner,
            phase=node.phase, result=node.result, commands=node.commands,
            scope='Pocket operator session; physical state remains separate from request admission')
        (session/'pocket-operator.json').write_text(json.dumps(snapshot, indent=2)+'\n')
        parameters.unlink(missing_ok=True)
        if runner_log is not None:
            runner_log.close()
        joystick.close(); executor.shutdown(); node.destroy_node(); rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default=DEVICE)
    parser.add_argument('--session', help='Owned manual SITL evidence directory')
    parser.add_argument('--return-button', type=int, choices=[0], help='Explicitly enable the verified Linux momentary button 0; default unassigned')
    parser.add_argument('--enable-sitl-control', action='store_true')
    args = parser.parse_args()
    if not args.enable_sitl_control:
        return observe(args.device, args.return_button)
    if not args.session:
        parser.error('--enable-sitl-control requires --session')
    session, _ = bind_session(args.session)
    with (session/'pocket-operator.lock').open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX|fcntl.LOCK_NB)
        control(session, args.device, args.return_button)


if __name__ == '__main__':
    main()
