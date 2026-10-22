#!/usr/bin/env python3
"""Owned Pocket SITL control with explicit legacy USB or UAV AETR profiles."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[2])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

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
from pocket_uav_switches import PocketUavSwitches
from usb_joystick import Joystick

ROOT = Path(__file__).resolve().parents[3]
DEVICE = '/dev/input/by-id/usb-EdgeTX_Radiomaster_Pocket_Joystick_00000000001B-joystick'


def prepare_manual(node, switches, prepared):
    status, land = node.samples.get('vehicle_status'), node.samples.get('vehicle_land_detected')
    if (not prepared and not switches.navigation and node.active_goal is None and
            status is not None and status.arming_state == 1 and land is not None and land.landed and
            node.operator_input_ready()):
        accepted, _ = node.operator_command(getattr(switches, 'requested_mode', 'MANUAL'))
        return accepted
    return prepared


def arm_from_switch(node, switches, *, task_prepared=False):
    mode=getattr(switches,'requested_mode',None)
    if isinstance(switches,PocketUavSwitches):
        if switches.safety_held:return False,'SD_SAFETY_HELD_RELEASE_BEFORE_CONTROL'
        if switches.navigation and not task_prepared:return False,'TASK_PREPARATION_REQUIRED'
        ready,reason=node.operator_mode_ready(mode)
        if not ready:return False,reason
        expected={'STABILIZED':15,'ALTITUDE':1}.get(mode,2)
        status=node.samples.get('vehicle_status')
        if status is None or status.nav_state != expected:
            prepared,detail=node.operator_command(mode)
            return False, ('ARM_MANUAL_MODE_REQUESTED_RETRY_SWITCH' if prepared
                           else 'ARM_MODE_PREPARE_REJECTED:'+detail)
    accepted, reason = node.operator_command('ARM')
    if reason == 'ARM_REQUIRES_MANUAL_MODE':
        # Mode preparation never queues an arm. The actual PX4 mode must be
        # confirmed before another physical switch edge can request ARM.
        mode = getattr(switches, 'requested_mode', 'MANUAL')
        prepared, detail = node.operator_command(mode)
        return False, ('ARM_MANUAL_MODE_REQUESTED_RETRY_SWITCH' if prepared
                       else 'ARM_MODE_PREPARE_REJECTED:'+detail)
    return accepted, reason


def return_intent(node):
    steps = getattr(node, 'steps', [])
    index = node.step_index
    returning = node.return_override or (0 <= index < len(steps) and steps[index]['type'] == 'RETURN')
    return ('RESUME' if node.phase == 'PAUSED' else 'PAUSE') if returning else 'RETURN'


def run_intent(node):
    if getattr(node,'pilot_control',None):return 'RESUME'
    if node.phase != 'PAUSED':
        return None  # A separate, guarded ground START; never restore a moving return.
    return ('RESUME_MISSION' if getattr(node,'return_override',False) and
            getattr(node,'return_checkpoint',None) is not None else 'RESUME')


def end_mission_intent(node, profile, dispatch):
    # A terminated/faulted root cannot accept another BT intent. Keep SD useful
    # for native rescue after a pilot envelope violation, without reviving it.
    if node.active_goal is None and (getattr(node,'cancel_manual_state',None) or node.phase=='FAULT'):
        return node.operator_command('LAND')
    if profile in ('uav-aetr-v1','pocket-rc-v2') and os.environ.get('UAV_NAVIGATION_BACKEND')!='EGO':
        return False,'RETURN_REQUIRES_AVOIDANCE_NAVIGATION'
    return dispatch('END_MISSION')


def task_parameters(node, manifest):
    """Build the same current-origin recipe for PREPARE and actual dispatch."""
    from copy import deepcopy
    recipe_file=(Path(os.environ['UAV_FLIGHT_MISSION_FILE'])
        if manifest.get('localization_profile')=='VIO_FLOW' else
        ROOT/'simulation/missions'/('flow_local_hover.json'
            if node.localization_profile=='FLOW_LOCAL' else 'W0_flight_sequence.json'))
    recipe=deepcopy(json.loads(recipe_file.read_text())['steps'])
    home=node.position()
    for step in recipe:
        if 'offset_enu' in step:
            offset=step.pop('offset_enu')
            step['target_map']=node.alignment.to_map(tuple(home[i]+offset[i] for i in range(3)))
    return dict(authorization=node.nonce,steps=recipe,
        navigation_backend=os.environ.get('UAV_NAVIGATION_BACKEND','DIRECT'),
        runner_progress_required=True,coordinator_instance=node.instance,step_controlled=True)


def prepare_task(node, switches, manifest, *, runner_ready=True):
    """Validate all survey/route envelopes on the ground; never dispatch or arm."""
    if not runner_ready:return False,'PREWARM_RUNNER_UNAVAILABLE_RESTART_SESSION'
    if not (switches.navigation and node.active_goal is None and node.owner=='NONE'
            and node.phase!='FAULT' and node.operator_input_ready() and node.healthy(ground=True)
            and node.samples['vehicle_land_detected'].landed and node.speed()<=.1):
        return False,'TASK_PREPARATION_GATE'
    from uav_nav_interfaces.action import ExecuteMission
    try:
        node.parse(ExecuteMission.Goal(backend='PX4_KNOWN_REGION',mission_type='FLIGHT_SEQUENCE',
            timeout_s=float(node.config.get('mission_timeout_max_s',240.)),
            parameters_json=json.dumps(task_parameters(node,manifest))))
    except (ValueError,KeyError,OSError) as exc:
        return False,'TASK_PREPARATION_INVALID_REQUEST:'+str(exc)
    return True,'TASK_PREPARED'


def bind_session(path):
    session = Path(path).resolve()
    if not any(session.is_relative_to(ROOT/'.cache/simulation'/name) for name in ('sitl','mtf01','vio-sensors')):
        raise ValueError('Session must be in the owned SITL cache')
    manifest = json.loads((session/'manifest.json').read_text())
    flow = manifest.get('localization_profile') == 'FLOW_LOCAL'
    vio=manifest.get('localization_profile')=='VIO_FLOW'
    expected_model, prefix = ('x500_vio_ref_7','uav_warehouse_flight_') if vio else ('x500_mtf01_7', 'uav_mtf01_') if flow else ('x500_7', 'uav_px4_s0_')
    if (not manifest.get('usb_joystick') or manifest.get('model') != expected_model or
            manifest.get('domain') != 78 or not manifest['partition'].startswith(prefix)):
        raise ValueError('Requires an isolated GNSS or FLOW_LOCAL Pocket USB profile')
    frozen = manifest.get('px4_parameter_overrides',{})
    if (frozen.get('PX4_PARAM_COM_OF_LOSS_T') != '0.5' or frozen.get('PX4_PARAM_COM_OBL_RC_ACT') != '4'):
        raise ValueError('Restart the manual USB profile with the frozen Offboard-loss landing parameters')
    if flow:
        expected=json.loads((ROOT/'simulation/px4/mtf01/flow_local.json').read_text())['parameters']
        if any(frozen.get('PX4_PARAM_'+key) != str(value) for key,value in expected.items()):
            raise ValueError('FLOW_LOCAL fusion parameters differ from the reviewed profile')
        if manifest.get('flight') or not json.loads((session/'ground-audit.json').read_text()).get('passed'):
            raise ValueError('FLOW_LOCAL Pocket requires successful ground fusion audit and no automatic flight')
    if vio:
        expected=json.loads((ROOT/'simulation/px4/mtf01/joint_fusion.json').read_text())['parameters']
        if (not manifest.get('pocket_session') or not manifest.get('experimental_chain') or
                frozen.get('PX4_PARAM_COM_RC_IN_MODE')!='1' or frozen.get('PX4_PARAM_EKF2_IMU_CTRL')!='5' or
                frozen.get('PX4_PARAM_NAV_RCL_ACT')!='3' or frozen.get('PX4_PARAM_COM_RC_OVERRIDE')!='0' or
                any(frozen.get('PX4_PARAM_'+key)!=str(value) for key,value in expected.items())):
            raise ValueError('VIO Pocket requires the isolated experimental joint fusion profile')
    pid = manifest['processes']['px4']['pid']
    command = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
    expected = str(session/'rootfs').encode()
    if expected not in command or not command[0].endswith(b'/bin/px4'):
        raise ValueError('Owned PX4 process is not alive or was replaced')
    return session, manifest


def operator_environment(session, manifest):
    if manifest.get('localization_profile')=='VIO_FLOW':
        recorded=manifest.get('operator_environment',{})
        region=recorded.get('UAV_FLIGHT_REGION_PROFILE')
        if region not in ('vio_chain','vio_avoidance'):raise ValueError('Unreviewed VIO Pocket region')
        mission='vio_static_avoidance.json' if region=='vio_avoidance' else 'depth_observation.json'
        task=manifest.get('pocket_task','avoidance')
        if task not in ('avoidance','intervention'):raise ValueError('Unreviewed Pocket task')
        if task=='intervention':
            limits={'PX4_PARAM_MPC_XY_VEL_MAX':'0.5','PX4_PARAM_MPC_XY_CRUISE':'0.5','PX4_PARAM_MPC_XY_VEL_ALL':'-10'}
            if region!='vio_avoidance' or any(manifest.get('px4_parameter_overrides',{}).get(k)!=v for k,v in limits.items()):
                raise ValueError('Pocket intervention requires its frozen native speed limits')
            mission='pocket_intervention.json'
        primary=manifest.get('vio_primary') is True
        if primary and (region!='vio_avoidance' or manifest.get('sdk_schur64') is not True):
            raise ValueError('VIO primary Pocket requires the owned SDK64 avoidance profile')
        expected=dict(UAV_REQUIRE_VIO='1',UAV_LOCALIZATION_PROFILE='DEFAULT',UAV_VIO_FUSION_PROFILE='aligned_pose_mtf_primary_sim_v1' if primary else 'aligned_pose_mtf_sim_v1',
            UAV_VIO_CALIBRATION_ID=manifest['calibration_id'],UAV_ENABLE_FUSION_SWITCH='0',
            UAV_FLIGHT_REGION_PROFILE=region,UAV_ENABLE_EGO_NAV='1',UAV_NAVIGATION_BACKEND='EGO',
            UAV_EXPERIMENTAL_VIO_CHAIN='1',UAV_EXPECT_OBSERVATION='1',
            UAV_FLIGHT_COORDINATOR_INSTANCE=manifest['run_id'],UAV_FLIGHT_MODEL='x500_vio_ref_7',
            UAV_FLIGHT_ASSET_RECEIPT=str(session/'flight-assets.json'),UAV_FLIGHT_EVIDENCE=str(session),
            UAV_FLIGHT_ADMISSION=str(session/'flight-admission.json'),
            UAV_FLIGHT_MISSION_FILE=str(ROOT/'simulation/missions'/mission))
        from sim_validation import file_hash
        expected['UAV_FLIGHT_ASSET_SHA256']=file_hash(session/'flight-assets.json')
        if any(recorded.get(key)!=value for key,value in expected.items()):
            raise ValueError('VIO Pocket environment/asset ownership mismatch')
        return expected
    if manifest.get('localization_profile') != 'FLOW_LOCAL':
        return dict(UAV_REQUIRE_VIO='0', UAV_LOCALIZATION_PROFILE='DEFAULT',
                    UAV_ENABLE_FUSION_SWITCH='0', UAV_FLIGHT_REGION_PROFILE='W0', UAV_ENABLE_EGO_NAV='0')
    expected=dict(UAV_REQUIRE_VIO='0', UAV_LOCALIZATION_PROFILE='FLOW_LOCAL', UAV_ENABLE_FUSION_SWITCH='0',
                  UAV_FLIGHT_REGION_PROFILE='flow_local', UAV_ENABLE_EGO_NAV='0', UAV_NAVIGATION_BACKEND='DIRECT',
                  UAV_FLIGHT_ASSET_RECEIPT=str(session/'flight-assets.json'),
                  UAV_FLIGHT_EVIDENCE=str(session), UAV_FLIGHT_MODEL='x500_mtf01_7')
    recorded=manifest.get('operator_environment', {})
    if any(recorded.get(key)!=value for key,value in expected.items()):
        raise ValueError('FLOW_LOCAL operator environment/asset ownership mismatch')
    from sim_validation import file_hash
    expected['UAV_FLIGHT_ASSET_SHA256']=file_hash(session/'flight-assets.json')
    if recorded.get('UAV_FLIGHT_ASSET_SHA256')!=expected['UAV_FLIGHT_ASSET_SHA256']:
        raise ValueError('FLOW_LOCAL asset receipt changed')
    return expected


def source_admitted(session, manifest):
    if manifest.get('localization_profile')!='VIO_FLOW':return True
    try:
        admission=json.loads((session/'flight-admission.json').read_text())
        valid=isinstance(admission,dict) and admission.get('sensor_checks_passed') is True and admission.get('calibration_id')==manifest['calibration_id']
        if valid and manifest.get('vio_primary') is True:
            valid=(admission.get('fusion_profile')=='aligned_pose_mtf_primary_sim_v1'
                and admission.get('joint_ground_preflight') is True)
        return valid
    except (OSError,ValueError):return False


def validate_virtual_device(device, joystick, profile):
    if profile != 'uav-aetr-v1':
        return
    from pocket_virtual_joystick import NAME
    name = Path('/sys/class/input') / Path(device).resolve().name / 'device/name'
    if (not name.exists() or name.read_text().strip() != NAME or
            (joystick.axes, joystick.buttons) != (9, 3)):
        raise ValueError('UAV AETR profile requires the matching virtual Pocket device')


def switches_for(profile, return_button=None):
    if profile in ('uav-aetr-v1', 'pocket-rc-v2'):
        if return_button is not None:
            raise ValueError('Matrix profiles do not accept a legacy return-button binding')
        return PocketUavSwitches(task_button_map={1:'pause',2:'cancel'} if profile == 'pocket-rc-v2' else None)
    if profile == 'legacy-usb':
        return PocketSwitches(return_button=return_button)
    raise ValueError('Unknown Pocket profile: '+profile)


def pause_intent(node):
    return 'RESUME' if node.phase == 'PAUSED' else 'PAUSE'


def observe(device, return_button=None, profile='legacy-usb'):
    joystick, switches = Joystick(device), switches_for(profile, return_button)
    try:
        validate_virtual_device(device, joystick, profile)
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


def manual_command_allowed(command):
    return command in ('ARM', 'DISARM', 'MANUAL', 'ACRO', 'VIO_POSITION', 'ASSISTED_POSITION',
                       'FLOW_POSITION', 'STABILIZED', 'ALTITUDE', 'LAND')


def control(session, device, return_button=None, profile='legacy-usb', manual_only=False):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from uav_mission.px4_flight import FlightServer
    from uav_nav_interfaces.srv import SetMissionIntent
    session, manifest = bind_session(session)
    manual_only = manual_only or manifest.get('manual_only', False)
    if manifest.get('input_profile') and manifest['input_profile'] != profile:
        raise ValueError('Input profile does not match the owned session manifest')
    from pocket_input import preflight_device
    preflight_device(device, profile)
    if profile in ('uav-aetr-v1', 'pocket-rc-v2'):
        overrides = manifest.get('px4_parameter_overrides', {})
        if overrides.get('PX4_PARAM_MPC_LAND_SPEED') != '0.3' or overrides.get('PX4_PARAM_MPC_LAND_CRWL') != '0.2':
            raise ValueError('Restart the owned USB session with slow native landing parameters')
    if manifest.get('flight_recipe_sha256'):
        raise ValueError('Pocket owner requires a manual session without another flight supervisor')
    os.environ.update(ROS_DOMAIN_ID='78', ROS_LOCALHOST_ONLY='1', UAV_POCKET_OPERATOR='1',
        UAV_SITL_AUTHORIZATION=str(uuid.uuid4()), UAV_WORKSPACE=str(ROOT), GZ_PARTITION=manifest['partition'])
    os.environ.update(operator_environment(session, manifest))
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'], signal_handler_options=SignalHandlerOptions.NO)
    from vio_observer_gc import ObservationGcGuard
    observation_gc=ObservationGcGuard(manifest.get('localization_profile')=='VIO_FLOW')
    node = FlightServer()
    executor = SingleThreadedExecutor(); executor.add_node(node)
    joystick, switches = Joystick(device), switches_for(profile, return_button)
    validate_virtual_device(device, joystick, profile)
    intent = node.create_client(SetMissionIntent, '/uav/bt/set_intent')
    runner = None
    runner_log = None
    parameters = session/'pocket-bt.private.json'
    warming = None
    requests = []
    pending = None
    pending_verb = None
    last_task_admission = None
    cancel_manual_root = None
    cancel_manual_admitted = False
    outcomes = []
    stopping = False
    manual_prepared = False
    previous_arming = None
    auto_prepared = False
    intervention_requested = False
    resume_when_paused = False
    previous_navigation = switches.navigation
    heartbeat=0.
    hid_events = 0
    hid_last_event = None
    hid_error = None
    def input_evidence(now):
        native = getattr(node,'manual_input_evidence',None)
        return dict(native=native.snapshot(now) if native is not None else None,
            hid=dict(events=hid_events,last_event_age_s=None if hid_last_event is None else now-hid_last_event,
                device=device,exists=Path(device).exists(),error=hid_error,
                scope='event-driven HID; no events at rest is normal, not a link freshness test'))
    def warm_next_runner():
        nonlocal warming
        if manual_only:return
        if os.environ.get('UAV_FLIGHT_REGION_PROFILE')!='vio_avoidance':return
        from bt_flight_client import prewarm_runner
        number=len(outcomes)+1
        warming=prewarm_runner(node,session,ROOT,parameters,
            log_name=f'pocket-bt-{number}.log',tree_name=f'pocket-tree-{number}.xml',
            wait_seconds=float(manifest.get('duration_s') or 3600.))
    warm_next_runner()
    def stop_requested(*_):
        nonlocal stopping
        stopping = True
    signal.signal(signal.SIGINT, stop_requested)
    signal.signal(signal.SIGTERM, stop_requested)

    def task_intent(verb):
        nonlocal pending, pending_verb
        if node.active_goal is None or not intent.service_is_ready():
            return False, 'NO_ACTIVE_TASK_OR_RUNNER'
        if pending is not None and not pending.done():
            if verb != 'END_MISSION':return False, 'REQUEST_PENDING'
            # Retire only the local receipt; the runner/gateway independently
            # arbitrate the in-flight command. SD cannot be dropped behind it.
            pending.cancel()
        request = SetMissionIntent.Request(mission_uuid=node.active_goal.goal_id,
            coordinator_instance=node.instance, command=verb)
        request.request_id.uuid = list(uuid.uuid4().bytes)
        pending = intent.call_async(request)
        pending_verb = verb
        return True, verb+'_REQUESTED'

    def start_task():
        nonlocal runner, runner_log, warming
        if manual_only:return False, 'MANUAL_ONLY_SESSION'
        if runner is not None and runner.poll() is None:
            return False, 'RUNNER_STILL_ACTIVE'
        if node.active_goal is not None or node.owner != 'NONE' or node.phase == 'FAULT':
            return False, 'TASK_BUSY_OR_LATCHED_FAULT'
        if not switches.arm or not switches.navigation or not node.operator_input_ready():
            return False, 'SA_SB_OR_QGC_NOT_READY'
        if (not node.healthy(ground=True) or not node.samples['vehicle_land_detected'].landed or
                node.samples['vehicle_status'].arming_state != 2 or node.speed() > .1):
            return False, 'GROUND_ARMED_STILL_REQUIRED'
        if not source_admitted(session,manifest):return False,'SOURCE_PREFLIGHT_PENDING'
        if os.environ.get('UAV_FLIGHT_REGION_PROFILE')=='vio_avoidance' and (warming is None or warming[0].poll() is not None):
            return False,'PREWARM_RUNNER_UNAVAILABLE_RESTART_SESSION'
        request=task_parameters(node,manifest)
        staging=session/'pocket-bt.pending.private.json'
        fd = os.open(staging, os.O_WRONLY|os.O_CREAT|os.O_EXCL, 0o600)
        with os.fdopen(fd, 'w') as stream:
            json.dump(request, stream)
        os.replace(staging,parameters)
        if runner_log is not None:
            runner_log.close()
        number = len(outcomes)+1
        runner_log = warming[1] if warming else (session/f'pocket-bt-{number}.log').open('w')
        runner = warming[0] if warming else subprocess.Popen([str(ROOT/'.deps/mission-install/uav_bt/lib/uav_bt/mission_runner'),
            '--ros-args', '-p', f'parameters_file:={parameters}', '-p', f'coordinator_instance:={node.instance}',
            '-p', f'timeout_s:={node.config.get("mission_timeout_max_s",240.)}', '-p', f'tree_output_file:={session / ("pocket-tree-"+str(number)+".xml")}'],
            stdout=runner_log, stderr=subprocess.STDOUT)
        warming=None
        return True, 'GROUND_TASK_DISPATCH_REQUESTED'

    try:
        print(f'Pocket owner ready: {profile}. QGC calibration/fresh input required.', flush=True)
        while rclpy.ok() and not stopping:
            executor.spin_once(timeout_sec=.01)
            now = time.monotonic()
            status = node.samples.get('vehicle_status')
            arming = getattr(status, 'arming_state', None)
            if previous_arming == 2 and arming == 1:
                # Native landing does not necessarily leave AUTO_TAKEOFF.
                # Prepare the operator's manual mode again after each disarm.
                manual_prepared = False
            previous_arming = arming
            for _ in range(64):
                event = joystick.read()
                if event is None:
                    break
                hid_events += 1
                hid_last_event = now
                switches.receive(event, now)
            node.operator_arm_enabled = switches.arm
            node.operator_navigation_enabled = switches.navigation and not manual_only
            if now-heartbeat>=1.:
                status=node.samples.get('vehicle_status');land=node.samples.get('vehicle_land_detected')
                source_preflight_admitted=source_admitted(session,manifest)
                source_ready=node.vio_healthy()
                gate=getattr(node,'vio_gate',None)
                state=dict(system_ns=time.time_ns(),source_admitted=source_preflight_admitted and source_ready,
                    source_preflight_admitted=source_preflight_admitted,source_ready=source_ready,
                    source_reason=getattr(gate,'reason',''),
                    manual_only=manual_only,
                    input_profile=profile, requested_mode=getattr(switches,'requested_mode','MANUAL'),
                    safety_held=getattr(switches,'safety_held',False),
                    operator_input_ready=node.operator_input_ready(),auto_prepared=auto_prepared,
                    owner=node.owner,phase=node.phase,arming=getattr(status,'arming_state',None),
                    ground_state_fresh=node.fresh('vehicle_status', .75) and node.fresh('vehicle_land_detected', 1.2),
                    canceled_manual_state=getattr(node,'cancel_manual_state',None),
                    nav_state=getattr(status,'nav_state',None),landed=getattr(land,'landed',None))
                state['input_evidence'] = input_evidence(now)
                state['last_operator_request'] = requests[-1] if requests else None
                state['last_task_admission'] = last_task_admission
                (session/'pocket-live.pending.json').write_text(json.dumps(state)+'\n')
                os.replace(session/'pocket-live.pending.json',session/'pocket-live.json')
                heartbeat=now
            # Apply the initial manual mode only after QGC is ready, on the ground
            # and disarmed. This never arms or starts a task.
            admitted=source_admitted(session,manifest)
            prepared = prepare_manual(node, switches, manual_prepared) if admitted else manual_prepared
            if prepared and not manual_prepared:
                print('Initial manual mode requested; actual PX4 mode confirms the transition', flush=True)
            manual_prepared = prepared
            if pending is not None and pending.done():
                response = pending.result()
                last_task_admission = dict(mono=now,accepted=response.accepted,
                    reason=response.reason,phase=response.phase,command=pending_verb)
                print(f'Task admission: {response.accepted} {response.reason}', flush=True)
                if not response.accepted:
                    intervention_requested=resume_when_paused=False
                if pending_verb=='CANCEL':
                    cancel_manual_admitted=response.accepted
                    if not response.accepted:cancel_manual_root=None
                pending = None
                pending_verb = None
            if node.phase=='PAUSED' and pending is None:
                if resume_when_paused:
                    accepted,reason=task_intent('RESUME')
                    resume_when_paused=False
                    print(f'Intervention resume: {accepted} {reason}',flush=True)
                elif intervention_requested and not node.pilot_control:
                    accepted,reason=node.enter_pilot_control()
                    intervention_requested=accepted
                    print(f'Intervention handoff: {accepted} {reason}',flush=True)
            if runner is not None and runner.poll() is not None:
                outcomes.append(dict(exit_code=runner.returncode, result=node.result))
                print(f'Root terminal: exit={runner.returncode} result={node.result}', flush=True)
                runner = None
                if cancel_manual_root is not None and cancel_manual_admitted:
                    accepted,reason=node.begin_canceled_manual(cancel_manual_root)
                    print(f'Canceled manual handoff: {accepted} {reason}',flush=True)
                cancel_manual_root=None
                cancel_manual_admitted=False
                auto_prepared = False
                parameters.unlink(missing_ok=True)
                # Keep DDS membership fixed after VIO binding. A new root needs
                # a new owned session, with its runner prewarmed before binding.
            for command in switches.tick(now):
                node.operator_arm_enabled = switches.arm
                node.operator_navigation_enabled = switches.navigation and not manual_only
                if manual_only and not manual_command_allowed(command):
                    accepted,reason=False,'MANUAL_ONLY_SESSION'
                elif not admitted and command in ('ARM','PREPARE','RUN','INTERVENE','MANUAL','ACRO','VIO_POSITION','ASSISTED_POSITION','FLOW_POSITION','RTL'):
                    accepted,reason=False,'SOURCE_PREFLIGHT_PENDING'
                elif command in ('ARM','DISARM','MANUAL','ACRO','VIO_POSITION','ASSISTED_POSITION','FLOW_POSITION','STABILIZED','ALTITUDE','LAND','RTL'):
                    if command != 'ARM':
                        auto_prepared = False
                        intervention_requested = resume_when_paused = False
                    accepted, reason = (arm_from_switch(node, switches,task_prepared=auto_prepared) if command == 'ARM'
                                        else node.operator_command(command))
                elif command == 'PREPARE':
                    runner_ready=(os.environ.get('UAV_FLIGHT_REGION_PROFILE')!='vio_avoidance'
                        or (warming is not None and warming[0].poll() is None))
                    accepted,reason=prepare_task(node,switches,manifest,runner_ready=runner_ready)
                    auto_prepared=accepted
                elif command == 'RUN':
                    was_intervention=intervention_requested or bool(node.pilot_control)
                    intervention_requested=False
                    verb = run_intent(node)
                    if was_intervention and node.phase in ('PAUSING','HOLD_HANDOFF'):
                        resume_when_paused=True
                        accepted,reason=True,'RESUME_AFTER_STOP_REQUESTED'
                    elif profile in ('uav-aetr-v1','pocket-rc-v2') and not auto_prepared and verb is None:
                        accepted, reason = False, 'TASK_PREPARATION_REQUIRED'
                    else:
                        accepted, reason = task_intent(verb) if verb else start_task()
                elif command == 'INTERVENE':
                    accepted,reason=node.pilot_request_ready()
                    if accepted:
                        if node.pilot_control:reason='INTERVENTION_ALREADY_REQUESTED'
                        elif node.phase!='PAUSED':accepted,reason=task_intent('PAUSE')
                        intervention_requested=accepted
                        resume_when_paused=False
                elif command == 'PAUSE':
                    accepted, reason = task_intent('PAUSE')
                elif command == 'TOGGLE_RETURN':
                    accepted, reason = task_intent(return_intent(node))
                elif command == 'END_MISSION':
                    auto_prepared = False
                    intervention_requested = resume_when_paused = False
                    cancel_manual_root=None
                    cancel_manual_admitted=False
                    accepted,reason=end_mission_intent(node,profile,task_intent)
                elif command == 'TOGGLE_PAUSE':
                    if intervention_requested or node.pilot_control:
                        accepted,reason=False,'INTERVENTION_USE_SC_EXECUTE_TO_RESUME'
                    else:accepted, reason = task_intent(pause_intent(node))
                elif command == 'CANCEL':
                    auto_prepared = False
                    intervention_requested = resume_when_paused = False
                    accepted, reason = task_intent('CANCEL')
                    if accepted:
                        cancel_manual_root=bytes(node.active_goal.goal_id.uuid)
                        cancel_manual_admitted=False
                elif command == 'BLOCKED_SAFETY_HELD':
                    accepted, reason = False, 'SD_SAFETY_HELD_RELEASE_BEFORE_CONTROL'
                else:
                    accepted, reason = False, 'SA_OR_SB_NOT_ENABLED'
                row = dict(mono=now, command=command, accepted=accepted, reason=reason)
                requests.append(row); print(json.dumps(row), flush=True)
            if switches.navigation != previous_navigation:
                if not switches.navigation:
                    auto_prepared = False
                previous_navigation = switches.navigation
    except (KeyboardInterrupt, OSError) as error:
        if isinstance(error,OSError):hid_error=dict(errno=error.errno,message=str(error),system_ns=time.time_ns())
        print(f'Pocket owner stopping: {error}', flush=True)
    finally:
        if warming is not None:
            if warming[0].poll() is None:warming[0].terminate()
            warming[0].wait(timeout=5);warming[1].close()
        if runner is not None and runner.poll() is None:
            runner.send_signal(signal.SIGTERM)
            until = time.monotonic()+10
            while runner.poll() is None and time.monotonic()<until:
                executor.spin_once(timeout_sec=.02)
            if runner.poll() is None:
                node.fault('USB_OPERATOR_STOP_UNCONFIRMED')
                runner.kill(); runner.wait(timeout=3)
        # Shutdown is not an operator request to remain flying. A pending manual
        # handoff still owns Offboard; close it through the normal LAND fallback.
        if getattr(node,'cancel_manual_state',None) in ('WAIT_CENTER','POSITION_REQUEST'):
            node.cancel_manual_state=None
            node.final_hold_until=time.monotonic()
        # Keep the sole gateway alive for its bounded post-cancel hold/landing.
        until = time.monotonic()+60
        while node.owner in ('TASK','HOLD_CONTROLLER','AUTOPILOT') and time.monotonic()<until:
            executor.spin_once(timeout_sec=.02)
        snapshot = dict(requests=requests, outcomes=outcomes, final_owner=node.owner,
            input_profile=profile,
            input_evidence=input_evidence(time.monotonic()),
            last_task_admission=last_task_admission,
            phase=node.phase, result=node.result, commands=node.commands,
            scope='Pocket operator session; physical state remains separate from request admission')
        (session/'pocket-operator.json').write_text(json.dumps(snapshot, indent=2)+'\n')
        parameters.unlink(missing_ok=True)
        if runner_log is not None:
            runner_log.close()
        observation_gc.restore()
        joystick.close(); executor.shutdown(); node.destroy_node(); rclpy.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--device', default=DEVICE)
    parser.add_argument('--input-profile', choices=['legacy-usb', 'uav-aetr-v1', 'pocket-rc-v2'], default='pocket-rc-v2')
    parser.add_argument('--session', help='Owned manual SITL evidence directory')
    parser.add_argument('--return-button', type=int, choices=[0], help='Explicitly enable the verified Linux momentary button 0; default unassigned')
    parser.add_argument('--enable-sitl-control', action='store_true')
    parser.add_argument('--manual-only', action='store_true', help='Reject automatic task commands in this operator session')
    args = parser.parse_args()
    if args.input_profile == 'uav-aetr-v1':
        if args.device == DEVICE or args.return_button is not None:
            parser.error('UAV virtual profile requires explicit --device and no --return-button')
    if not args.enable_sitl_control:
        return observe(args.device, args.return_button, args.input_profile)
    if not args.session:
        parser.error('--enable-sitl-control requires --session')
    session, _ = bind_session(args.session)
    with (session/'pocket-operator.lock').open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX|fcntl.LOCK_NB)
        control(session, args.device, args.return_button, args.input_profile, manual_only=args.manual_only)


if __name__ == '__main__':
    main()
