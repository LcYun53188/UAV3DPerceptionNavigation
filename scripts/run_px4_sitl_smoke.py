#!/usr/bin/env python3
"""Owned, disarmed x500 smoke session. Run with scripts/with_px4_sim.sh.

Checks DDS samples/clock and QGC log evidence. --flight explicitly enables tasks.
"""
import argparse
import fcntl
import math
import re
from datetime import datetime, timezone
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleStatus, VehicleOdometry, VehicleLocalPosition, VehicleLandDetected
from sim_validation import ROOT, file_hash, read, write_json
from prepare_px4_sim import ensure_source, check_external


def stop(process):
    # Retire every member of our process group, even if its root exited early.
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=8)
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    process.wait(timeout=5)


def free_port(port):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', port))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vision-fusion-profile',choices=('full_odometry','aligned_pose_v1'),default='full_odometry')
    parser.add_argument('--vision-fusion-smoke', action='store_true', help='Separate disarmed build: synthetic EV input and actual EKF fusion telemetry')
    parser.add_argument('--vio-fusion-profile',choices=('full_odometry','aligned_pose_v1'),default='full_odometry',help='Explicit VIO admission profile; existing W0 scene authorization still required')
    parser.add_argument('--require-vio', action='store_true', help='Require VIO source and actual EKF fusion telemetry for flight admission')
    parser.add_argument('--vio-calibration-id', default='', help='Reviewed VIO calibration/config SHA256')
    parser.add_argument('--depth-camera', action='store_true', help='Audit pinned x500_depth camera, disarmed only')
    parser.add_argument('--duration', type=float, default=45)
    parser.add_argument('--mission-file', type=Path, help='Custom W0 JSON recipe; NAVIGATE offset_enu is relative to launch')
    parser.add_argument('--bt', action='store_true', help='Execute flight through BehaviorTree.CPP runner')
    parser.add_argument('--ui', action='store_true', help='Show the owned Gazebo and QGC windows')
    parser.add_argument('--flight-scenario', choices=['full','pause-resume','cancel','clock-fault','runner-exit','runner-stall','odometry-stale','odometry-reset'], default='full')
    parser.add_argument('--flight', action='store_true', help='Run real known-region flight mission after disarmed smoke')
    parser.add_argument('--aircraft-state', action='store_true',
                        help='Also validate the S1 observer and source/clock loss')
    args = parser.parse_args()
    if args.vio_fusion_profile!='full_odometry' and not args.require_vio:
        parser.error('--vio-fusion-profile requires --require-vio')
    if args.vision_fusion_profile!='full_odometry' and not args.vision_fusion_smoke:
        parser.error('--vision-fusion-profile requires --vision-fusion-smoke')
    if args.vision_fusion_smoke and (args.flight or args.bt or args.depth_camera or args.aircraft_state or args.mission_file or args.flight_scenario != 'full'):
        parser.error('--vision-fusion-smoke is an independent disarmed audit')
    if args.require_vio and (not args.flight or not re.fullmatch('[0-9a-f]{64}', args.vio_calibration_id)):
        parser.error('--require-vio needs --flight and --vio-calibration-id SHA256')
    if args.vio_calibration_id and not args.require_vio:
        parser.error('--vio-calibration-id requires --require-vio')
    if args.depth_camera and (args.flight or args.bt or args.mission_file or args.flight_scenario != 'full'):
        parser.error('--depth-camera is a disarmed profile and cannot use flight options')
    if args.bt and not args.flight:
        parser.error('--bt requires --flight')
    if args.flight_scenario in ('runner-exit','runner-stall') and not args.bt:
        parser.error('Runner fault scenarios require --bt')
    if not 10 <= args.duration <= 180:
        parser.error('duration must be within [10, 180] seconds')
    lock = read(ROOT / 'simulation/px4/versions.lock.yaml')
    for key in ['sitl', 'agent', 'px4_msgs']:
        ensure_source(lock[key])
    check_external(lock)
    for artifact in lock.get('build_artifacts', []):
        if file_hash(ROOT / artifact['path']) != artifact['sha256']:
            raise RuntimeError(f"Build artifact differs from lock: {artifact['path']}")
    # Dedicated instance; domain/partition are distinct from the algorithm session.
    instance, domain, xrce_port = 7, 78, 8898
    for port in [xrce_port, 18570 + instance, 14580 + instance, 14550]:
        free_port(port)
    if Path(f'/tmp/px4_lock-{instance}').exists():
        raise RuntimeError('PX4 instance lock already exists; inspect before starting')
    run_id = str(uuid.uuid4())
    run_dir = ROOT / '.cache/simulation/sitl' / run_id
    run_dir.mkdir(parents=True)
    (run_dir / 'rootfs').mkdir()
    px4 = ROOT / lock['sitl']['path']
    build = px4 / 'build/px4_sitl_default'
    if args.vision_fusion_smoke or args.require_vio:
        from build_px4_vio import BUILD, verify
        vio_build = verify(lock)
        build = BUILD
    models = px4 / 'Tools/simulation/gz/models'
    worlds = px4 / 'Tools/simulation/gz/worlds'
    model_name = 'x500_depth' if args.depth_camera else 'x500'
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain), ROS_LOCALHOST_ONLY='1',
               GZ_PARTITION=f'uav_px4_s0_{run_id}', GZ_IP='127.0.0.1',
               PX4_SIM_MODEL=f'gz_{model_name}', PX4_GZ_STANDALONE='1', PX4_GZ_WORLD='default',
               HEADLESS='1', PX4_UXRCE_DDS_PORT=str(xrce_port), PX4_PARAM_UXRCE_DDS_SYNCT='0',
               PX4_GZ_MODELS=str(models), PX4_GZ_WORLDS=str(worlds),
               GZ_SIM_RESOURCE_PATH=f"{models}:{worlds}:" + os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
               GZ_SIM_SYSTEM_PLUGIN_PATH=str(build / 'src/modules/simulation/gz_plugins') + ':' + os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
               GZ_SIM_SERVER_CONFIG_PATH=str(ROOT / 'simulation/px4/server_control.config'))
    if args.vision_fusion_smoke:
        env.update(PX4_PARAM_EKF2_EV_CTRL='11' if args.vision_fusion_profile=='aligned_pose_v1' else '15', PX4_PARAM_EKF2_GPS_CTRL='0',
                   PX4_PARAM_EKF2_MAG_TYPE='5', PX4_PARAM_EKF2_HGT_REF='3',
                   PX4_PARAM_SENS_IMU_MODE='0', PX4_PARAM_EKF2_MULTI_IMU='1',
                   PX4_PARAM_EKF2_MULTI_MAG='0')
        if args.vision_fusion_profile=='aligned_pose_v1':
            profile=read(ROOT/'simulation/px4/vio/pose_fusion.json')
            if profile['schema']!=1 or profile['profile']!=args.vision_fusion_profile:
                raise RuntimeError('Unsupported pose fusion profile')
            env.update({'PX4_PARAM_'+name:str(value) for name,value in profile['parameters'].items()})
    if args.flight:
        env.update(UAV_SITL_AUTHORIZATION=str(uuid.uuid4()),
                   UAV_REQUIRE_VIO='1' if args.require_vio else '0',
                   UAV_VIO_CALIBRATION_ID=args.vio_calibration_id, UAV_WORKSPACE=str(ROOT),
                   UAV_FLIGHT_MISSION_FILE=str(args.mission_file.resolve()) if args.mission_file else '',
                   UAV_FLIGHT_EVIDENCE=str(run_dir), UAV_FLIGHT_SCENARIO=args.flight_scenario,
                   UAV_FLIGHT_BT='1' if args.bt else '0', UAV_VIO_FUSION_PROFILE=args.vio_fusion_profile, PX4_PARAM_COM_RC_IN_MODE='4',
                   PX4_PARAM_COM_OF_LOSS_T='0.5', PX4_PARAM_COM_OBL_RC_ACT='4', PX4_PARAM_COM_DISARM_LAND='2', PX4_PARAM_EKF2_MAG_TYPE='6')
    if args.require_vio and args.vio_fusion_profile=='aligned_pose_v1':
        pose_profile=read(ROOT/'simulation/px4/vio/pose_fusion.json')
        if pose_profile['schema']!=1 or pose_profile['profile']!=args.vio_fusion_profile:
            raise RuntimeError('Unsupported flight VIO profile')
        env.update({'PX4_PARAM_'+name:str(value) for name,value in pose_profile['parameters'].items()})
    manifest = dict(run_id=run_id, started_at=datetime.now(timezone.utc).isoformat(),
                    domain=domain, partition=env['GZ_PARTITION'], instance=instance,
                    xrce_port=xrce_port, gcs_port=14550, px4_gcs_local_port=18577,
                    model=f'{model_name}_7', namespace='/px4_7', versions=lock,
                    flight_recipe_sha256=file_hash(args.mission_file.resolve() if args.mission_file else ROOT/'simulation/missions/W0_flight_sequence.json') if args.flight else None,
                    vio_fusion_profile=args.vio_fusion_profile,flight_profile_sha256=file_hash(ROOT/'simulation/safe_regions/W0.json') if args.flight else None,
                    px4_parameter_overrides={k:v for k,v in env.items() if k.startswith('PX4_PARAM_')},
                    tool_sha256=file_hash(Path(__file__)),
                    world_sha256=file_hash(worlds / 'default.sdf'),
                    server_config_sha256=file_hash(ROOT / 'simulation/px4/server_control.config'),
                    model_sha256=file_hash(models / f'{model_name}/model.sdf'),
                    hardware_camera='OAK-D Pro W', require_vio=args.require_vio,
                    vision_fusion_smoke=args.vision_fusion_smoke,
                    vision_fusion_profile=args.vision_fusion_profile,
                    pose_fusion_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in (ROOT/'simulation/px4/vio/pose_fusion.json',
                        ROOT/'src/px4_comm_bridge/px4_comm_bridge/pose_fusion.py')} if args.vision_fusion_profile=='aligned_pose_v1' else None,
                    vio_build=vio_build if args.vision_fusion_smoke or args.require_vio else None,
                    vision_audit_sha256=file_hash(ROOT/'scripts/px4_vision_audit.py') if args.vision_fusion_smoke else None,
                    vio_gate_sha256=file_hash(ROOT/'src/uav_mission/uav_mission/vio_gate.py') if args.vision_fusion_smoke else None,
                    vio_calibration_id=args.vio_calibration_id or None,
                    sensor_profile='px4-reference-oakd-lite-disarmed' if args.depth_camera else None,
                    sensor_topics={'depth': '/depth_camera', 'camera_info': '/camera_info'} if args.depth_camera else {},
                    model_dependencies_sha256={name: file_hash(models / name / 'model.sdf')
                                               for name in ('x500', 'x500_base', 'OakD-Lite')} if args.depth_camera else {},
                    depth_audit_sha256=file_hash(ROOT / 'scripts/px4_depth_audit.py') if args.depth_camera else None,
                    processes={})
    write_json(run_dir / 'manifest.json', manifest)
    processes, logs, managed = [], [], {}

    def launch(name, command, process_env=env):
        log = (run_dir / f'{name}.log').open('w')
        logs.append(log)
        process = subprocess.Popen(command, cwd=run_dir, env=process_env, stdout=log,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        managed[name] = process
        manifest['processes'][name] = dict(pid=process.pid, command=[str(v) for v in command])
        write_json(run_dir / 'manifest.json', manifest)
        return process

    os.environ['ROS_DOMAIN_ID'] = str(domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    rclpy.init()
    node = rclpy.create_node('px4_s0_observer')
    samples, last, received = {}, {}, {}
    arming_states = set()

    def callback(name):
        def receive(msg):
            samples[name] = samples.get(name, 0) + 1
            last[name] = msg
            received[name] = time.monotonic()
            if name == 'vehicle_status':
                arming_states.add(int(msg.arming_state))
        return receive

    for topic, kind in [('vehicle_status', VehicleStatus), ('vehicle_odometry', VehicleOdometry),
                        ('vehicle_local_position', VehicleLocalPosition), ('vehicle_land_detected', VehicleLandDetected)]:
        version = kind.MESSAGE_VERSION
        name = f'/px4_{instance}/fmu/out/{topic}' + (f'_v{version}' if version else '')
        node.create_subscription(kind, name, callback(topic), qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock', callback('clock'), qos_profile_sensor_data)
    if args.aircraft_state:
        from uav_nav_interfaces.msg import AircraftState
        node.create_subscription(AircraftState, '/aircraft_state', callback('aircraft_state'), 10)
    depth_audit = None
    if args.depth_camera:
        from px4_depth_audit import DepthAudit
        from sensor_msgs.msg import Image, CameraInfo
        depth_audit = DepthAudit()
        node.create_subscription(Image, '/px4_depth/image',
                                 lambda m: depth_audit.image(m, time.monotonic()), qos_profile_sensor_data)
        node.create_subscription(CameraInfo, '/px4_depth/camera_info',
                                 lambda m: depth_audit.info(m, time.monotonic()), qos_profile_sensor_data)
    vision_audit = None
    if args.vision_fusion_smoke:
        from px4_vision_audit import VisionFusionAudit
        vision_audit = VisionFusionAudit(node,args.vision_fusion_profile)
    first_clock = None
    failure = None
    try:
        launch('agent', [ROOT / '.deps/microxrce-install/bin/MicroXRCEAgent', 'udp4', '-p', str(xrce_port)])
        launch('gazebo', ['gz', 'sim', '-r', '-s', worlds / 'default.sdf'])
        if args.ui:
            launch('gazebo_gui', ['gz', 'sim', '-g'])
        launch('clock_bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                               '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])
        if args.depth_camera:
            launch('depth_bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                                   '/depth_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                                   '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                                   '--ros-args', '-r', '/depth_camera:=/px4_depth/image',
                                   '-r', '/camera_info:=/px4_depth/camera_info'])
        launch('px4', [build / 'bin/px4', '-d', '-i', str(instance), '-w', run_dir / 'rootfs', build / 'etc'])
        config = run_dir / 'qgc-config/QGroundControl'
        config.mkdir(parents=True)
        (config / 'QGroundControl.ini').write_text('[AutoConnect]\nautoConnectUDP=true\nautoConnectPixhawk=false\nautoConnectSiKRadio=false\nautoConnectRTKGPS=false\nautoConnectLibrePilot=false\n')
        qgc_env = dict(env, XDG_CONFIG_HOME=str(run_dir / 'qgc-config'),
                       XDG_CACHE_HOME=str(run_dir / 'qgc-cache'),
                       QT_QPA_PLATFORM=os.environ.get('QT_QPA_PLATFORM', 'xcb') if args.ui else 'offscreen')
        launch('qgc', [lock['artifacts']['qgc']['path'], '--allow-multiple', '--log-output',
                       '--logging', 'Vehicle.MultiVehicleManager,Vehicle.VehicleLinkManager'], qgc_env)
        if args.aircraft_state:
            launch('aircraft_state', [sys.executable, '-c',
                   'from uav_mission.aircraft_state_node import main; main()',
                   '--ros-args', '-p', 'use_sim_time:=true', '-p', 'px4_namespace:=/px4_7'])
        deadline = time.monotonic() + args.duration
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            if vision_audit is not None:
                vision_audit.tick(last['clock'].clock if 'clock' in last else None)
            if first_clock is None and 'clock' in last:
                c = last['clock'].clock
                first_clock = c.sec + c.nanosec / 1e9
            if any(p.poll() is not None for p in processes):
                raise RuntimeError('A managed process exited; inspect logs')
        writers = {name: len(node.get_publishers_info_by_topic(name))
                   for name, types in node.get_topic_names_and_types() if name == '/clock' or name.startswith(f'/px4_{instance}/fmu/out/')}
        status = last.get('vehicle_status')
        local = last.get('vehicle_local_position')
        landed = last.get('vehicle_land_detected')
        c = last.get('clock')
        clock_end = c.clock.sec + c.clock.nanosec / 1e9 if c else None
        qgc_log = (run_dir / 'qgc.log').read_text(errors='replace')
        qgc_connected = bool(re.search(r'Adding new vehicle.*\"UDP Link \(AutoConnect\)\" 8 1 12 2', qgc_log))
        odom = last.get('vehicle_odometry')
        source_age = clock_end - odom.timestamp / 1e6 if odom and clock_end is not None else None
        result = dict(qgc_connected=qgc_connected, arming_states=sorted(arming_states),
                      odom_source_age_s=source_age, scope=f'disarmed {model_name}/DDS/clock/QGC smoke only', samples=dict(samples),
                      receive_age_s={k: time.monotonic() - v for k, v in received.items()}, writers=writers,
                      arming_state=status.arming_state if status else None,
                      nav_state=status.nav_state if status else None,
                      landed=landed.landed if landed else None,
                      xy_valid=local.xy_valid if local else None, z_valid=local.z_valid if local else None,
                      clock_start=first_clock, clock_end=clock_end,
                      position_ned=[local.x, local.y, local.z] if local else None)
        result['dds_clock_passed'] = (all(samples.get(k, 0) >= 5 for k in ['vehicle_status', 'vehicle_odometry', 'vehicle_local_position', 'vehicle_land_detected', 'clock'])
            and arming_states == {VehicleStatus.ARMING_STATE_DISARMED} and landed.landed
            and local.xy_valid and local.z_valid and clock_end > first_clock
            and writers.get('/clock') == 1 and all(v == 1 for v in writers.values())
            and all(v < 1 for v in result['receive_age_s'].values()))
        result['passed'] = result['dds_clock_passed'] and qgc_connected and source_age is not None and math.isfinite(source_age) and abs(source_age) < .5
        if depth_audit is not None:
            depth_result = depth_audit.result(time.monotonic(), clock_end,
                {name: len(node.get_publishers_info_by_topic(name))
                 for name in ('/px4_depth/image', '/px4_depth/camera_info')})
            write_json(run_dir / 'depth-camera.json', depth_result)
            result['depth_camera_passed'] = depth_result['passed']
            result['passed'] = result['passed'] and depth_result['passed']
        if vision_audit is not None:
            vision_audit.stop_input()
            until = time.monotonic()+6
            while time.monotonic()<until:
                rclpy.spin_once(node,timeout_sec=.05)
                vision_audit.tick(last['clock'].clock if 'clock' in last else None)
            vision_result = vision_audit.result()
            write_json(run_dir / 'vision-fusion.json', vision_result)
            result['vision_fusion_passed'] = vision_result['passed']
            result['arming_states'] = sorted(arming_states)
            result['passed'] = result['passed'] and vision_result['passed'] and arming_states == {VehicleStatus.ARMING_STATE_DISARMED}
        if args.flight and result['passed']:
            flight = launch('flight_tasks', [sys.executable, ROOT / 'scripts/run_px4_flight_tasks.py'])
            until = time.monotonic() + 245
            next_log,log_offset=0.,0
            while flight.poll() is None and time.monotonic() < until:
                if time.monotonic()>=next_log:
                    with (run_dir/'flight_tasks.log').open() as progress:
                        progress.seek(log_offset)
                        for line in progress:
                            if '[px4_flight_gateway]:' in line:print(line.rstrip(),flush=True)
                        log_offset=progress.tell()
                    next_log=time.monotonic()+.2
                rclpy.spin_once(node, timeout_sec=.05)
            result['flight_passed'] = flight.poll() == 0
            result['passed'] = result['passed'] and result['flight_passed']
        if args.aircraft_state:
            def current_state():
                msg = last.get('aircraft_state')
                if msg is None:
                    return None
                return {key: dict(value=getattr(msg, key).value, valid=getattr(msg, key).valid,
                                  reason=getattr(msg, key).reason)
                        for key in ('arming', 'ground', 'mode', 'localization', 'navigation', 'link')}

            fresh_state = current_state()
            # Collect evidence at a fresh detector window; default detector rate is 1 Hz
            # while the configured freshness limit remains 0.5 s.
            until = time.monotonic() + 3
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.05)
                candidate = current_state()
                if candidate and candidate['arming']['valid'] and candidate['ground']['valid']:
                    fresh_state = candidate
                    break
            stop(managed['px4'])
            until = time.monotonic() + 1.8
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.05)
            source_lost = current_state()
            stop(managed['clock_bridge'])
            until = time.monotonic() + 1.2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.05)
            clock_lost = current_state()
            observer_passed = bool(fresh_state and source_lost and clock_lost
                and fresh_state['arming']['value'] == 'DISARMED'
                and fresh_state['ground']['value'] == 'ON_GROUND'
                and fresh_state['arming']['valid'] and fresh_state['ground']['valid']
                and not fresh_state['navigation']['valid']
                and not source_lost['arming']['valid']
                and source_lost['arming']['reason'] == 'STALE_SAMPLE'
                and not clock_lost['arming']['valid']
                and clock_lost['arming']['reason'] == 'ROS_TIME_STALLED'
                and time.monotonic() - received.get('aircraft_state', 0) < .5)
            result['aircraft_state'] = dict(fresh=fresh_state, source_lost=source_lost,
                                            clock_lost=clock_lost, passed=observer_passed)
            result['passed'] = result['passed'] and observer_passed
        write_json(run_dir / 'observation.json', result)
    except Exception as exc:
        failure = str(exc)
        write_json(run_dir / 'failure.json', dict(error=failure))
    finally:
        node.destroy_node()
        rclpy.shutdown()
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
        write_json(run_dir / 'cleanup.json', dict(root_exit_codes=[p.returncode for p in processes]))
    print(f'Evidence: {run_dir}', flush=True)
    if failure:
        print(f'FAIL: {failure}', flush=True)
        return 1
    print(f"DDS/clock: {result['dds_clock_passed']}; QGC: {result['qgc_connected']}; PASS: {result['passed']}", flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    guard = ROOT / '.cache/simulation/px4-smoke.lock'
    guard.parent.mkdir(parents=True, exist_ok=True)
    with guard.open('a') as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        raise SystemExit(main())
