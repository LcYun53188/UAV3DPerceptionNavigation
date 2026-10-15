#!/usr/bin/env python3
"""Owned disarmed Gazebo/PX4 MTF-01 flow/range audit; no serial or flight commands."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from sim_validation import ROOT, read, file_hash, write_json
from prepare_px4_sim import ensure_source, check_external
from run_px4_sitl_smoke import stop, free_port
from prepare_mtf01_assets import assets, PROFILE
from mtf01_audit import fields, assessment, parameters_match
from vio_render_device import environment, capture

TOPICS = ('sensor_optical_flow', 'vehicle_optical_flow', 'distance_sensor',
          'estimator_aid_src_optical_flow', 'estimator_aid_src_rng_hgt',
          'estimator_status_flags', 'vehicle_local_position', 'vehicle_status')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=45)
    parser.add_argument('--no-floor-texture', action='store_true', help='Explicit featureless-floor negative comparison')
    parser.add_argument('--ui', action='store_true', help='Show owned Gazebo viewer')
    parser.add_argument('--stop-sensors', action='store_true', help='After the healthy window, stop both owned adapters and audit 12 seconds of source loss')
    parser.add_argument('--flight', action='store_true', help='Explicit isolated FLOW_LOCAL takeoff/hover/native LAND, after successful disarmed audit; no EV/VIO source')
    parser.add_argument('--native-manual', action='store_true', help='With --flight: native Position and simulated RC sticks, no mission gateway or Offboard heartbeat')
    parser.add_argument('--usb-joystick', action='store_true', help='Persistent FLOW_LOCAL warehouse/Pocket session; no automatic arm or flight')
    parser.add_argument('--session-duration', type=float, default=3600, help='Operator session seconds after successful ground audit')
    parser.add_argument('--device', default='/dev/input/js0', help='Explicit virtual Pocket joystick for the operator session')
    parser.add_argument('--qgc-config-from', type=Path, help='Preserve calibration from an owned SITL QGC config')
    parser.add_argument('--render-device', choices=('default','nvidia','intel'), default='nvidia')
    args = parser.parse_args()
    if args.native_manual and not args.flight:
        parser.error('--native-manual requires --flight')
    if args.usb_joystick and (not args.ui or args.flight or args.native_manual or args.no_floor_texture or args.stop_sensors):
        parser.error('--usb-joystick requires --ui, textured continuous sensors and no automatic flight flags')
    if not 20 <= args.session_duration <= 3600:
        parser.error('session-duration must be within [20,3600] seconds')
    if args.qgc_config_from and (not args.usb_joystick or not args.qgc_config_from.is_file() or
            not any(args.qgc_config_from.resolve().is_relative_to(ROOT/'.cache/simulation'/name) for name in ('sitl','mtf01'))):
        parser.error('--qgc-config-from requires an owned QGC config and --usb-joystick')
    if args.flight and (args.no_floor_texture or args.stop_sensors):
        parser.error('Flight requires the reviewed textured floor and continuous sensor inputs')
    def interrupted(signum,frame):
        raise KeyboardInterrupt('signal '+str(signum))
    signal.signal(signal.SIGTERM,interrupted)
    if not 20 <= args.duration <= 120:
        parser.error('duration must be within [20,120] seconds')
    lock = read(ROOT/'simulation/px4/versions.lock.yaml')
    for key in ('sitl','agent','px4_msgs'):
        ensure_source(lock[key])
    check_external(lock)
    for artifact in lock['build_artifacts']:
        if file_hash(ROOT/artifact['path']) != artifact['sha256']:
            raise RuntimeError('Locked artifact drift: '+artifact['path'])
    mutex_path = ROOT/'.cache/simulation/px4-smoke.lock'
    mutex_path.parent.mkdir(parents=True, exist_ok=True)
    with mutex_path.open('a') as mutex:
        fcntl.flock(mutex, fcntl.LOCK_EX | fcntl.LOCK_NB)
        for port in (8898,18577,14587,14550):
            free_port(port)
        if Path('/tmp/px4_lock-7').exists():
            raise RuntimeError('PX4 instance 7 occupied')
        execute(args, lock)


def execute(args, lock):
    run_id = str(uuid.uuid4())
    out = ROOT/'.cache/simulation/mtf01'/run_id
    (out/'rootfs').mkdir(parents=True)
    px4 = ROOT/lock['sitl']['path']
    build = px4/'build/px4_sitl_default'
    build_receipt = None
    local_control = args.flight or args.usb_joystick
    if local_control:
        from build_px4_vio import BUILD, verify
        build_receipt = verify(lock)
        build = BUILD  # Exposes native EKF aid telemetry; does not start VIO.
    models = px4/'Tools/simulation/gz/models'
    profile = assets(out/'assets', px4/'Tools/simulation/gz/worlds/default.sdf', textured=not args.no_floor_texture)
    runtime=json.loads(subprocess.check_output(['/usr/bin/python3','-c',
        'import cv2,numpy,json,sys; print(json.dumps(dict(python=sys.version,opencv=cv2.__version__,numpy=numpy.__version__)))'],text=True))
    fusion_file = ROOT/('simulation/px4/mtf01/flow_local.json' if local_control else 'simulation/px4/mtf01/fusion.json')
    fusion = read(fusion_file)
    env = dict(os.environ, ROS_DOMAIN_ID='78', ROS_LOCALHOST_ONLY='1',
        GZ_DISTRO='harmonic', GZ_PARTITION='uav_mtf01_'+run_id, GZ_IP='127.0.0.1',
        PX4_SIM_MODEL='gz_'+profile['model'], PX4_SYS_AUTOSTART='4001',
        PX4_GZ_STANDALONE='1', PX4_GZ_WORLD='default', HEADLESS='1',
        PX4_UXRCE_DDS_PORT='8898', PX4_PARAM_UXRCE_DDS_SYNCT='0',
        PX4_GZ_MODELS=str(out/'assets'), PX4_GZ_WORLDS=str(out/'assets'),
        GZ_SIM_RESOURCE_PATH=str(out/'assets')+':'+str(models),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(build/'src/modules/simulation/gz_plugins'),
        GZ_SIM_SERVER_CONFIG_PATH=str(ROOT/'simulation/px4/server_control.config'))
    env.update({'PX4_PARAM_'+k:str(v) for k,v in fusion['parameters'].items()})
    if local_control:
        region_file=ROOT/'simulation/safe_regions/flow_local.json'
        receipt=out/'flight-assets.json'
        write_json(receipt,dict(schema=1,partition=env['GZ_PARTITION'],region_sha256=file_hash(region_file),
            files={str(p.relative_to(out/'assets')):file_hash(p) for p in (out/'assets').rglob('*') if p.is_file()}))
        env.update(UAV_SITL_AUTHORIZATION=str(uuid.uuid4()),UAV_WORKSPACE=str(ROOT),
            UAV_REQUIRE_VIO='0',UAV_LOCALIZATION_PROFILE='FLOW_LOCAL',UAV_ENABLE_FUSION_SWITCH='0',
            UAV_FLIGHT_REGION_PROFILE='flow_local',UAV_FLIGHT_ASSET_RECEIPT=str(receipt),
            UAV_FLIGHT_ASSET_SHA256=file_hash(receipt),UAV_FLIGHT_EVIDENCE=str(out),
            UAV_FLIGHT_MODEL=profile['model']+'_7',UAV_FLIGHT_SCENARIO='full',UAV_FLIGHT_BT='0',
            UAV_ENABLE_EGO_NAV='0',UAV_NAVIGATION_BACKEND='DIRECT',UAV_POCKET_OPERATOR='0',
            UAV_FLIGHT_MISSION_FILE=str(ROOT/'simulation/missions/flow_local_hover.json'),
            PX4_PARAM_COM_RC_IN_MODE='4',PX4_PARAM_COM_OF_LOSS_T='0.5',
            PX4_PARAM_COM_OBL_RC_ACT='4',PX4_PARAM_COM_DISARM_LAND='2')
        if args.native_manual:env['PX4_PARAM_COM_RC_IN_MODE']='1'
        if args.usb_joystick:
            env.update(PX4_PARAM_COM_RC_IN_MODE='1', PX4_PARAM_COM_RC_OVERRIDE='2',
                       PX4_PARAM_MPC_LAND_SPEED='0.3', PX4_PARAM_MPC_LAND_CRWL='0.2')
    inputs = [Path(__file__), PROFILE, fusion_file, ROOT/'scripts/prepare_mtf01_assets.py',
        ROOT/'scripts/mtf01_audit.py', ROOT/'scripts/vio_warehouse_scene.py',
        ROOT/'scripts/mtf01_range_bridge.py',
        ROOT/'scripts/mtf01_flow_bridge.py',
        ROOT/'scripts/sim.sh', ROOT/'scripts/with_px4_sim.sh',
        ROOT/'scripts/vio_render_device.py', ROOT/'scripts/run_px4_sitl_smoke.py',
        ROOT/'simulation/scenes/vio_warehouse.json', ROOT/'simulation/px4/server_control.config',
        px4/'Tools/simulation/gz/worlds/default.sdf', models/'x500/model.sdf', models/'x500_base/model.sdf',
        px4/'src/modules/simulation/gz_bridge/GZBridge.cpp',
        px4/'src/modules/simulation/gz_plugins/optical_flow/OpticalFlowSensor.cpp']
    inputs.extend(p for p in (out/'assets').rglob('*') if p.is_file())
    if local_control:
        inputs.extend(ROOT/name for name in ('scripts/run_px4_flight_tasks.py',
            'scripts/run_flow_manual_tasks.py',
            'src/uav_mission/uav_mission/flow_gate.py','src/uav_mission/uav_mission/px4_flight.py',
            'src/uav_mission/uav_mission/flight_profiles.py','simulation/safe_regions/flow_local.json',
            'simulation/missions/flow_local_hover.json'))
    manifest = dict(schema=1, run_id=run_id, scope='owned disarmed MTF-01 reference audit; no UART or PMW3901 firmware emulation',
        partition=env['GZ_PARTITION'], domain=78, instance=7, duration_s=args.duration,
        profile=profile, fusion=fusion, textured_floor=not args.no_floor_texture,
        stop_sensors=args.stop_sensors, runtime=runtime, versions=lock,
        flight=args.flight,native_manual=args.native_manual,vio_build=build_receipt,
        usb_joystick=args.usb_joystick,model=profile['model']+'_7',namespace='/px4_7',
        localization_profile='FLOW_LOCAL' if local_control else None,
        px4_parameter_overrides={k:v for k,v in env.items() if k.startswith('PX4_PARAM_')},
        operator_environment={k:env[k] for k in ('UAV_REQUIRE_VIO','UAV_LOCALIZATION_PROFILE','UAV_ENABLE_FUSION_SWITCH',
            'UAV_FLIGHT_REGION_PROFILE','UAV_FLIGHT_ASSET_RECEIPT','UAV_FLIGHT_ASSET_SHA256',
            'UAV_FLIGHT_EVIDENCE','UAV_FLIGHT_MODEL','UAV_ENABLE_EGO_NAV','UAV_NAVIGATION_BACKEND') if k in env},
        input_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in inputs}, processes={})
    processes, logs, receipts = [], [], []
    result = dict(passed=False)
    def launch(name, command, launch_env=env):
        log = (out/(name+'.log')).open('w')
        logs.append(log)
        process = subprocess.Popen(list(map(str,command)), cwd=ROOT, env=launch_env,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(process)
        manifest['processes'][name] = dict(pid=process.pid, command=list(map(str,command)), started_system_ns=time.time_ns())
        write_json(out/'manifest.json', manifest)
        return process
    def cli(module, *arguments):
        run = subprocess.run([str(build/'bin'/('px4-'+module)), '--instance','7', *arguments],
            env=env, cwd=ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=4)
        return run.stdout
    def snapshot(prefix=''):
        receipt={}
        for topic in TOPICS:
            raw=cli('listener',topic,'-n','1')
            with (out/(prefix+topic+'.log')).open('a') as log:
                log.write(raw+'\n')
            receipt[topic]=fields(raw)
        return receipt
    try:
        launch('agent', [ROOT/'.deps/microxrce-install/bin/MicroXRCEAgent','udp4','-p','8898'])
        renderer_env = environment(env, args.render_device)
        launch('gazebo', ['gz','sim','-r','-s','--headless-rendering',out/'assets/default.sdf'], renderer_env)
        if args.ui:
            launch('gazebo_gui', ['gz','sim','-g'], renderer_env)
        launch('sensor_bridge', ['ros2','run','ros_gz_bridge','parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/mtf01/flow/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mtf01/range/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'])
        launch('range_adapter', [sys.executable, ROOT/'scripts/mtf01_range_bridge.py','--ros-args','-p','use_sim_time:=true'])
        # Ubuntu OpenCV uses the same NumPy 1.x ABI as ROS, avoiding a new
        # CUDA/venv dependency or changes to the pinned PX4 plugin binaries.
        launch('flow_adapter', ['/usr/bin/python3', ROOT/'scripts/mtf01_flow_bridge.py','--ros-args','-p','use_sim_time:=true'])
        launch('px4', [build/'bin/px4','-d','-i','7','-w',out/'rootfs',build/'etc'])
        if local_control:
            config=out/'qgc-config/QGroundControl';config.mkdir(parents=True)
            if args.qgc_config_from:
                import configparser
                saved=configparser.ConfigParser(interpolation=None,strict=False);saved.optionxform=str
                saved.read(args.qgc_config_from)
                if not saved.has_section('AutoConnect'):saved.add_section('AutoConnect')
                for key,value in dict(autoConnectUDP='true',autoConnectPixhawk='false',autoConnectSiKRadio='false',
                                      autoConnectRTKGPS='false',autoConnectLibrePilot='false').items():saved.set('AutoConnect',key,value)
                with (config/'QGroundControl.ini').open('w') as stream:saved.write(stream,space_around_delimiters=False)
            else:
                (config/'QGroundControl.ini').write_text('[AutoConnect]\nautoConnectUDP=true\nautoConnectPixhawk=false\nautoConnectSiKRadio=false\nautoConnectRTKGPS=false\nautoConnectLibrePilot=false\n')
            launch('qgc',[lock['artifacts']['qgc']['path'],'--allow-multiple','--log-output'],
                dict(env,XDG_CONFIG_HOME=str(out/'qgc-config'),XDG_CACHE_HOME=str(out/'qgc-cache'),
                    QT_QPA_PLATFORM='xcb' if args.ui else 'offscreen'))
        print('MTF-01 audit:', out, flush=True)
        started = time.monotonic()
        next_sample = started+12
        while time.monotonic()-started < args.duration:
            exited=[name for name,p in zip(manifest['processes'],processes) if p.poll() is not None]
            if exited:
                raise RuntimeError('Owned process exited early: '+','.join(exited))
            if time.monotonic() < next_sample:
                time.sleep(.1)
                continue
            receipts.append(snapshot())
            write_json(out/'receipts.json', receipts)
            next_sample = time.monotonic()+2
        parameters = {}
        for name in fusion['parameters']:
            parameters[name] = cli('param','show',name)
        write_json(out/'parameters.json', parameters)
        result = assessment(receipts, profile)
        result['parameter_receipts'] = parameters_match(parameters, fusion['parameters'])
        # Renderer log belongs to this partition and must have been written this run.
        render_log = Path.home()/'.gz/rendering/ogre2.log'
        renderer = capture(render_log, manifest['processes']['gazebo']['started_system_ns'], args.render_device)
        write_json(out/'renderer.json', renderer)
        if render_log.is_file():
            (out/'ogre2.log').write_bytes(render_log.read_bytes())
        result['checks']['renderer_verified'] = renderer['passed']
        result['checks']['parameters_verified'] = result['parameter_receipts']
        if args.stop_sensors:
            for (name,_),process in zip(manifest['processes'].items(),processes):
                if name in ('range_adapter','flow_adapter'):
                    stop(process)
            loss=[]
            loss_start=time.monotonic()
            while time.monotonic()-loss_start<12:
                loss.append(snapshot('loss-'))
                time.sleep(1)
            write_json(out/'source-loss.json',loss)
            from mtf01_audit import loss_assessment
            result['source_loss']=loss_assessment(loss)
            result['checks']['source_loss_detected']=result['source_loss']['passed']
        result['passed'] = all(result['checks'].values())
        if args.usb_joystick and result['passed']:
            write_json(out/'ground-audit.json', result)
            owner=launch('pocket_operator',[sys.executable,ROOT/'scripts/pocket_operator_session.py',
                '--device',args.device,'--input-profile','uav-aetr-v1','--session',str(out),'--enable-sitl-control'])
            live=dict(session=str(out),profile='FLOW_LOCAL',scene='warehouse',scope='operator session; no automatic arm/takeoff',
                      ground_audit_passed=True,operator_pid=owner.pid)
            write_json(out/'live-session.json',live)
            print('FLOW_LOCAL Pocket ready:',out,flush=True)
            until=time.monotonic()+args.session_duration
            while time.monotonic()<until:
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('Owned FLOW_LOCAL operator/sensor process exited')
                time.sleep(.1)
            final=snapshot('final-')
            write_json(out/'final-state.json',final)
            result['operator_session_closed_ground_disarmed']=final['vehicle_status'].get('arming_state')==1
            result['passed'] &= result['operator_session_closed_ground_disarmed']
            result['scope']='FLOW_LOCAL warehouse Pocket session; ground fusion audit, no automatic flight acceptance'
        if args.flight and result['passed']:
            flight=launch('flight_tasks',[sys.executable,ROOT/('scripts/run_flow_manual_tasks.py' if args.native_manual else 'scripts/run_px4_flight_tasks.py')])
            until=time.monotonic()+110.
            while flight.poll() is None and time.monotonic()<until:
                if any(p.poll() is not None for p in processes if p is not flight):
                    raise RuntimeError('Owned sensor/PX4 process exited during flow flight')
                time.sleep(.1)
            flight_file=out/'flight-observation.json'
            flight_result=read(flight_file) if flight_file.is_file() else {}
            result['flight']=flight_result
            result['checks']['standalone_flow_flight']=(flight.poll()==0 and flight_result.get('passed') is True
                and flight_result.get('standalone_flow',{}).get('passed') is True)
            result['passed']=all(result['checks'].values())
            result['scope']='No-GNSS/no-EV rendered MTF reference takeoff/hover/native landing; no hardware flight qualification'
    except BaseException as exc:
        result['error'] = type(exc).__name__+': '+str(exc)
        raise
    finally:
        for process in reversed(processes):
            stop(process)
        for log in logs:
            log.close()
        groups={p.pid for p in processes}
        live=[]
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                stat=(entry/'stat').read_text().rsplit(')',1)[1].split()
                if int(stat[2]) in groups and stat[0]!='Z':
                    live.append(int(entry.name))
            except (OSError,ValueError):
                continue
        result['remaining_owned_processes']=live
        result['cleanup_confirmed']=not live and all(p.poll() is not None for p in processes)
        result['passed'] &= result['cleanup_confirmed']
        result['cleanup'] = {name:dict(pid=receipt['pid'], exited=p.poll() is not None)
            for (name,receipt),p in zip(manifest['processes'].items(),processes)}
        write_json(out/'result.json', result)
        print(json.dumps(dict(directory=str(out), **result), indent=2), flush=True)
    if not result['passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
