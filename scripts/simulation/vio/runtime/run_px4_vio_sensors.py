#!/usr/bin/env python3
"""Owned official PX4/Gazebo/QGC VIO audits and explicit warehouse BT flight."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import argparse
import fcntl
import json
import math
import os
from pathlib import Path
import re
import signal
from shutil import disk_usage
import subprocess
import sys
import time
import uuid
import rclpy
from sim_validation import ROOT,read,file_hash,write_json
from prepare_px4_sim import ensure_source,check_external
from run_px4_sitl_smoke import stop,free_port
from prepare_vio_sensor_assets import assets,PROFILE
from px4_vio_sensor_audit import SensorAudit
from px4_vio_pose_audit import PoseAudit
from px4_vio_motion_audit import MotionAudit
from vio_sdk_parameters import stream_parameters
from vio_sdk_runtime import SdkRuntimeReceipt
from vio_ui_lifecycle import grounded_for_viewers
from vio_render_device import environment as render_environment,capture as capture_renderer
from vio_evidence import nullable
from usb_joystick import qgc_environment


def main():
    from simulation.backends import source_hashes
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--imu-noise-model',choices=('ideal','declared'),default='ideal',help='Declared white IMU noise and matching estimator model; OpenVINS disarmed experiment only')
    parser.add_argument('--experimental-chain',action='store_true',help='Explicit SDK16/MTF simulation flight with covariance qualification deferred; sequence connects real depth/EGO')
    parser.add_argument('--flight',choices=('hover','hover-low','sequence'),help='Owned warehouse BT flight; separate from disarmed audits')
    from no_gnss_flight_cases import CASES,INPUT_CASES
    parser.add_argument('--flight-case',choices=CASES,default='full',help='Explicit no-GNSS return/landing/intervention acceptance; full remains the normal mission')
    parser.add_argument('--sdk-debug-dump',action='store_true',help='Explicit SDK-consumed input dump; diagnostic load, not VIO qualification')
    parser.add_argument('--sdk-information-log',action='store_true',help='Record native conditional/full information for SDK16 disarmed or experimental-chain diagnostics; extra I/O, not production latency evidence')
    parser.add_argument('--sdk-schur-diagnostic',action='store_true',help='Explicit isolated passive Hessian capture; requires SDK16 information logging')
    parser.add_argument('--sdk-schur64',action='store_true',help='Explicit isolated double precision marginalization candidate; no default SDK replacement')
    parser.add_argument('--vio-primary',action='store_true',help='Fixed SDK64 avoidance profile: mandatory EV fusion and fresh MTF inputs, opportunistic auxiliary fusion; joint ground preflight remains mandatory')
    parser.add_argument('--sdk-profile',action='store_true',help='Explicit Nsight CUDA/NVTX trace; disarmed diagnostic only, never source qualification')
    parser.add_argument('--sdk-executor',choices=('standalone','single_thread'),default='standalone',help='Explicit SDK executor comparison; single_thread matches the navigation component container')
    parser.add_argument('--estimator',choices=('cuvslam',),default='cuvslam',help='cuVSLAM only; OpenVINS development paused')
    parser.add_argument('--sdk-candidate',choices=('sdk16','sdk16-stock','sdk16-marginal'),default='sdk16',help='Default SDK16 with corrected conditional covariance; disarmed diagnostics only')
    parser.add_argument('--sdk-cpu-list',help='Explicit taskset CPU list for disarmed SDK scheduling comparison only')
    parser.add_argument('--diagnostic-visual-only',action='store_true',help='Motion-only SDK stereo comparison; no normalized source, no VIO acceptance')
    parser.add_argument('--duration',type=float,default=45.,help='Disarmed observation duration only; flight stops at task terminal with a 280 s cap')
    parser.add_argument('--scene',choices=('planar','layered','warehouse'),default='planar')
    parser.add_argument('--camera-pitch-deg',type=int,choices=(0,5,15,30),default=5,help='Default fixed 5 degree downward rig; other angles are historical reproduction only')
    parser.add_argument('--warehouse-floor-texture',action='store_true',help='Explicit near-field floor texture comparison, warehouse only')
    parser.add_argument('--warehouse-texture-style',choices=('corners','unique'),default='corners',help='Explicit independent floor atlas comparison; unique is not flight-qualified')
    parser.add_argument('--warehouse-variant',choices=('baseline','complex','avoidance'),default='baseline',help='Complex geometry and multiscale texture experiment; disarmed only')
    parser.add_argument('--motion',action='store_true',help='Independent force-driven sensor carrier; PX4 remains disarmed')
    parser.add_argument('--same-aircraft-motion',action='store_true',help='Test-only wrench on actual disarmed x500 after ground fusion READY; never motor flight')
    parser.add_argument('--fixture-attitude-amplitude',type=float,choices=(0.,.12),default=0.,help='Actual roll/pitch excitation on the disarmed SDK16 fixture')
    parser.add_argument('--ui',action='store_true')
    parser.add_argument('--diagnostic-ui',action='store_true',help='Explicit UI fusion/flight observation; never qualifies the headless baseline')
    parser.add_argument('--keep-running',action='store_true',help='Retain the full live VIO/fusion chain after verified landed/disarmed mission; close via run directory close-ui file')
    parser.add_argument('--ui-hold-seconds',type=float,default=0.,help='Keep owned viewers after confirmed landed/disarmed terminal; stop via run directory close-ui file')
    parser.add_argument('--image-resolution',choices=('640x400','480x300'),default='640x400',help='Explicit simulated stereo resolution with regenerated calibration')
    parser.add_argument('--stereo-encoding',choices=('rgb8','mono8'),default='rgb8',help='Native Gazebo single-channel stereo candidate matching navigation cameras; explicit, not flight-qualified')
    parser.add_argument('--quality-policy',choices=('strict','bounded_gap'),default='strict',help='Explicit covariance sample rejection policy; no freshness extension')
    parser.add_argument('--ekf-delay-max-ms',type=int,choices=(160,200),default=None,help='Explicit EKF delayed-horizon setting for pose fusion or owned flight; no EV timestamp offset')
    parser.add_argument('--sdk-image-depth',type=int,choices=range(1,11),default=None,help='cuVSLAM image subscription depth; OpenVINS uses reliable depth 8 and paired queues of 4')
    parser.add_argument('--headless-rendering',action='store_true',help='Use Gazebo native offscreen renderer for disarmed VIO fusion or carrier motion')
    parser.add_argument('--render-device',choices=('default','nvidia','intel'),default='default',help='Select owned Gazebo renderer; explicit devices require fresh hardware renderer confirmation')
    parser.add_argument('--real-time-factor',type=float,default=None,help='Explicit [0.8,1.0] simulation pacing for disarmed pose fusion or carrier motion')
    parser.add_argument('--fuse-pose',action='store_true',help='Same-aircraft actual VIO pose -> PX4 EV; disarmed audit with source stop')
    parser.add_argument('--mtf01',action='store_true',help='Explicit same-aircraft optical flow/range joint audit; disarmed warehouse only')
    parser.add_argument('--joint-fault',choices=('vio','flow','range','all'),default='vio',help='Joint audit first stops this source; VIO is always retired before cleanup')
    parser.add_argument('--normalize',action='store_true',help='Audit reviewed SDK pose normalization; EV requires --fuse-pose or --flight')
    parser.add_argument('--source-warmup-seconds',type=float,default=None,help='Original-sample-time startup exclusion; default 20 s for disarmed PX4 pose fusion, 0 s for source-only audits and historical qualified flight')
    parser.add_argument('--reset-source',action='store_true',help='Retire normalized source then reset actual SDK; disarmed only')
    parser.add_argument('--pocket-session',action='store_true',help='Persistent disarmed startup with sole Pocket flight owner; no automatic mission')
    parser.add_argument('--pocket-task',choices=('avoidance','intervention'),default='avoidance',
                        help='Pocket task recipe; intervention holds near launch with a 0.5 m/s native horizontal velocity cap')
    parser.add_argument('--manual-only',action='store_true',help='Pocket session restricted to manual flight; reject automatic task intents')
    parser.add_argument('--device',default='/dev/input/js0')
    parser.add_argument('--input-profile', choices=('uav-aetr-v1', 'pocket-rc-v2', 'legacy-usb'), default='uav-aetr-v1',
                        help='Virtual AETR or physical SB/SC/SD matrix; legacy-usb retains the old two-state mapping')
    parser.add_argument('--qgc-config-from',type=Path)
    parser.add_argument('--session-duration',type=float,default=3600.)
    args = parser.parse_args()
    if args.pocket_task=='intervention' and not (args.pocket_session and args.experimental_chain
            and args.flight=='sequence' and args.warehouse_variant=='avoidance' and not args.manual_only):
        parser.error('Pocket intervention task requires the complete experimental avoidance Pocket session')
    if args.manual_only and not args.pocket_session:
        parser.error('--manual-only requires --pocket-session')
    if args.input_profile in ('legacy-usb','pocket-rc-v2') and not args.pocket_session:
        parser.error('Physical Pocket input requires --pocket-session')
    if args.flight_case!='full' and (not args.experimental_chain or args.flight!='sequence' or
            args.warehouse_variant!='avoidance' or not args.vio_primary or args.pocket_session):
        parser.error('Flight cases require the explicit no-GNSS VIO-primary avoidance sequence, without a Pocket operator')
    if args.flight_case in INPUT_CASES and not args.qgc_config_from:
        parser.error('Input acceptance needs --qgc-config-from with actual virtual joystick calibration')
    if args.pocket_session and (not args.experimental_chain or args.flight!='sequence' or not args.diagnostic_ui or args.keep_running or args.ui_hold_seconds or not 60<=args.session_duration<=14400):
        parser.error('Pocket requires experimental sequence, diagnostic UI and bounded session, without automatic postflight retention')
    if args.warehouse_variant=='avoidance' and (not args.experimental_chain or args.flight!='sequence'):
        parser.error('Avoidance scene requires experimental depth/EGO sequence')
    region_name='vio_avoidance' if args.warehouse_variant=='avoidance' else 'vio_chain' if args.experimental_chain else 'warehouse'
    if args.experimental_chain and (not args.flight or args.sdk_candidate!="sdk16" or not args.mtf01 or args.scene!="warehouse" or args.camera_pitch_deg!=5 or args.reset_source or args.same_aircraft_motion or args.motion):
        parser.error("Experimental chain requires default SDK16, MTF, warehouse, fixed 5 degrees and a motor-flight task")
    selected_sdk=args.sdk_candidate
    openvins=args.estimator=='openvins'
    if openvins:
        if args.sdk_candidate or args.sdk_debug_dump or args.sdk_profile or args.reset_source:
            parser.error('OpenVINS requires its own source; no SDK profiling, dump or reset')
        args.sdk_candidate='openvins'

    if args.sdk_candidate and (args.reset_source or (args.flight and not args.experimental_chain)):
        parser.error(('OpenVINS' if openvins else 'SDK16')+' candidate requires disarmed diagnostics without reset or flight')
    if args.sdk_information_log and (args.sdk_candidate!='sdk16' or (args.flight and not args.experimental_chain)):
        parser.error('--sdk-information-log requires default SDK16 disarmed or experimental-chain diagnostics')
    if args.sdk_schur_diagnostic and (args.sdk_candidate!='sdk16' or not args.sdk_information_log):
        parser.error('--sdk-schur-diagnostic requires SDK16 and --sdk-information-log')
    if args.sdk_schur64 and (args.sdk_candidate!='sdk16' or args.sdk_schur_diagnostic):
        parser.error('--sdk-schur64 requires SDK16 without the passive diagnostic variant')
    if args.vio_primary and (not args.experimental_chain or args.warehouse_variant!='avoidance' or not args.sdk_schur64):
        parser.error('--vio-primary requires the explicitly owned SDK64 avoidance experiment')
    if args.imu_noise_model=='declared' and not openvins:
        parser.error('Declared IMU noise is an isolated OpenVINS disarmed experiment')
    if args.same_aircraft_motion and (args.sdk_candidate not in ('sdk16','sdk16-stock','sdk16-marginal','openvins') or not args.fuse_pose or
            not args.normalize or args.flight or args.motion or args.reset_source or args.duration<220):
        parser.error('--same-aircraft-motion requires a VIO candidate with disarmed normalized fusion, duration >=220, without flight/carrier/reset')
    if args.fixture_attitude_amplitude and not (args.sdk_candidate and args.same_aircraft_motion):
        parser.error('Fixture attitude excitation requires SDK16 same-aircraft disarmed motion')
    if args.warehouse_variant!='baseline' and (args.scene!='warehouse' or not args.warehouse_floor_texture or args.warehouse_texture_style!='corners' or (args.flight and not args.experimental_chain)):
        parser.error('Complex warehouse requires textured warehouse without flight or other texture styles')
    if args.mtf01 and ((not args.fuse_pose and not args.experimental_chain) or (args.flight and not args.experimental_chain) or args.motion or args.reset_source or
                       args.scene!='warehouse' or not args.warehouse_floor_texture or args.duration<60):
        parser.error('--mtf01 requires disarmed --normalize --fuse-pose --scene warehouse --warehouse-floor-texture and duration >=60')
    if args.joint_fault!='vio' and not args.mtf01:parser.error('--joint-fault requires --mtf01')
    if args.mtf01 and (args.sdk_debug_dump or args.sdk_profile or (args.ui_hold_seconds and not args.experimental_chain)):
        parser.error('Joint audit cannot combine profiler/debug dump or retained UI')
    if args.sdk_cpu_list:
        if (args.flight and not args.experimental_chain) or not (args.fuse_pose or args.motion or args.experimental_chain):parser.error('--sdk-cpu-list requires disarmed --fuse-pose or --motion')
        from vio_cpu_affinity import cpu_set
        try:cpu_set(args.sdk_cpu_list)
        except ValueError as exc:parser.error(str(exc))
    if args.sdk_profile and (args.flight or not (args.fuse_pose or args.motion)):
        parser.error('--sdk-profile requires disarmed --fuse-pose or --motion')
    if args.sdk_executor!='standalone' and ((args.flight and not args.experimental_chain) or not (args.fuse_pose or args.motion or args.experimental_chain)):
        parser.error('--sdk-executor single_thread requires disarmed --fuse-pose or --motion')
    if args.sdk_profile and args.sdk_executor!='standalone':
        parser.error('SDK profiling currently requires the standalone executable')
    if args.flight and (args.scene!='warehouse' or not args.normalize or args.fuse_pose or args.motion or args.reset_source or args.sdk_debug_dump or args.diagnostic_visual_only):
        parser.error('--flight requires warehouse/normalize without disarmed fusion, motion, reset or diagnostics')
    if args.diagnostic_ui and not (args.ui and (args.flight or args.fuse_pose)):parser.error('--diagnostic-ui requires --ui and --flight or --fuse-pose')
    if not 0<=args.ui_hold_seconds<=3600 or (args.ui_hold_seconds and not args.diagnostic_ui):parser.error('UI hold requires diagnostic UI flight and 0..3600 seconds')
    if args.keep_running and (not args.experimental_chain or not args.diagnostic_ui or args.ui_hold_seconds):
        parser.error('--keep-running requires experimental chain diagnostic UI and no timed viewer hold')
    if args.flight and args.camera_pitch_deg==5 and not args.experimental_chain:parser.error('5 degree VIO flight source is not qualified yet; inspect with --fuse-pose --ui --diagnostic-ui without GNSS before flight')
    pose_fusion_mode=bool(args.fuse_pose or args.flight)
    if args.source_warmup_seconds is not None and (not args.normalize or not 0<=args.source_warmup_seconds<=30):
        parser.error('--source-warmup-seconds requires --normalize and a finite value within [0,30]')
    source_warmup_seconds=args.source_warmup_seconds if args.source_warmup_seconds is not None else (20. if args.fuse_pose and not args.sdk_candidate else 0.)
    sdk_image_depth=args.sdk_image_depth if args.sdk_image_depth is not None else (1 if pose_fusion_mode else 10)
    if args.flight and (args.stereo_encoding!='rgb8' or source_warmup_seconds!=0.):
        parser.error('Changed stereo encoding or warmup requires new source qualification; inspect with disarmed --fuse-pose first')
    if args.diagnostic_visual_only and (not args.motion or args.normalize or args.fuse_pose or args.reset_source):
        parser.error('--diagnostic-visual-only requires --motion without normalize/fuse/reset')
    if args.warehouse_texture_style!="corners" and (args.scene!="warehouse" or not args.warehouse_floor_texture):parser.error("Unique texture requires warehouse floor texture")
    if args.warehouse_floor_texture and args.scene!='warehouse':
        parser.error('--warehouse-floor-texture requires --scene warehouse')
    if args.ekf_delay_max_ms is not None and not pose_fusion_mode:
        parser.error('--ekf-delay-max-ms requires --fuse-pose or --flight')
    if args.sdk_image_depth is not None and not (pose_fusion_mode or args.motion):
        parser.error('--sdk-image-depth requires --fuse-pose, --flight or --motion')
    if args.headless_rendering and not (pose_fusion_mode or args.motion):
        parser.error('--headless-rendering requires --fuse-pose, --flight or --motion')
    if args.render_device!='default' and not (pose_fusion_mode or args.motion):
        parser.error('--render-device requires --fuse-pose, --flight or --motion')
    if args.real_time_factor is not None and (not (pose_fusion_mode or args.motion) or not .8<=args.real_time_factor<=1.):
        parser.error('--real-time-factor requires --fuse-pose, --flight or --motion, and a value within [0.8,1.0]')
    if args.fuse_pose and (not args.normalize or args.motion or args.reset_source or args.duration<40):
        parser.error('--fuse-pose requires --normalize, duration >=40 s, no carrier motion or reset')
    if args.scene!='planar' and not (args.motion or pose_fusion_mode):
        parser.error('--scene layered/warehouse requires --motion or disarmed --fuse-pose')
    if args.motion and (not (args.normalize or args.diagnostic_visual_only) or args.reset_source or args.duration < 50):
        parser.error('--motion requires --normalize, duration >=50 s, and no reset')
    if args.reset_source and (not args.normalize or args.duration < 35):
        parser.error('--reset-source requires --normalize and duration >=35 s')
    duration_max=300 if args.sdk_candidate and args.same_aircraft_motion else 240 if args.sdk_candidate else 120
    if not 20 <= args.duration <= duration_max: parser.error('duration outside allowed disarmed observation range')
    if args.experimental_chain:
        # Raw input snapshots, preview pixels and native flight logs exceed
        # the final compressed archive size. Reject before assets/processes.
        available=disk_usage(ROOT/'.cache').free
        if available < 2*1024**3:
            parser.error('Experimental VIO flight evidence requires at least 2 GiB free; '
                         f'available {available/1024**3:.3f} GiB')
    lock = read(ROOT/'simulation/px4/versions.lock.yaml')
    input_identity = None
    if args.pocket_session:
        from pocket_input import preflight_device
        input_identity = preflight_device(args.device, args.input_profile)
    for key in ('sitl','agent','px4_msgs'): ensure_source(lock[key])
    check_external(lock)
    for artifact in lock['build_artifacts']:
        if file_hash(ROOT/artifact['path']) != artifact['sha256']:
            raise RuntimeError('Locked baseline artifact drift: '+artifact['path'])
    for port in (8898,18577,14587,14550): free_port(port)
    if Path('/tmp/px4_lock-7').exists(): raise RuntimeError('PX4 instance 7 is occupied')
    run_id = str(uuid.uuid4())
    out = ROOT/'.cache/simulation/vio-sensors'/run_id
    (out/'rootfs').mkdir(parents=True)
    px4 = ROOT/lock['sitl']['path']
    build = px4/'build/px4_sitl_default'
    vio_build=None
    if pose_fusion_mode:
        from build_px4_vio import BUILD,verify
        vio_build=verify(lock);build=BUILD
    if args.experimental_chain:
        import shutil
        # Preserve the full generated startup tree before adding owned logging.
        shutil.copytree(build/'etc',out/'rootfs/etc',dirs_exist_ok=True)
        logger_target=out/'rootfs/etc/logging/logger_topics.txt'
        logger_target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/'simulation/px4/vio/telemetry/logger_topics.txt',logger_target)
    models = px4/'Tools/simulation/gz/models'
    upstream_world = px4/'Tools/simulation/gz/worlds/default.sdf'
    motion_plugin = ROOT/'.deps/vio-motion-build/libvio_motion_carrier.so' if args.motion else None
    if motion_plugin is not None and not motion_plugin.is_file():
        raise RuntimeError('Build the independent fixture first: ./scripts/build/build_vio_motion.sh')
    if motion_plugin is not None:
        receipt = read(motion_plugin.parent/'build-inputs.json')
        for name,digest in receipt.items():
            if file_hash(ROOT/name) != digest:
                raise RuntimeError('Motion fixture build drift: '+name)
    noise_path=ROOT/'simulation/px4/vio/openvins/imu_noise.json'
    profile,frames = assets(out/'assets',upstream_world,motion_plugin=motion_plugin,scene=args.scene,real_time_factor=args.real_time_factor,resolution=tuple(map(int,args.image_resolution.split('x'))),warehouse_floor_texture=args.warehouse_floor_texture,camera_pitch_deg=args.camera_pitch_deg,warehouse_texture_style=args.warehouse_texture_style,stereo_encoding=args.stereo_encoding,warehouse_variant=args.warehouse_variant,imu_noise_profile=read(noise_path) if args.imu_noise_model=='declared' else None)
    mtf_profile=None
    if args.mtf01:
        from prepare_mtf01_assets import attach_to_vio_assets
        mtf_profile=attach_to_vio_assets(out/'assets',profile)
    chain_depth=None
    if args.experimental_chain and args.flight=="sequence":
        from vio_chain_mapping import attach_depth
        chain_depth=attach_depth(out/"assets",profile,frames)
    fixture_profile=None
    if args.same_aircraft_motion:
        fixture_plugin=ROOT/'.deps/vio-same-aircraft-build/libvio_same_aircraft_fixture.so'
        fixture_receipt=read(fixture_plugin.parent/'build-inputs.json')
        for name,digest in fixture_receipt.items():
            if file_hash(ROOT/name)!=digest:raise RuntimeError('Same-aircraft fixture build drift: '+name)
        from vio_same_aircraft_dynamic import attach_fixture
        fixture_profile=attach_fixture(out/'assets',profile,models,fixture_plugin,attitude_amplitude=args.fixture_attitude_amplitude)
        write_json(out/'fixture-profile.json',fixture_profile)
    env = dict(os.environ,ROS_DOMAIN_ID='78',ROS_LOCALHOST_ONLY='1',GZ_DISTRO='harmonic',
        GZ_PARTITION=('uav_warehouse_flight_' if args.flight else 'uav_vio_sensors_')+run_id,GZ_IP='127.0.0.1',
        PX4_SIM_MODEL='gz_'+('x500' if args.motion else profile['model']),PX4_SYS_AUTOSTART='4001',
        PX4_GZ_STANDALONE='1',PX4_GZ_WORLD='default',HEADLESS='1',
        PX4_UXRCE_DDS_PORT='8898',PX4_PARAM_UXRCE_DDS_SYNCT='0',
        PX4_GZ_MODELS=str(models if args.motion else out/'assets'),PX4_GZ_WORLDS=str(out/'assets'),
        GZ_SIM_RESOURCE_PATH=str(out/'assets')+':'+str(models),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(build/'src/modules/simulation/gz_plugins'),
        GZ_SIM_SERVER_CONFIG_PATH=str(ROOT/'simulation/px4/server_control.config'))
    fusion_profile=None
    fusion_profile_path=ROOT/('simulation/px4/mtf01/joint_fusion.json' if args.mtf01 else 'simulation/px4/vio/profiles/pose_fusion.json')
    if pose_fusion_mode:
        fusion_profile=read(fusion_profile_path)
        if fusion_profile['schema']!=1 or fusion_profile['profile']!=('vio_mtf01_disarmed_v1' if args.mtf01 else 'aligned_pose_v1'):
            raise RuntimeError('Unsupported pose fusion profile')
        env.update({'PX4_PARAM_'+k:str(v) for k,v in fusion_profile['parameters'].items()})
    if args.experimental_chain:
        # x500_base injects zero-mean white acceleration noise, without a
        # constant accelerometer bias. At rest, tilt and learned acceleration
        # bias are not independently observable. Keep gyro bias and gravity
        # fusion, and retain the normal PX4 preflight health checks.
        env['PX4_PARAM_EKF2_IMU_CTRL']='5'
        write_json(out/'simulation-imu-contract.json',dict(
            profile='gazebo_zero_accel_bias_v1',EKF2_IMU_CTRL=5,
            scope='experimental Gazebo only; zero-mean x500_base IMU model',
            accelerometer_bias_learning=False,gyro_bias_learning=True,
            gravity_vector_fusion=True,preflight_checks_disabled=False,
            hardware_qualified=False))
    if args.ekf_delay_max_ms is not None:
        env['PX4_PARAM_EKF2_DELAY_MAX']=str(args.ekf_delay_max_ms)
    os.environ.update({k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','GZ_PARTITION','GZ_IP')})
    params = dict(use_sim_time=True,num_cameras=2,min_num_images=2,tracking_mode=0 if args.diagnostic_visual_only else 1,
        enable_localization_n_mapping=False,rectified_images=True,
        sync_matching_threshold_ms=1.,image_jitter_threshold_ms=60.,imu_jitter_threshold_ms=12.,
        calibration_frequency=float(profile['imu_rate_hz']),
        gyro_noise_density=.000244,gyro_random_walk=.000019393,
        accel_noise_density=.001862,accel_random_walk=.003,
        base_frame='base_link',odom_frame='odom',map_frame='map',imu_frame='vio_imu',
        camera_optical_frames=['vio_left_optical','vio_right_optical'],
        publish_map_to_odom_tf=False,publish_odom_to_base_tf=False)
    if args.sdk_debug_dump:
        (out/'sdk-input').mkdir()
        params.update(enable_debug_mode=True,debug_dump_path=str(out/'sdk-input'))
    params.update(stream_parameters(sdk_image_depth))
    (out/'vio-params.yaml').write_text(json.dumps({'visual_slam':{'ros__parameters':params}},indent=2)+'\n')
    binary = ROOT/'install_uav/isaac_ros_visual_slam/lib/isaac_ros_visual_slam/isaac_ros_visual_slam'
    source_contract='cuvslam15_right_tangent_base_link_v1'
    candidate=None
    vio_env=env
    if args.sdk_candidate:
        from build_vio_sdk16_ros_candidate import PREFIX, FOLDER, CONTRACT, COMMIT
        if args.sdk_candidate=='sdk16':
            from build_vio_sdk16_default import PREFIX, FOLDER, CONTRACT
            if args.sdk_schur_diagnostic:
                from build_vio_sdk16_schur_diagnostic import PREFIX, FOLDER, CONTRACT
            if args.sdk_schur64:
                from build_vio_sdk16_schur64 import PREFIX, FOLDER, CONTRACT
        if args.sdk_candidate=='sdk16-marginal':
            from build_vio_sdk16_marginal_candidate import PREFIX, FOLDER, CONTRACT
        reviewed=FOLDER/'ros-contract.json'
        if openvins:
            from build_openvins_candidate import PREFIX,FOLDER,CONTRACT,BINARY,COMMIT
            reviewed=FOLDER/'contract.json'
        candidate=read(reviewed)
        if (candidate['contract']!=CONTRACT or candidate['source_commit']!=COMMIT
                or candidate['flight_authorized'] or (not openvins and candidate['keyframe_max_seconds']!=1)):
            raise RuntimeError('Unsupported SDK16 candidate contract')
        for name,digest in {**candidate['expected_sha256'],**candidate['protected_baseline_sha256']}.items():
            if file_hash(ROOT/name)!=digest:raise RuntimeError('Candidate contract drift: '+name)
        source_contract=CONTRACT
        binary=BINARY if openvins else PREFIX/'lib/isaac_ros_visual_slam/isaac_ros_visual_slam'
        vio_env=dict(env,AMENT_PREFIX_PATH=str(PREFIX)+':'+env.get('AMENT_PREFIX_PATH',''),
            LD_LIBRARY_PATH=str(PREFIX/'lib')+':'+env.get('LD_LIBRARY_PATH',''))
        if args.sdk_candidate=='sdk16-marginal' or args.sdk_information_log:
            vio_env['CUVSLAM_COVARIANCE_DIAGNOSTIC']=str(out/'sdk16-information.csv')
        if args.sdk_schur_diagnostic:
            vio_env['CUVSLAM_SCHUR_DIAGNOSTIC']=str(out/'sdk16-schur.csv')
        if openvins:
            prefixes=[PREFIX/package for package in ('ov_core','ov_init','ov_msckf')]
            vio_env=dict(env,AMENT_PREFIX_PATH=':'.join(map(str,prefixes))+':'+env.get('AMENT_PREFIX_PATH',''),
                LD_LIBRARY_PATH=':'.join(str(p/'lib') for p in prefixes)+':'+env.get('LD_LIBRARY_PATH',''))
            from prepare_openvins_config import generate
            focal=profile['width']/(2*math.tan(profile['horizontal_fov_rad']/2))
            intrinsic=dict(focal=[focal,focal],principal=[profile['width']/2,profile['height']/2],size=[profile['width'],profile['height']])
            generate(dict(parameters=params,frames=frames,profile=profile),[intrinsic,intrinsic],out/'openvins-config',imu_axes='flu')
    bounded_gc_observation = args.experimental_chain or not args.flight and (args.same_aircraft_motion or (args.sdk_candidate and args.fuse_pose))
    inputs = [Path(__file__),ROOT/'scripts/simulation/vio/runtime/vio_evidence.py',ROOT/'scripts/simulation/vio/sdk/vio_sdk_selection.py',ROOT/'scripts/simulation/vio/runtime/vio_stereo_ui.py',ROOT/'scripts/simulation/vio/runtime/vio_ui_lifecycle.py',ROOT/'src/uav_bringup/rviz/px4_vio_sim.rviz',ROOT/'scripts/simulation/vio/runtime/px4_vio_sensor_audit.py',ROOT/'scripts/simulation/vio/runtime/vio_sensor_transforms.py',ROOT/'scripts/simulation/vio/runtime/prepare_vio_sensor_assets.py',
              ROOT/'scripts/simulation/vio/runtime/vio_sensor_quality.py',ROOT/'scripts/simulation/compat.sh',ROOT/'scripts/simulation/vio/runtime/vio_render_device.py',ROOT/'scripts/simulation/vio/sdk/vio_sdk_parameters.py',ROOT/'scripts/simulation/vio/sdk/vio_sdk_runtime.py',ROOT/'scripts/simulation/vio/runtime/vio_pose_window.py',ROOT/'scripts/simulation/vio/runtime/px4_vio_pose_audit.py',ROOT/'scripts/simulation/vio/runtime/vio_observer_gc.py',
              ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam/include/isaac_ros_visual_slam/impl/message_stream_sequencer.hpp',
              ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam/include/isaac_ros_visual_slam/impl/stopwatch.hpp',
              ROOT/'src/isaac_ros_common/isaac_ros_common/src/qos.cpp',
              ROOT/'src/px4_comm_bridge/px4_comm_bridge/source_timing.py',ROOT/'src/px4_comm_bridge/px4_comm_bridge/pose_dropout.py',
              PROFILE,upstream_world,models/'x500/model.sdf',models/'x500_base/model.sdf',
              out/'assets/default.sdf',out/'assets'/profile['model']/'model.sdf',out/'assets/frames.json',
              out/'vio-params.yaml',ROOT/'simulation/px4/server_control.config',binary,
              ROOT/'install_uav/isaac_ros_visual_slam/lib/libvisual_slam_node.so',
              ROOT/'install_uav/isaac_ros_visual_slam/lib/libcuvslam.so']
    if args.sdk_profile:inputs.extend([ROOT/'scripts/simulation/vio/sdk/vio_sdk_profile.py',ROOT/'scripts/simulation/vio/sdk/vio_sdk_target.py'])
    if args.sdk_executor=='single_thread':inputs.append(ROOT/'scripts/simulation/vio/runtime/vio_component.launch.py')
    if args.sdk_cpu_list:inputs.append(ROOT/'scripts/simulation/vio/runtime/vio_cpu_affinity.py')
    if args.mtf01:
        inputs.extend([ROOT/'scripts/simulation/vio/runtime/vio_mtf01_joint.py',ROOT/'scripts/simulation/flow/prepare_mtf01_assets.py',
            ROOT/'scripts/simulation/flow/mtf01_audit.py',ROOT/'scripts/simulation/flow/mtf01_flow_bridge.py',ROOT/'scripts/simulation/flow/mtf01_range_bridge.py',
            ROOT/'simulation/px4/mtf01/sensors.json',ROOT/'src/uav_mission/uav_mission/vio_gate.py'])
    if bounded_gc_observation:
        inputs.append(ROOT/'scripts/simulation/vio/runtime/vio_gc_pause_audit.py')
    if args.same_aircraft_motion:
        inputs.extend([ROOT/name for name in fixture_receipt])
        inputs.extend([fixture_plugin.parent/'build-inputs.json',ROOT/'scripts/simulation/vio/runtime/vio_same_aircraft_dynamic.py',
                       ROOT/'scripts/simulation/vio/runtime/px4_vio_motion_audit.py',ROOT/'scripts/simulation/vio/runtime/vio_covariance_consistency.py',
                       ROOT/'scripts/simulation/vio/runtime/vio_gc_pause_audit.py',ROOT/'scripts/simulation/vio/analysis/audit_vio_truth_topics.py',out/'fixture-profile.json'])
    if args.scene=='warehouse':
        from vio_warehouse_scene import layout_path
        inputs += [layout_path(args.warehouse_variant),ROOT/'scripts/simulation/vio/runtime/vio_warehouse_scene.py',out/'assets/warehouse-layout.json']
        inputs += sorted((out/'assets/warehouse_textures').rglob('*.png'))
    if args.motion:
        inputs += [motion_plugin,ROOT/'simulation/px4/vio/plugins/motion/MotionCarrier.cc',
                   ROOT/'simulation/px4/vio/plugins/motion/CMakeLists.txt',ROOT/'scripts/build/build_vio_motion.sh',
                   motion_plugin.parent/'build-inputs.json',ROOT/'scripts/simulation/vio/runtime/record_vio_motion_build.py',
                   ROOT/'scripts/simulation/vio/runtime/px4_vio_motion_audit.py']
    if args.normalize and not args.sdk_candidate:
        reviewed = ROOT/'simulation/px4/vio/profiles/pose_contract.json'
        approved = read(reviewed)
        if approved['schema'] != 1 or approved['contract'] != 'cuvslam15_right_tangent_base_link_v1':
            raise RuntimeError('Unsupported SDK pose contract')
        for name,digest in approved['expected_sha256'].items():
            if file_hash(ROOT/name) != digest:
                raise RuntimeError('Reviewed SDK pose contract drift: '+name)
        inputs.append(reviewed)
        inputs += [ROOT/'src/px4_comm_bridge/px4_comm_bridge/cuvslam_pose.py',
            ROOT/'src/px4_comm_bridge/px4_comm_bridge/cuvslam_pose_node.py',ROOT/'scripts/simulation/vio/runtime/px4_vio_pose_audit.py',
            ROOT/'src/px4_comm_bridge/px4_comm_bridge/source_warmup.py',
            ROOT/'src/isaac_ros_nitros/isaac_ros_nitros/lib/cuvslam/include/cuvslam/cuvslam2.h',
            ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/cuvslam_ros_conversion.cpp',
            ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp']
    if args.sdk_candidate:
        inputs += [reviewed]+[ROOT/name for name in candidate['expected_sha256']]
        inputs += [ROOT/'scripts/build/build_vio_sdk16_ros_candidate.py',ROOT/'scripts/simulation/vio/sdk/vio_sdk16_state_audit.py',
            ROOT/'src/px4_comm_bridge/px4_comm_bridge/cuvslam_pose.py',
            ROOT/'src/px4_comm_bridge/px4_comm_bridge/cuvslam_pose_node.py']
    if openvins:
        inputs += list((out/'openvins-config').iterdir())
        if args.imu_noise_model=='declared':inputs.append(noise_path)
        inputs += [ROOT/'src/px4_comm_bridge/px4_comm_bridge/openvins_state.py']
    write_json(out/'calibration.json',dict(schema=1,profile=profile,frames=frames,parameters=params,
        source_contract=source_contract,sdk_cpu_list=args.sdk_cpu_list,sdk_executor=args.sdk_executor,source_warmup_seconds=source_warmup_seconds,quality_policy=args.quality_policy,render_device=args.render_device,headless_rendering=args.headless_rendering,
        native_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in inputs
                       if 'libvisual_slam_node.so' in str(p) or 'libcuvslam.so' in str(p)
                       or 'cuvslam2.h' in str(p) or str(p).endswith('cuvslam_ros_conversion.cpp')
                       or str(p).endswith('visual_slam_impl.cpp')}))
    if args.keep_running:inputs.append(ROOT/'scripts/simulation/vio/runtime/vio_live_evidence.py')
    if args.experimental_chain:inputs.extend((ROOT/'scripts/simulation/vio/runtime/vio_sim_clock.py',ROOT/'simulation/px4/vio/telemetry/logger_topics.txt'))
    calibration = file_hash(out/'calibration.json')
    inputs.append(out/'calibration.json')
    if pose_fusion_mode:
        anchor=dict(schema=1,scope=('fixed launch origin, owned warehouse flight' if args.flight else 'fixed configured local launch origin, disarmed same-aircraft audit only'),
            position_enu=[0.,0.,0.],yaw_enu=0.,height_reference='launch body origin, not terrain altitude',
            calibration_id=calibration,scene_sha256=file_hash(out/'assets/default.sdf'),
            model_sha256=file_hash(out/'assets'/profile['model']/'model.sdf'),
            fusion_profile_sha256=file_hash(fusion_profile_path))
        write_json(out/'anchor.json',anchor)
        inputs += [out/'anchor.json',fusion_profile_path,
            ROOT/'scripts/simulation/vio/runtime/px4_vio_real_fusion.py',ROOT/'src/px4_comm_bridge/px4_comm_bridge/pose_stream.py',
            ROOT/'src/px4_comm_bridge/px4_comm_bridge/pose_fusion.py',ROOT/'src/px4_comm_bridge/px4_comm_bridge/vio_input.py',
            ROOT/'src/uav_mission/uav_mission/vio_gate.py',ROOT/'scripts/build/build_px4_vio.py',
            build/'bin/px4',build/'vio-build.json']
    if args.flight:
        from vio_warehouse_scene import box_clearance
        layout=read(out/'assets/warehouse-layout.json')
        box_clearance(layout['layout'])
        region=ROOT/('simulation/safe_regions/'+region_name+'.json')
        config=read(region);margin=sum(config[k] for k in ('body_radius_m','tracking_margin_m','braking_margin_m'))
        if any(not math.isclose(config.get('known_clearance_bounds_min',config['bounds_min'])[i]+margin,layout['layout']['flight_box_min'][i],abs_tol=1e-9) or
               not math.isclose(config.get('known_clearance_bounds_max',config['bounds_max'])[i]-margin,layout['layout']['flight_box_max'][i],abs_tol=1e-9) for i in (0,1)):
            raise RuntimeError('Warehouse profile/clearance bounds mismatch')
        if args.experimental_chain:
            if not (args.headless_rendering and args.render_device=='nvidia' and sdk_image_depth==1
                    and args.sdk_executor=='single_thread' and args.sdk_cpu_list=='0-15'
                    and args.quality_policy=='bounded_gap' and args.warehouse_floor_texture):
                raise RuntimeError('Experimental chain requires tested renderer, queue, executor, affinity and source quality configuration')
            write_json(out/'source-prerequisites.json',dict(scope='explicit simulation-only progression',
                covariance_qualification_deferred=True,global_covariance_qualified=False,
                position_error_accepted_by_user=True,hardware_authorized=False,
                source_contract=source_contract))
        else:
            from assess_vio_warehouse import assess as qualify
            base=ROOT/'docs/validation/simulation'
            prerequisites=[base/'2026-10-09-warehouse-low-load-motion/motion',
                           base/'2026-10-09-warehouse-queue-latency/nvidia-depth1-headless-horizon160',
                           base/'2026-10-09-warehouse-queue-latency/nvidia-depth1-headless-horizon160-repeat']
            # Exact checked source configuration only; UI/realtime are still unqualified.
            if not (args.headless_rendering and (not args.ui or args.diagnostic_ui) and args.render_device=='nvidia' and
                    args.sdk_image_depth==1 and args.real_time_factor==.8 and args.ekf_delay_max_ms==160 and
                    args.warehouse_texture_style=='corners' and args.camera_pitch_deg==15 and args.image_resolution=='640x400' and args.stereo_encoding=='rgb8' and source_warmup_seconds==0. and
                    args.warehouse_floor_texture and args.quality_policy=='bounded_gap'):
                raise RuntimeError('Warehouse flight requires explicitly qualified low-load configuration')
            prerequisite_receipts=[]
            for folder in prerequisites:
                assessment=qualify(folder)
                if not assessment['passed']:raise RuntimeError('Warehouse source prerequisite failed: '+str(folder))
                prerequisite_receipts.append(assessment)
            write_json(out/'source-prerequisites.json',prerequisite_receipts)
        receipt_files=[p for p in inputs if p.is_relative_to(out) and p.is_file()]
        write_json(out/'flight-assets.json',dict(schema=1,partition=env['GZ_PARTITION'],
            calibration_id=calibration,region_sha256=file_hash(region),
            files={str(p.relative_to(out)):file_hash(p) for p in receipt_files}))
        env.update(UAV_SITL_AUTHORIZATION=str(uuid.uuid4()),UAV_WORKSPACE=str(ROOT),
            UAV_REQUIRE_VIO='1',UAV_VIO_FUSION_PROFILE=('aligned_pose_mtf_primary_sim_v1' if args.vio_primary else 'aligned_pose_mtf_sim_v1') if args.experimental_chain else 'aligned_pose_v1',UAV_VIO_CALIBRATION_ID=calibration,
            UAV_FLIGHT_REGION_PROFILE=region_name,UAV_FLIGHT_ASSET_RECEIPT=str(out/'flight-assets.json'),
            UAV_FLIGHT_ASSET_SHA256=file_hash(out/'flight-assets.json'),UAV_FLIGHT_MODEL=profile['model']+'_7',
            UAV_FLIGHT_MISSION_FILE=str(ROOT/('simulation/missions/'+{'hover':'warehouse_vio_hover.json','hover-low':'warehouse_vio_hover_low.json','sequence':'warehouse_vio_sequence.json'}[args.flight])),
            UAV_FLIGHT_HOLD_RECEIPT='1',UAV_FLIGHT_EVIDENCE=str(out),UAV_FLIGHT_ADMISSION=str(out/'flight-admission.json'),UAV_FLIGHT_SCENARIO=args.flight_case,UAV_FLIGHT_BT='1',
            PX4_PARAM_COM_RC_IN_MODE='4',PX4_PARAM_COM_OF_LOSS_T='0.5',PX4_PARAM_COM_OBL_RC_ACT='4',PX4_PARAM_COM_DISARM_LAND='2')
        if args.experimental_chain:
            env.update(UAV_EXPERIMENTAL_VIO_CHAIN='1',UAV_FLIGHT_COORDINATOR_INSTANCE=run_id)
            if args.flight=='sequence':
                env.update(UAV_FLIGHT_MISSION_FILE=str(ROOT/('simulation/missions/vio_static_avoidance.json' if args.warehouse_variant=='avoidance' else 'simulation/missions/depth_observation.json')),
                    UAV_EXPECT_OBSERVATION='1',UAV_ENABLE_EGO_NAV='1',UAV_NAVIGATION_BACKEND='EGO')
            if args.pocket_task=='intervention':
                env['UAV_FLIGHT_MISSION_FILE']=str(ROOT/'simulation/missions/pocket_intervention.json')
                # PX4 constrains its manual velocity parameter by this native
                # limit. Also cap autonomous references for this dedicated test.
                env.update(PX4_PARAM_MPC_XY_VEL_MAX='0.5',PX4_PARAM_MPC_XY_CRUISE='0.5',
                           PX4_PARAM_MPC_XY_VEL_ALL='-10')
            inputs += [ROOT/'scripts/simulation/vio/runtime/vio_chain_mapping.py',ROOT/'src/uav_nav_sim/uav_nav_sim/planning_sources.py',ROOT/'src/uav_nav_sim/uav_nav_sim/planning_sources_node.py',ROOT/'src/uav_ego_adapter/src/planner.cpp',ROOT/'src/uav_ego_adapter/src/braking_volume.hpp',ROOT/'install_uav/uav_ego_adapter/lib/uav_ego_adapter/ego_nvblox_planner']
        inputs += [region,out/'flight-assets.json',out/'source-prerequisites.json',
            ROOT/'scripts/simulation/vio/runtime/px4_vio_flight_session.py',ROOT/'scripts/simulation/px4/run_px4_flight_tasks.py',
            ROOT/'src/uav_mission/uav_mission/flight_profiles.py',ROOT/'src/uav_mission/uav_mission/px4_flight.py',
            ROOT/'src/uav_mission/uav_mission/vio_gate.py',
            ROOT/'src/uav_mission/uav_mission/mtf_inputs.py',
            ROOT/'src/uav_mission/uav_mission/flight_geometry.py',ROOT/'src/uav_mission/uav_mission/observation_survey.py',
            ROOT/'src/uav_mission/uav_mission/ego_execution.py',
            ROOT/'src/uav_mission/uav_mission/home_reference.py',
            ROOT/'src/uav_mission/uav_mission/pilot_intervention.py',
            ROOT/'src/uav_mission/uav_mission/manual_input_evidence.py',
            ROOT/'scripts/simulation/px4/no_gnss_flight_cases.py',
            ROOT/'scripts/simulation/px4/bt_flight_client.py',
            Path(env['UAV_FLIGHT_MISSION_FILE']),ROOT/'src/uav_bt/src/mission_runner.cpp',ROOT/'.deps/mission-install/uav_bt/lib/uav_bt/mission_runner']
    if args.pocket_session or (args.flight and args.experimental_chain and args.warehouse_variant=='avoidance'):
        # Exercise the same native descent used by the operator session. Fast
        # default LAND is a different VIO motion envelope, not an acceptance of
        # the Pocket's deliberately slow landing configuration.
        env.update(PX4_PARAM_MPC_LAND_SPEED='0.3',PX4_PARAM_MPC_LAND_CRWL='0.2',
            PX4_PARAM_NAV_RCL_ACT='3',PX4_PARAM_COM_RC_OVERRIDE='0')
    if args.pocket_session:
        env.update(UAV_LOCALIZATION_PROFILE='DEFAULT',PX4_PARAM_COM_RC_IN_MODE='1',UAV_ENABLE_FUSION_SWITCH='0')
        inputs += [ROOT/'scripts/simulation/pocket/pocket_operator_session.py',ROOT/'scripts/simulation/pocket/pocket_uav_switches.py',ROOT/'scripts/simulation/vio/sdk/vio_sdk_runtime.py']
    if args.flight_case in INPUT_CASES:
        env['PX4_PARAM_COM_RC_IN_MODE']='1'
    if args.flight_case!='full':
        inputs += [ROOT/'scripts/simulation/px4/no_gnss_flight_cases.py',ROOT/'scripts/simulation/pocket/pocket_validation_input.py',ROOT/'scripts/simulation/pocket/pocket_virtual_joystick.py']
    launcher_hashes = source_hashes()
    from simulation.policy import no_gnss_environment
    localization_policy = no_gnss_environment(env, out/'assets/default.sdf')
    manifest = dict(localization_policy=localization_policy,run_id=run_id,launcher_sha256=launcher_hashes,scope=('owned warehouse BT flight using actual simulated VIO; NOT Pro W calibration' if args.flight else 'disarmed stereo/IMU VIO reference, NOT Pro W calibration or VIO flight'),
        diagnostic_ui=args.diagnostic_ui,ui_hold_seconds=args.ui_hold_seconds,keep_running=args.keep_running,flight=args.flight,partition=env['GZ_PARTITION'],domain=78,duration_s=args.session_duration if args.pocket_session else None if args.flight else args.duration,flight_observation_limit_s=config.get('flight_observation_limit_s',280.) if args.flight else None,ui=args.ui,model=profile['model'] if args.motion else profile['model']+'_7',versions=lock,profile=profile,
        render_device=args.render_device,headless_rendering=args.headless_rendering,quality_policy=args.quality_policy,source_warmup_seconds=source_warmup_seconds,effective_sdk_image_depth=sdk_image_depth,requested_sdk_image_depth=args.sdk_image_depth,requested_ekf_delay_max_ms=args.ekf_delay_max_ms,requested_real_time_factor=args.real_time_factor,normalize=args.normalize,fuse_pose=args.fuse_pose,vio_build=vio_build,
        px4_parameter_overrides={k:v for k,v in env.items() if k.startswith('PX4_PARAM_')},motion=args.motion,sdk_debug_dump=args.sdk_debug_dump,sdk_profile=args.sdk_profile,sdk_cpu_list=args.sdk_cpu_list,sdk_executor=args.sdk_executor,diagnostic_visual_only=args.diagnostic_visual_only,reset_source=args.reset_source,calibration_id=calibration,
        mtf01=args.mtf01,joint_fault=args.joint_fault if args.mtf01 else None,
        input_sha256={**launcher_hashes, **{str(p.relative_to(ROOT)):file_hash(p) for p in inputs}},processes={})
    if args.mtf01:manifest['scope']='same-aircraft actual '+('OpenVINS' if openvins else 'cuVSLAM')+'/MTF-01 joint audit; disarmed, not hardware calibration or flight'
    if args.same_aircraft_motion:
        manifest.update(same_aircraft_motion=True,fixture_profile=fixture_profile,
                        scope='same-aircraft actual x500 dynamic fusion, disarmed wrench fixture, NOT motor flight')
    manifest['thread_budget']=dict(opencv_flow=1,openblas=env.get('OPENBLAS_NUM_THREADS'),omp=env.get('OMP_NUM_THREADS'))
    manifest['sdk_information_log']=args.sdk_information_log
    manifest['sdk_schur_diagnostic']=args.sdk_schur_diagnostic
    manifest['sdk_schur64']=args.sdk_schur64
    manifest['vio_primary']=args.vio_primary
    manifest['pocket_task']=args.pocket_task
    manifest['flight_fusion_policy']=dict(primary='EV position/height/yaw',
        auxiliary_fusion='opportunistic' if args.vio_primary else 'mandatory',
        auxiliary_raw_integrity_required=args.vio_primary,joint_ground_preflight_required=args.experimental_chain)
    manifest['production_latency_evidence']=False if args.sdk_information_log else None
    manifest['estimator']=args.estimator
    manifest['candidate_contract']=source_contract
    manifest['selected_sdk']=selected_sdk
    manifest['experimental_chain']=args.experimental_chain
    if args.experimental_chain:
        manifest['scope']='owned same-aircraft SDK16/MTF Gazebo/PX4 flight; covariance qualification deferred; not hardware acceptance'
    manifest['covariance_qualification_deferred']=args.experimental_chain
    manifest['pocket_session']=args.pocket_session
    manifest['manual_only']=args.manual_only
    manifest['input_profile']=args.input_profile if args.pocket_session else None
    manifest['input_device']=input_identity
    manifest['flight_case']=args.flight_case
    if args.pocket_session:
        manifest.update(usb_joystick=True,automatic_flight=False,joystick_device=args.device,localization_profile='VIO_FLOW',operator_environment={k:v for k,v in env.items() if k.startswith('UAV_') and k not in ('UAV_SITL_AUTHORIZATION','UAV_WORKSPACE')})
    manifest['chain_depth']=chain_depth
    manifest['map_visualization']=dict(enabled=bool(chain_depth and args.ui),
        topic='/nvblox_node/mesh',publish_rate_hz=.5 if chain_depth and args.ui else 0.)
    manifest['covariance_kind']=candidate.get('covariance','legacy SDK15 right tangent') if candidate else 'legacy SDK15 right tangent'
    if args.experimental_chain:
        import shutil
        for name,digest in manifest['input_sha256'].items():
            source=ROOT/name
            if file_hash(source)!=digest:raise RuntimeError('Input drift before owned flight: '+name)
            target=out/'consumed-inputs'/name
            target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(source,target)
    processes,logs = [],[]
    def launch(name,command,environment=env):
        log = (out/(name+'.log')).open('w');logs.append(log)
        process = subprocess.Popen(list(map(str,command)),cwd=ROOT,env=environment,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
        manifest['processes'][name] = dict(pid=process.pid,command=list(map(str,command)),started_system_ns=time.time_ns())
        write_json(out/'manifest.json',manifest)
        return process
    def require_processes(phase):
        for name, process in zip(manifest['processes'], processes):
            code = process.poll()
            if code is not None:
                raise RuntimeError(f'{phase} process exited: {name} (exit={code}); log: {out / (name + ".log")}')
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node = rclpy.create_node('vio_sensor_observer')
    audit = SensorAudit(node,profile,frames,allow_pose_fusion=pose_fusion_mode,estimator=args.estimator)
    runtime_receipt=(SdkRuntimeReceipt(node,dict(source_commit=COMMIT,calibration_id=calibration,use_fej=True,num_cameras=2),'/openvins')
        if openvins else SdkRuntimeReceipt(node,{k:params[k] for k in (
        "image_buffer_size","imu_buffer_size","image_qos","image_qos_depth",
        "tracking_mode","num_cameras","min_num_images","camera_optical_frames")}))
    fusion_audit=None
    joint_native=None
    mtf_processes={}
    flight_process=None
    pose_audit = PoseAudit(node,raw_topic='/openvins/pose' if openvins else '/visual_slam/tracking/vo_pose_covariance') if args.normalize or args.diagnostic_visual_only else None
    candidate_audit=None
    if args.sdk_candidate:
        from vio_sdk16_state_audit import Sdk16StateAudit
        candidate_audit=(Sdk16StateAudit(node,state_topic='/openvins/state',pose_topic='/openvins/pose') if openvins else Sdk16StateAudit(node))
    motion_audit = MotionAudit(node) if args.motion or args.same_aircraft_motion else None
    fixture_trigger=None
    gc_audit=None
    truth_probe=None
    if args.same_aircraft_motion:
        from vio_same_aircraft_dynamic import FixtureTrigger
        fixture_trigger=FixtureTrigger(node,attitude_amplitude=args.fixture_attitude_amplitude)
    if bounded_gc_observation:
        from vio_gc_pause_audit import GcPauseAudit
        gc_audit=GcPauseAudit()
    result = dict(passed=False)
    affinity_receipt=None
    candidate_loaded=None
    from vio_observer_gc import ObservationGcGuard
    observation_gc=ObservationGcGuard(bounded_gc_observation)
    try:
        launch('agent',[ROOT/'.deps/microxrce-install/bin/MicroXRCEAgent','udp4','-p','8898'])
        gazebo_env=render_environment(env,args.render_device)
        launch('gazebo',['gz','sim','-r','-s']+(['--headless-rendering'] if args.headless_rendering else [])+[out/'assets/default.sdf'],gazebo_env)
        if args.ui:
            # A viewer-only alias establishes the SDK odom reference in RViz.
            # It defines no aircraft pose, map alignment, or control transform.
            launch('rviz_reference_frame',['ros2','run','tf2_ros','static_transform_publisher',
                '--frame-id','odom','--child-frame-id','vio_display_origin',
                '--ros-args','-p','use_sim_time:=true'])
            launch('gazebo_gui',['gz','sim','-g'],gazebo_env)
            rviz_config=ROOT/'src/uav_bringup/rviz/px4_vio_sim.rviz'
            if openvins:
                original_rviz=rviz_config.read_text();rviz_config=out/'openvins.rviz'
                rviz_config.write_text(original_rviz.replace('/visual_slam/tracking/odometry','/openvins/odometry').replace('/visual_slam/tracking/vo_path','/openvins/path'))
            launch('rviz',['ros2','run','rviz2','rviz2','-d',rviz_config,'--ros-args','-p','use_sim_time:=true'])
            launch('stereo_monitor',[sys.executable,ROOT/'scripts/simulation/vio/runtime/vio_stereo_ui.py',str(out)])
        if args.experimental_chain:
            launch('simulation_clock',[sys.executable,ROOT/'scripts/simulation/vio/runtime/vio_sim_clock.py'])
            write_json(out/'clock-source.json',dict(source='Gazebo IMU unchanged physics header stamp',
                topic='/vio/imu',publisher='gazebo_imu_simulation_clock',extrapolation=False,
                sensor_retimestamping=False,hardware_scope=False))
        launch('sensor_bridge',['ros2','run','ros_gz_bridge','parameter_bridge']+
            ([] if args.experimental_chain else ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])+[

            '/vio/left/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/vio/right/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/vio/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/vio/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/vio/imu@sensor_msgs/msg/Imu[gz.msgs.IMU']+
            (['/vio/truth@nav_msgs/msg/Odometry[gz.msgs.Odometry'] if args.motion or args.same_aircraft_motion else [])+
            (['/vio/fixture/start@std_msgs/msg/Bool]gz.msgs.Boolean'] if args.same_aircraft_motion else [])+
            (['/mtf01/flow/image@sensor_msgs/msg/Image[gz.msgs.Image','/mtf01/range/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'] if args.mtf01 else []))
        if args.mtf01:
            # Range must start before PX4's stuck detector timeout; stationary
            # first samples arriving after ten seconds can remain classified stuck.
            mtf_processes['range']=launch('mtf_range',[sys.executable,ROOT/'scripts/simulation/flow/mtf01_range_bridge.py','--ros-args','-p','use_sim_time:=true'])
            # Load OpenCV and discover DDS before source binding. For the joint
            # experimental chain, compute/warm images now but publish no flow
            # until 50 distinct fresh EV samples have actually been observed.
            mtf_processes['flow']=launch('mtf_flow',['/usr/bin/python3',ROOT/'scripts/simulation/flow/mtf01_flow_bridge.py','--ros-args','-p','use_sim_time:=true']+
                (['-p','wait_for_ev_samples:=50'] if args.experimental_chain else []))
        normalizer=None
        def launch_vio():
            nonlocal normalizer
            reset_remap = ['-r','visual_slam/reset:=/visual_slam/internal/reset'] if args.normalize else []
            vio_command=[binary,'--ros-args','--params-file',out/'vio-params.yaml',
                '-r','visual_slam/image_0:=/vio/left/image','-r','visual_slam/image_1:=/vio/right/image',
                '-r','visual_slam/camera_info_0:=/vio/left/camera_info','-r','visual_slam/camera_info_1:=/vio/right/camera_info',
                '-r','visual_slam/imu:=/vio/imu']+reset_remap
            if args.sdk_profile:
                from vio_sdk_profile import command
                vio_command=command(vio_command,out/'sdk-trace')
            if openvins:
                vio_command=[binary,out/'openvins-config','--ros-args','-p','use_sim_time:=true','-p','calibration_id:='+calibration]
            if args.sdk_executor=='single_thread' and not openvins:
                vio_command=['ros2','launch',ROOT/'scripts/simulation/vio/runtime/vio_component.launch.py',
                    'params_file:='+str(out/'vio-params.yaml'),'reset_proxy:='+str(bool(args.normalize)).lower()]
            if args.sdk_cpu_list:
                from vio_cpu_affinity import command
                vio_command=command(vio_command,args.sdk_cpu_list)
            launch('vio',vio_command,vio_env)
            if args.normalize:
                normalizer=launch('normalizer',['ros2','run','px4_comm_bridge','cuvslam_pose_node','--ros-args',
                    '-p','use_sim_time:=true','-p','calibration_id:='+calibration,
                    '-p','warmup_sec:='+str(source_warmup_seconds),
                    '-p','source_contract:='+source_contract,'-p','quality_policy:='+args.quality_policy]+
                    (['-p','timing_path:='+str(out/'normalizer-timing.json')] if pose_fusion_mode else []))
        # A stationary VIO-only SDK can retain its unknown initial covariance.
        # Observe the aircraft's physical spawn/settling; never fabricate motion.
        if not args.motion: launch_vio()
        if args.pocket_session:
            from vio_sdk_runtime import wait_for_sdk_subscriptions
            write_json(out/'vio-startup-readiness.json',
                wait_for_sdk_subscriptions(node,runtime_receipt,require_processes))
        launch('px4',[build/'bin/px4','-d','-i','7','-w',out/'rootfs',build/'etc'])
        qgc_config = out/'qgc-config/QGroundControl';qgc_config.mkdir(parents=True)
        (qgc_config/'QGroundControl.ini').write_text('[AutoConnect]\nautoConnectUDP=true\nautoConnectPixhawk=false\nautoConnectSiKRadio=false\nautoConnectRTKGPS=false\nautoConnectLibrePilot=false\n')
        if (args.pocket_session or args.flight_case in INPUT_CASES) and args.qgc_config_from:
            import configparser
            saved=configparser.ConfigParser(interpolation=None);saved.optionxform=str
            with args.qgc_config_from.open() as stream:saved.read_file(stream)
            if not saved.has_section('AutoConnect'):saved.add_section('AutoConnect')
            for key in ('autoConnectPixhawk','autoConnectSiKRadio','autoConnectRTKGPS','autoConnectLibrePilot'):saved['AutoConnect'][key]='false'
            saved['AutoConnect']['autoConnectUDP']='true'
            with (qgc_config/'QGroundControl.ini').open('w') as stream:saved.write(stream,space_around_delimiters=False)
        qgc_env = dict(env,XDG_CONFIG_HOME=str(out/'qgc-config'),XDG_CACHE_HOME=str(out/'qgc-cache'),
                       QT_QPA_PLATFORM='xcb' if args.ui else 'offscreen')
        if args.flight_case in INPUT_CASES:
            launch('pocket_test_input',[sys.executable,ROOT/'scripts/simulation/pocket/pocket_validation_input.py',out])
        qgc_env = qgc_environment(qgc_env)
        launch('qgc',[lock['artifacts']['qgc']['path'],'--allow-multiple','--log-output',
            '--logging','Vehicle.MultiVehicleManager,Vehicle.VehicleLinkManager'],qgc_env)
        if args.motion or pose_fusion_mode:
            # Finish model spawn and simulator/QGC
            # startup before binding a VIO source; freshness limits stay unchanged.
            deadline = time.monotonic()+25.
            while True:
                rclpy.spin_once(node,timeout_sec=.05)
                if (audit.counts['left']>=50 and audit.counts['right']>=50
                        and 'vehicle' in audit.last and 'land' in audit.last
                        and 'Adding new vehicle' in (out/'qgc.log').read_text(errors='replace')):
                    break
                if time.monotonic()>deadline: raise RuntimeError('Sensor/vehicle/QGC startup not ready')
            if pose_fusion_mode:
                from px4_vio_real_fusion import RealPoseFusionAudit
                if args.flight:
                    from px4_vio_flight_session import FlightPoseSession
                    if args.experimental_chain:
                        from px4_vio_flight_session import JointFlightPoseSession
                        FlightPoseSession=JointFlightPoseSession
                    fusion_audit=FlightPoseSession(node,calibration,anchor,file_hash(out/'anchor.json'),quality_policy=args.quality_policy)
                elif args.mtf01:
                    from vio_mtf01_joint import JointPoseFusionAudit,NativeAidRecorder
                    joint_native=NativeAidRecorder(build,out,env,
                        extra_topics=('vehicle_visual_odometry',) if args.same_aircraft_motion else ())
                    if args.same_aircraft_motion:
                        from vio_same_aircraft_dynamic import DynamicJointFusionAudit
                        JointPoseFusionAudit=DynamicJointFusionAudit
                    fusion_audit=JointPoseFusionAudit(node,calibration,anchor,file_hash(out/'anchor.json'),quality_policy=args.quality_policy,native=joint_native,profile=mtf_profile)
                else:
                    if args.same_aircraft_motion:
                        from vio_same_aircraft_dynamic import DynamicPoseFusionAudit
                        RealPoseFusionAudit=DynamicPoseFusionAudit
                    fusion_audit=RealPoseFusionAudit(node,calibration,anchor,file_hash(out/'anchor.json'),quality_policy=args.quality_policy)
            if args.motion:launch_vio()
            if args.same_aircraft_motion:
                from audit_vio_truth_topics import TruthTopicAudit
                truth_probe=TruthTopicAudit(out)
        if args.flight:
            if chain_depth:
                # Warm DDS/CUDA participants while VIO alignment is unbound.
                # Save an empty native map before warming depth integration;
                # discard the startup map before binding the aligned mission.
                from vio_chain_mapping import launch_mapping, native_map_file
                launch_mapping(launch,env,out,run_id,graph_only=True,visualize=args.ui)
                launch_mapping(launch,env,out,run_id,binding_only=True)
                write_json(out/'mapping-startup.json',dict(planning_source_process_prewarmed=True,
                    activation_requires='map-admission.json and first valid promoted map',
                    scope='DDS startup before source qualification; no early map binding'))
            command=([sys.executable,ROOT/'scripts/simulation/pocket/pocket_operator_session.py','--device',args.device,'--input-profile',args.input_profile,'--session',out,'--enable-sitl-control', *(['--manual-only'] if args.manual_only else [])] if args.pocket_session else [sys.executable,ROOT/'scripts/simulation/px4/run_px4_flight_tasks.py'])
            flight_process=launch('flight_gateway',command)
            if chain_depth:
                empty_map=out/'prewarm-empty.nvblx'
                native_map_file(node,'save',empty_map)
                launch_mapping(launch,env,out,run_id,depth_only=True)
            preflight_blocked=''
            joint_startup_recorded=False
            ready_until=time.monotonic()+(300. if chain_depth else 190. if args.experimental_chain else 40.)
            while time.monotonic()<ready_until:
                rclpy.spin_once(node,timeout_sec=.02)
                if args.experimental_chain and not joint_startup_recorded and len(fusion_audit.outputs)>=50:
                    joint_startup_recorded=True
                    write_json(out/'joint-startup.json',dict(vio_outputs_before_mtf=len(fusion_audit.outputs),ros=fusion_audit.ros(),flow_process_prewarmed=True,scope='Flow output waits for 50 fresh EV samples; process loads before source binding'))
                require_processes('Owned preflight')
                preflight_fault=fusion_audit.stream.fault or fusion_audit.gate.fault
                if preflight_fault:
                    if not args.pocket_session:raise RuntimeError('VIO preflight fault: '+preflight_fault)
                    preflight_blocked='VIO preflight fault: '+preflight_fault
                    break
                if (fusion_audit.gate.reason=='READY' and runtime_receipt.result()['passed']
                        and (not args.experimental_chain or (candidate_audit.states and candidate_audit.covariance_stamps
                            and candidate_audit.writer_valid
                            and (candidate_audit.states[-1][0]-candidate_audit.covariance_stamps[0])/1e9>=120.))):break
            else:
                if not args.pocket_session:raise RuntimeError('VIO preflight fusion unavailable: '+fusion_audit.gate.reason)
                preflight_blocked='VIO preflight fusion unavailable: '+fusion_audit.gate.reason
            if args.experimental_chain:
                from vio_sdk16_state_audit import loaded_candidate_receipt
                candidate_loaded=loaded_candidate_receipt(manifest['processes']['vio']['pid'],PREFIX)
                write_json(out/'sdk16-loaded-libraries.json',candidate_loaded)
                if not candidate_loaded['passed']:raise RuntimeError('Experimental flight SDK library identity mismatch')
            if not preflight_blocked:
                if chain_depth:
                    # Start a fresh map only after the EKF/VIO launch alignment has
                    # settled; ground initialization resets must not smear TSDF.
                    from vio_chain_mapping import launch_mapping
                    native_map_file(node,'load',empty_map)
                    write_json(out/'map-admission.json',dict(schema=1,native_map_cleared=True,
                        source_ready=True,empty_map_sha256=file_hash(empty_map),
                        scope='GPU startup map discarded; fresh epoch before planning binding'))
                    settle_until=time.monotonic()+10.
                    while time.monotonic()<settle_until:
                        rclpy.spin_once(node,timeout_sec=.02)
                        if fusion_audit.stream.fault:raise RuntimeError('VIO map startup fault: '+fusion_audit.stream.fault)
                    if fusion_audit.gate.reason!='READY':
                        raise RuntimeError('VIO map startup not ready: '+fusion_audit.gate.reason)
                preflight=audit.result()
                write_json(out/'sensor-preflight.json',preflight)
                sensor_failures=[k for k,v in preflight['checks'].items() if not v and k!='only_pose_fmu_input']
                if sensor_failures:raise RuntimeError('Sensor preflight failed: '+str(sensor_failures))
                write_json(out/'flight-admission.pending.json',dict(calibration_id=calibration,sensor_checks_passed=True,
                    fusion_profile=env['UAV_VIO_FUSION_PROFILE'],joint_ground_preflight=args.experimental_chain))
                os.replace(out/'flight-admission.pending.json',out/'flight-admission.json')
        if args.pocket_session:
            write_json(out/'live-session.json',dict(state='POCKET_PREFLIGHT_BLOCKED' if preflight_blocked else 'POCKET_ADMITTED',reason=preflight_blocked,automatic_flight=False,qualification=False,arm_and_task_admitted=not bool(preflight_blocked) and not args.manual_only,
                arm_admitted=not bool(preflight_blocked),task_admitted=not bool(preflight_blocked) and not args.manual_only,
                manual_only=args.manual_only,input_profile=args.input_profile,input_device=input_identity))
            print(('POCKET_DIAGNOSTIC_BLOCKED '+preflight_blocked+' ' if preflight_blocked else 'POCKET_SESSION ')+str(out),flush=True)
        until = time.monotonic()+(args.session_duration if args.pocket_session else config.get('flight_observation_limit_s',280.) if args.flight else args.duration)
        heartbeat=0.
        before_reset = None
        while time.monotonic()<until or (args.pocket_session and not grounded_for_viewers(audit.last,audit.receive,time.monotonic())):
            if args.sdk_candidate and candidate_loaded is None and runtime_receipt.result()['passed']:
                from vio_sdk16_state_audit import loaded_candidate_receipt
                if openvins:
                    from build_openvins_candidate import loaded_receipt
                    candidate_loaded=loaded_receipt(manifest['processes']['vio']['pid'])
                else:candidate_loaded=loaded_candidate_receipt(manifest['processes']['vio']['pid'],PREFIX)
                write_json(out/'sdk16-loaded-libraries.json',candidate_loaded)
                if not candidate_loaded['passed']:raise RuntimeError('Loaded SDK16 candidate libraries differ from contract')
            if args.sdk_cpu_list and affinity_receipt is None and runtime_receipt.result()['passed']:
                from vio_cpu_affinity import receipt
                affinity_receipt=receipt(manifest['processes']['vio']['pid'],args.sdk_cpu_list)
            rclpy.spin_once(node,timeout_sec=.02 if args.flight else .05)
            if fixture_trigger is not None:
                if truth_probe.error:raise RuntimeError('Truth isolation probe failed: '+truth_probe.error)
                fixture_trigger.tick(fusion_audit,motion_audit,truth_isolated=truth_probe.passed)
            if not args.flight and any(p.poll() is not None for p in processes):
                raise RuntimeError('Owned simulation process exited; inspect per-process logs')
            if args.pocket_session:
                require_processes('Owned Pocket')
                if (out/'close-ui').exists() and grounded_for_viewers(audit.last,audit.receive,time.monotonic()):break
                now=time.monotonic()
                if now-heartbeat>=5.:
                    write_json(out/'live-heartbeat.json',dict(system_ns=time.time_ns(),gate=fusion_audit.gate.reason,source_fault=fusion_audit.stream.fault,landed=getattr(audit.last.get('land'),'landed',None),arming=getattr(audit.last.get('vehicle'),'arming_state',None)))
                    for records,limit in ((fusion_audit.history,50000),(fusion_audit.outputs,12000),(fusion_audit.echoes,12000)):
                        if len(records)>limit:del records[:-limit]
                    if len(pose_audit.raw)>12000:
                        for key in list(pose_audit.raw)[:-12000]:del pose_audit.raw[key]
                    heartbeat=now
            elif args.flight and ((out/'flight-observation.json').exists() or flight_process.poll() is not None):break
            if args.reset_source and pose_audit.reset_time is None and time.monotonic() > until-8.:
                before_reset = audit.result()
                before_reset['normalized_pose'] = pose_audit.result(calibration)
                result = before_reset
                if not before_reset['passed'] or not before_reset['normalized_pose']['passed']:
                    raise RuntimeError('Sensor/normalized source not ready before reset')
                pose_audit.request_reset()
        if args.pocket_session:
            result=dict(passed=not fusion_audit.stream.fault,scope='Pocket live diagnostic; mission outcomes separate; no automatic flight')
            if args.manual_only:
                result.update(manual_session_closed=True, manual_preflight_admitted=not bool(preflight_blocked),
                    manual_final_grounded=grounded_for_viewers(audit.last,audit.receive,time.monotonic()),
                    manual_final_source_ready=fusion_audit.gate.reason=='READY' and not fusion_audit.stream.fault and not fusion_audit.gate.fault)
        elif args.flight:
            if not (out/'flight-observation.json').exists():raise RuntimeError('Warehouse flight deadline exceeded or gateway exit')
            result=dict(passed=flight_process.poll() is None,
                scope='warehouse BT actual VIO takeoff/hover/native landing',flight=read(out/'flight-observation.json'))
            result['passed'] &= result['flight']['passed']
        else:result = before_reset or audit.result()
        if args.mtf01 and not args.flight:
            result['scope']=manifest['scope']
            if 'only_pose_fmu_input' in result['checks']:
                result['checks']['reviewed_sensor_fmu_inputs']=result['checks'].pop('only_pose_fmu_input')
        if args.motion:
            result['scope'] = 'independent force-driven sensor motion, disarmed PX4, no EV or flight acceptance'
            for name in ('quality:static_imu','static_drift'):
                result['checks'].pop(name,None)
            result['static_drift_m'] = None
            result['quality'].pop('static_imu',None)
            result['motion'] = motion_audit.result(list(pose_audit.poses))
            first = pose_audit.poses[0]['stamp'] if pose_audit.poses else min(pose_audit.raw,default=float('inf'))+2.
            raw_poses = [dict(stamp=t,position=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z],
                quaternion=[m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w])
                for t,m in pose_audit.raw.items() if t>=first]
            result['motion_raw_sdk_diagnostic'] = motion_audit.result(raw_poses)
            if args.diagnostic_visual_only:
                result['motion']=result['motion_raw_sdk_diagnostic']
                result['scope']='diagnostic visual-only SDK motion; no normalized VIO source or acceptance'
            result['passed'] = all(result['checks'].values()) and result['motion']['passed']
        if args.normalize and not args.flight:
            result['normalized_pose'] = pose_audit.result(calibration)
            result['passed'] &= result['normalized_pose']['passed']
        if args.diagnostic_visual_only:
            result['diagnostic_passed']=result['passed']
            result['passed']=False  # A diagnostic never qualifies warehouse VIO.
        result['qgc_connected'] = bool(re.search(r'Adding new vehicle.*\"UDP Link \(AutoConnect\)\" 8 1 12 2',
                                                     (out/'qgc.log').read_text(errors='replace')))
        result['passed'] &= result['qgc_connected'] and all(p.poll() is None for p in processes if p is not flight_process)
        if args.keep_running:
            terminal=result['flight']
            # The independent gateway may publish its terminal receipt before
            # this observer has drained the final PX4 ground callbacks.
            ground_deadline=time.monotonic()+5.
            while not grounded_for_viewers(audit.last,audit.receive,time.monotonic()) and time.monotonic()<ground_deadline:
                rclpy.spin_once(node,timeout_sec=.02)
            write_json(out/'live-admission.json',dict(mission_passed=result['passed'],
                terminal_landed=terminal.get('final_land'),terminal_arming=terminal.get('final_arming'),
                ground_fresh=grounded_for_viewers(audit.last,audit.receive,time.monotonic()),
                process_exit_codes={k:p.poll() for k,p in zip(manifest['processes'],processes)}))
            if not result['passed'] or not terminal.get('final_land') or terminal.get('final_arming')!=1 or not grounded_for_viewers(audit.last,audit.receive,time.monotonic()):
                raise RuntimeError('Live retention requires passed mission and current landed/disarmed telemetry')
            live_path=out/'live-session.json'
            write_json(live_path,dict(state='LIVE_LANDED_DISARMED',diagnostic=True,qualification=False,
                mission=result,close_file=str(out/'close-ui'),processes=manifest['processes']))
            from vio_live_evidence import start_mission_snapshot
            from px4_vio_real_fusion import json_message
            start_mission_snapshot(out,result,fusion_audit.outputs,json_message)
            print('LIVE_SESSION '+str(out),flush=True)
            heartbeat=0.
            while not (out/'close-ui').exists():
                rclpy.spin_once(node,timeout_sec=.02)
                now=time.monotonic()
                if not grounded_for_viewers(audit.last,audit.receive,now):
                    raise RuntimeError('Live retention lost fresh landed/disarmed telemetry')
                if any(p.poll() is not None for p in processes):
                    raise RuntimeError('Owned live simulation process exited')
                if now-heartbeat>=5.:
                    write_json(out/'live-heartbeat.json',dict(state='LIVE_LANDED_DISARMED',
                        system_ns=time.time_ns(),source_fault=fusion_audit.stream.fault))
                    # This is a live diagnostic window, not an ever-growing
                    # flight archive. Native ULog/mission receipts retain flight.
                    if len(pose_audit.raw)>12000:
                        for stamp_key in list(pose_audit.raw)[:-12000]:del pose_audit.raw[stamp_key]
                    for records,limit in ((fusion_audit.history,50000),(fusion_audit.outputs,12000),(fusion_audit.echoes,12000)):
                        if len(records)>limit:del records[:-limit]
                    heartbeat=now
            write_json(live_path,dict(state='CLOSING_BY_REQUEST',close_file=str(out/'close-ui')))
        if fusion_audit is not None and not args.flight:
            if args.mtf01:
                fusion_audit.freeze_healthy()
                joint_native.phase='healthy_parameters'
                from mtf01_audit import parameters_match
                from concurrent.futures import ThreadPoolExecutor
                from vio_mtf01_joint import parameter_receipts
                with ThreadPoolExecutor(max_workers=1) as executor:
                    future=executor.submit(parameter_receipts,build,env,fusion_profile['parameters'])
                    while not future.done():rclpy.spin_once(node,timeout_sec=.02)
                    observed_parameters=future.result()
                write_json(out/'joint-parameters.json',observed_parameters)
                result['joint_parameters_verified']=parameters_match(observed_parameters,fusion_profile['parameters'])
                result['passed'] &= result['joint_parameters_verified']
                if args.joint_fault!='vio':
                    fusion_audit.mark_mtf_stop(args.joint_fault)
                    for key in (('flow','range') if args.joint_fault=='all' else (args.joint_fault,)):
                        # Waiting for process teardown stalls the VIO callbacks
                        # and can itself trip the receive-gap watchdog. Reap in
                        # owned cleanup after continuing to spin the observer.
                        os.killpg(mtf_processes[key].pid,signal.SIGINT)
                    fault_until=time.monotonic()+12.
                    while time.monotonic()<fault_until:rclpy.spin_once(node,timeout_sec=.02)
                joint_native.phase='vio_loss'
            fusion_audit.mark_stop()
            # Actually terminate the normalized source, keep the SDK running.
            os.killpg(normalizer.pid,signal.SIGINT)
            drain_until=time.monotonic()+(12. if args.mtf01 else 6.)
            while time.monotonic()<drain_until:rclpy.spin_once(node,timeout_sec=.02)
        if fusion_audit is not None and not args.flight:fusion_audit.freeze_terminal_state()
        for side in ('left','right'):
            image = audit.last.get(side)
            if image is not None and image.encoding == 'rgb8' and len(image.data) == image.step*image.height:
                pixels = bytes(image.data)
                rows = b''.join(pixels[i*image.step:i*image.step+3*image.width] for i in range(image.height))
                (out/(side+'.ppm')).write_bytes(f'P6\n{image.width} {image.height}\n255\n'.encode()+rows)
    except Exception as exc:
        result.update(passed=False,error=str(exc))
    finally:
        # Stop the operator while its localization/sensors still run, allowing
        # its bounded cancellation/landing cleanup to finish with fresh VIO.
        if args.pocket_session and flight_process is not None and flight_process.poll() is None:
            flight_process.send_signal(signal.SIGTERM)
            operator_deadline=time.monotonic()+75.
            while flight_process.poll() is None and time.monotonic()<operator_deadline:
                rclpy.spin_once(node,timeout_sec=.02)
        # No further live observer spins occur in cleanup. Restore the caller's
        # GC state before expensive statistics and owned-process teardown.
        observation_gc.restore()
        write_json(out/'observer-gc-policy.json',observation_gc.receipt())
        if gc_audit is not None:write_json(out/'observer-gc-pauses.json',gc_audit.stop())
        if truth_probe is not None:
            truth_probe.stop()
            result['truth_topic_isolation_passed']=truth_probe.passed
            result['passed'] &= truth_probe.passed
        if joint_native is not None:
            try:joint_native.stop()
            except Exception as exc:
                joint_native.errors.append(str(exc))
                result.update(passed=False,joint_cleanup_error=str(exc))
        if fusion_audit is not None or args.motion:
            renderer_log=Path.home()/'.gz/rendering/ogre2.log'
            renderer=capture_renderer(renderer_log,manifest['processes']['gazebo']['started_system_ns'],args.render_device)
            write_json(out/'renderer-info.json',renderer)
            if renderer_log.is_file():(out/'renderer.log').write_bytes(renderer_log.read_bytes())
            result['renderer']=renderer
            result['passed'] &= renderer['passed']
        if fusion_audit is not None:
            if fixture_trigger is not None:
                # Heavy truth/trajectory statistics belong after the observation
                # and source-stop drain, away from the 200 ms live watchdog.
                result['same_aircraft_dynamic']=fixture_trigger.result(motion_audit,fusion_audit)
                result['passed'] &= result['same_aircraft_dynamic']['passed']
            if args.flight:
                result['flight_pose_session']=fusion_audit.flight_result()
                result['passed'] &= not fusion_audit.stream.fault and not fusion_audit.graph_violations and fusion_audit.gateway_seen
            else:
                result['real_pose_fusion']=fusion_audit.result()
                result['passed'] &= result['real_pose_fusion']['passed']
                if args.mtf01:
                    result['joint_observation']=fusion_audit.joint_result()
                    result['passed'] &= result['joint_observation']['passed']
        if args.flight:(out/'release-flight-gateway').touch()
        node.destroy_node();rclpy.try_shutdown()
        if args.diagnostic_ui:
            result['diagnostic_checks_passed']=result['passed']
            result['passed']=False
            result['scope']='UI diagnostic only; no headless flight qualification'
        if args.ui_hold_seconds:
            terminal=read(out/'flight-observation.json') if (out/'flight-observation.json').exists() else dict(scope='disarmed fusion diagnostic; no mission',final_arming=getattr(audit.last.get('vehicle'),'arming_state',None),final_land=getattr(audit.last.get('land'),'landed',False))
            if terminal.get('final_land') and terminal.get('final_arming')==1 and grounded_for_viewers(audit.last,audit.receive,time.monotonic()):
                if flight_process is not None:stop(flight_process)
                if normalizer is not None:stop(normalizer)
                fusion_audit.timing.write(out/'fusion-timing.json')
                for name,evidence in fusion_audit.evidence().items():write_json(out/name,evidence)
                write_json(out/'normalized-poses.json',nullable(list(pose_audit.poses)))
                write_json(out/'normalized-status.json',list(pose_audit.statuses))
                write_json(out/'ui-session.json',dict(result_snapshot=result,state='LANDED_DISARMED_VIEWERS_OPEN',diagnostic=True,qualification=False,close_file=str(out/'close-ui'),terminal=terminal))
                # No observer spins or EV publishing after the mission terminal.
                until_ui=time.monotonic()+args.ui_hold_seconds
                try:
                    while time.monotonic()<until_ui and not (out/'close-ui').exists():time.sleep(.2)
                except KeyboardInterrupt:pass  # Continue owned cleanup.
        profile_cleanup=None
        for process in reversed(processes):
            if args.sdk_profile and process.pid==manifest['processes'].get('vio',{}).get('pid'):
                from vio_sdk_profile import stop_profile
                profile_cleanup=stop_profile(process,out/'sdk-trace')
            else:stop(process)
        for log in logs: log.close()
        try:
            if audit.timing is not None:audit.timing.write(out/'sensor-timing.json')
            if fusion_audit is not None:
                fusion_audit.timing.write(out/'fusion-timing.json')
                for name,evidence in fusion_audit.evidence().items():write_json(out/name,evidence)
            from vio_evidence import nullable,save_raw_sdk
            if pose_audit is not None:save_raw_sdk(out/'sdk-poses-raw.npz',pose_audit.raw)
            write_json(out/'numeric-evidence-contract.json',dict(nonfinite_json='null',raw_sdk='sdk-poses-raw.npz',scope='Invalid source values retained for diagnosis; never repaired or admitted to flight'))
            write_json(out/'odometry.json',nullable(list(audit.records)))
            write_json(out/'samples.json',{k:list(v) for k,v in audit.samples.items()})
            if pose_audit is not None:
                write_json(out/'normalized-poses.json',nullable(list(pose_audit.poses)))
                write_json(out/'normalized-status.json',list(pose_audit.statuses))
                if motion_audit is not None:
                    write_json(out/'truth.json',motion_audit.truth)
                    write_json(out/'motion-imu.json',motion_audit.imu)
                if fixture_trigger is not None:write_json(out/'fixture-trigger.json',fixture_trigger.started)
                write_json(out/'sdk-poses.json',nullable([dict(stamp=t,covariance=list(m.pose.covariance),
                    position=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z],
                    quaternion=[m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w])
                    for t,m in pose_audit.raw.items()]))
        except Exception as exc:
            result.update(passed=False,evidence_error=str(exc))
        result['root_exit_codes'] = {name:p.returncode for name,p in zip(manifest['processes'],processes)}
        groups = {p.pid for p in processes}
        live = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit(): continue
            try:
                stat = (entry/'stat').read_text().rsplit(')',1)[1].split()
                if int(stat[2]) in groups and stat[0] != 'Z': live.append(int(entry.name))
            except (OSError,ValueError): continue
        if args.normalize:
            result['passed'] &= result['root_exit_codes'].get('normalizer') == 0
        result['remaining_owned_processes'] = live
        result['cleanup_confirmed'] = not live and all(p.poll() is not None for p in processes)
        if args.sdk_profile:
            result['sdk_profile_cleanup']=profile_cleanup
            result['cleanup_confirmed'] &= bool(profile_cleanup and profile_cleanup['cleanup_confirmed'])
        result['passed'] &= result['cleanup_confirmed']
        result['sdk_runtime_receipt']=runtime_receipt.result()
        write_json(out/'sdk-runtime-receipt.json',result['sdk_runtime_receipt'])
        result['passed'] &= result['sdk_runtime_receipt']['passed']
        if args.sdk_cpu_list:
            result['sdk_cpu_affinity_receipt']=affinity_receipt
            result['passed'] &= bool(affinity_receipt and affinity_receipt['passed'])
        if args.sdk_debug_dump:
            result['sdk_dump_checks_passed']=result['passed']
            result['passed']=False  # Recording load is diagnostic, never qualification.
        if args.sdk_candidate:
            result['sdk16_inertial_state']=candidate_audit.result()
            write_json(out/'sdk16-inertial-state.json',candidate_audit.states)
            diagnostic_checks=result.get('diagnostic_checks_passed',result['passed'])
            result['sdk16_loaded_libraries']=candidate_loaded
            result['sdk16_candidate_checks_passed']=bool(diagnostic_checks and result['cleanup_confirmed']
                and result['sdk_runtime_receipt']['passed'] and result['sdk16_inertial_state']['passed']
                and result['root_exit_codes'].get('normalizer')==0
                and (not args.sdk_cpu_list or bool(affinity_receipt and affinity_receipt['passed']))
                and candidate_loaded and candidate_loaded['passed'])
            result['sdk16_candidate_contract']=candidate
            if openvins:
                result['estimator_inertial_state']=result.pop('sdk16_inertial_state')
                result['estimator_loaded_libraries']=result.pop('sdk16_loaded_libraries')
                result['estimator_candidate_checks_passed']=result.pop('sdk16_candidate_checks_passed')
                result['estimator_candidate_contract']=result.pop('sdk16_candidate_contract')
            result['flight_authorized']=False
            if args.experimental_chain:
                result['experimental_mission_passed']=bool(result['sdk16_candidate_checks_passed']
                    and result.get('flight',{}).get('passed'))
                if args.warehouse_variant=='avoidance' and not args.flight_case.startswith('local-land-'):
                    result['experimental_mission_passed'] &= (result.get('flight',{}).get('action_status')==4
                        and result.get('flight',{}).get('observation_task_succeeded') is True)
                result['scope']='owned SDK16/MTF simulation mission; covariance qualification deferred; not hardware acceptance'
                result['covariance_qualification_deferred']=True
                result['global_covariance_qualified']=False
                result['passed']=result['experimental_mission_passed']
                if args.flight_case.startswith('local-land-'):
                    result['experimental_mission_passed']=False
                    result['no_gnss_landing_case_passed']=bool(result['sdk16_candidate_checks_passed'] and result.get('flight',{}).get('passed'))
                    result['passed']=result['no_gnss_landing_case_passed']
            else:
                result['passed']=False  # Experimental diagnostics never qualify the flight source.
        if args.sdk_profile:
            result['sdk_profile_report_present']=(out/'sdk-trace.nsys-rep').is_file()
            result['sdk_profile_checks_passed']=result['passed'] and result['sdk_profile_report_present']
            result['passed']=False
        if args.manual_only:
            from vio_ui_lifecycle import manual_session_result
            result=manual_session_result(result)
        write_json(out/'result.json',result)
    if args.manual_only:
        print('Pocket 手动会话：', '正常结束' if result.get('manual_session_passed') else 'FAIL', out, flush=True)
    elif args.sdk_candidate:
        print(('No-GNSS local landing case:' if args.flight_case.startswith('local-land-') else
            ('OpenVINS' if openvins else 'SDK16')+(' experimental simulation flight:' if args.experimental_chain else ' disarmed diagnostic:')), 'PASS' if result.get('no_gnss_landing_case_passed',result.get('experimental_mission_passed',result.get('estimator_candidate_checks_passed',result.get('sdk16_candidate_checks_passed')))) else 'FAIL',
              '(flight qualification pending)',out,flush=True)
    else:
        print('Stereo/IMU VIO:', 'PASS' if result['passed'] else 'FAIL',out,flush=True)
    if result.get('error'):
        print('失败原因：' + result['error'], file=sys.stderr, flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    guard = ROOT/'.cache/simulation/px4-smoke.lock'
    guard.parent.mkdir(parents=True,exist_ok=True)
    with guard.open('a') as handle:
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            raise SystemExit(main())
        except (ValueError, OSError, RuntimeError) as exc:
            print(f'VIO 启动检查失败：{exc}', file=sys.stderr, flush=True)
            raise SystemExit(1)
