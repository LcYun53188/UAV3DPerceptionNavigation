#!/usr/bin/env python3
"""Owned disarmed official PX4/Gazebo/QGC with independent stereo + IMU VIO."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import signal
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
from vio_render_device import environment as render_environment,capture as capture_renderer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sdk-debug-dump',action='store_true',help='Explicit SDK-consumed input dump; diagnostic load, not VIO qualification')
    parser.add_argument('--diagnostic-visual-only',action='store_true',help='Motion-only SDK stereo comparison; no normalized source, no VIO acceptance')
    parser.add_argument('--duration',type=float,default=45.)
    parser.add_argument('--scene',choices=('planar','layered','warehouse'),default='planar')
    parser.add_argument('--camera-pitch-deg',type=int,choices=(0,15),default=0,help='Explicit sensor pitch comparison with regenerated optical TF')
    parser.add_argument('--warehouse-floor-texture',action='store_true',help='Explicit near-field floor texture comparison, warehouse only')
    parser.add_argument('--motion',action='store_true',help='Independent force-driven sensor carrier; PX4 remains disarmed')
    parser.add_argument('--ui',action='store_true')
    parser.add_argument('--image-resolution',choices=('640x400','480x300'),default='640x400',help='Explicit simulated stereo resolution with regenerated calibration')
    parser.add_argument('--quality-policy',choices=('strict','bounded_gap'),default='strict',help='Explicit covariance sample rejection policy; no freshness extension')
    parser.add_argument('--ekf-delay-max-ms',type=int,choices=(160,200),default=None,help='Explicit disarmed EKF delayed-horizon comparison; no EV timestamp offset')
    parser.add_argument('--sdk-image-depth',type=int,choices=range(1,11),default=None,help='Explicit SDK image subscription depth for disarmed fusion timing comparison')
    parser.add_argument('--headless-rendering',action='store_true',help='Use Gazebo native offscreen renderer for disarmed VIO fusion')
    parser.add_argument('--render-device',choices=('default','nvidia'),default='default',help='Select owned Gazebo renderer; NVIDIA requires fresh driver confirmation')
    parser.add_argument('--real-time-factor',type=float,default=None,help='Explicit [0.8,1.0] simulation pacing for disarmed pose fusion only')
    parser.add_argument('--fuse-pose',action='store_true',help='Same-aircraft actual SDK pose -> PX4 EV; disarmed audit with source stop')
    parser.add_argument('--normalize',action='store_true',help='Audit reviewed SDK pose normalization; EV requires --fuse-pose')
    parser.add_argument('--reset-source',action='store_true',help='Retire normalized source then reset actual SDK; disarmed only')
    args = parser.parse_args()
    if args.diagnostic_visual_only and (not args.motion or args.normalize or args.fuse_pose or args.reset_source):
        parser.error('--diagnostic-visual-only requires --motion without normalize/fuse/reset')
    if args.warehouse_floor_texture and args.scene!='warehouse':
        parser.error('--warehouse-floor-texture requires --scene warehouse')
    if args.ekf_delay_max_ms is not None and not args.fuse_pose:
        parser.error('--ekf-delay-max-ms requires --fuse-pose')
    if args.sdk_image_depth is not None and not args.fuse_pose:
        parser.error('--sdk-image-depth requires --fuse-pose')
    if args.headless_rendering and not args.fuse_pose:
        parser.error('--headless-rendering requires --fuse-pose')
    if args.render_device!='default' and not args.fuse_pose:
        parser.error('--render-device nvidia requires --fuse-pose')
    if args.real_time_factor is not None and (not args.fuse_pose or not .8<=args.real_time_factor<=1.):
        parser.error('--real-time-factor requires --fuse-pose and a value within [0.8,1.0]')
    if args.fuse_pose and (not args.normalize or args.motion or args.reset_source or args.duration<40):
        parser.error('--fuse-pose requires --normalize, duration >=40 s, no carrier motion or reset')
    if args.scene!='planar' and not (args.motion or args.fuse_pose):
        parser.error('--scene layered/warehouse requires --motion or disarmed --fuse-pose')
    if args.motion and (not (args.normalize or args.diagnostic_visual_only) or args.reset_source or args.duration < 50):
        parser.error('--motion requires --normalize, duration >=50 s, and no reset')
    if args.reset_source and (not args.normalize or args.duration < 35):
        parser.error('--reset-source requires --normalize and duration >=35 s')
    if not 20 <= args.duration <= 120: parser.error('duration must be within [20,120] s')
    lock = read(ROOT/'simulation/px4/versions.lock.yaml')
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
    if args.fuse_pose:
        from build_px4_vio import BUILD,verify
        vio_build=verify(lock);build=BUILD
    models = px4/'Tools/simulation/gz/models'
    upstream_world = px4/'Tools/simulation/gz/worlds/default.sdf'
    motion_plugin = ROOT/'.deps/vio-motion-build/libvio_motion_carrier.so' if args.motion else None
    if motion_plugin is not None and not motion_plugin.is_file():
        raise RuntimeError('Build the independent fixture first: ./scripts/build_vio_motion.sh')
    if motion_plugin is not None:
        receipt = read(motion_plugin.parent/'build-inputs.json')
        for name,digest in receipt.items():
            if file_hash(ROOT/name) != digest:
                raise RuntimeError('Motion fixture build drift: '+name)
    profile,frames = assets(out/'assets',upstream_world,motion_plugin=motion_plugin,scene=args.scene,real_time_factor=args.real_time_factor,resolution=tuple(map(int,args.image_resolution.split('x'))),warehouse_floor_texture=args.warehouse_floor_texture,camera_pitch_deg=args.camera_pitch_deg)
    env = dict(os.environ,ROS_DOMAIN_ID='78',ROS_LOCALHOST_ONLY='1',GZ_DISTRO='harmonic',
        GZ_PARTITION='uav_vio_sensors_'+run_id,GZ_IP='127.0.0.1',
        PX4_SIM_MODEL='gz_'+('x500' if args.motion else profile['model']),PX4_SYS_AUTOSTART='4001',
        PX4_GZ_STANDALONE='1',PX4_GZ_WORLD='default',HEADLESS='1',
        PX4_UXRCE_DDS_PORT='8898',PX4_PARAM_UXRCE_DDS_SYNCT='0',
        PX4_GZ_MODELS=str(models if args.motion else out/'assets'),PX4_GZ_WORLDS=str(out/'assets'),
        GZ_SIM_RESOURCE_PATH=str(out/'assets')+':'+str(models),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(build/'src/modules/simulation/gz_plugins'),
        GZ_SIM_SERVER_CONFIG_PATH=str(ROOT/'simulation/px4/server_control.config'))
    fusion_profile=None
    if args.fuse_pose:
        fusion_profile=read(ROOT/'simulation/px4/vio/pose_fusion.json')
        if fusion_profile['schema']!=1 or fusion_profile['profile']!='aligned_pose_v1':
            raise RuntimeError('Unsupported pose fusion profile')
        env.update({'PX4_PARAM_'+k:str(v) for k,v in fusion_profile['parameters'].items()})
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
    params.update(stream_parameters(args.sdk_image_depth if args.sdk_image_depth is not None else 10))
    (out/'vio-params.yaml').write_text(json.dumps({'visual_slam':{'ros__parameters':params}},indent=2)+'\n')
    binary = ROOT/'install_uav/isaac_ros_visual_slam/lib/isaac_ros_visual_slam/isaac_ros_visual_slam'
    inputs = [Path(__file__),ROOT/'scripts/px4_vio_sensor_audit.py',ROOT/'scripts/vio_sensor_transforms.py',ROOT/'scripts/prepare_vio_sensor_assets.py',
              ROOT/'scripts/vio_sensor_quality.py',ROOT/'scripts/run_px4_vio_sensors.sh',ROOT/'scripts/vio_render_device.py',ROOT/'scripts/vio_sdk_parameters.py',ROOT/'scripts/vio_sdk_runtime.py',ROOT/'scripts/vio_pose_window.py',
              ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam/include/isaac_ros_visual_slam/impl/message_stream_sequencer.hpp',
              ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam/include/isaac_ros_visual_slam/impl/stopwatch.hpp',
              ROOT/'src/isaac_ros_common/isaac_ros_common/src/qos.cpp',
              ROOT/'src/px4_comm_bridge/px4_comm_bridge/source_timing.py',ROOT/'src/px4_comm_bridge/px4_comm_bridge/pose_dropout.py',
              PROFILE,upstream_world,models/'x500/model.sdf',models/'x500_base/model.sdf',
              out/'assets/default.sdf',out/'assets'/profile['model']/'model.sdf',out/'assets/frames.json',
              out/'vio-params.yaml',ROOT/'simulation/px4/server_control.config',binary,
              ROOT/'install_uav/isaac_ros_visual_slam/lib/libvisual_slam_node.so',
              ROOT/'install_uav/isaac_ros_visual_slam/lib/libcuvslam.so']
    if args.scene=='warehouse':
        from vio_warehouse_scene import LAYOUT
        inputs += [LAYOUT,ROOT/'scripts/vio_warehouse_scene.py',out/'assets/warehouse-layout.json']
        inputs += sorted((out/'assets/warehouse_textures').rglob('*.png'))
    if args.motion:
        inputs += [motion_plugin,ROOT/'simulation/px4/vio/motion/MotionCarrier.cc',
                   ROOT/'simulation/px4/vio/motion/CMakeLists.txt',ROOT/'scripts/build_vio_motion.sh',
                   motion_plugin.parent/'build-inputs.json',ROOT/'scripts/record_vio_motion_build.py',
                   ROOT/'scripts/px4_vio_motion_audit.py']
    if args.normalize:
        reviewed = ROOT/'simulation/px4/vio/pose_contract.json'
        approved = read(reviewed)
        if approved['schema'] != 1 or approved['contract'] != 'cuvslam15_right_tangent_base_link_v1':
            raise RuntimeError('Unsupported SDK pose contract')
        for name,digest in approved['expected_sha256'].items():
            if file_hash(ROOT/name) != digest:
                raise RuntimeError('Reviewed SDK pose contract drift: '+name)
        inputs.append(reviewed)
        inputs += [ROOT/'src/px4_comm_bridge/px4_comm_bridge/cuvslam_pose.py',
            ROOT/'src/px4_comm_bridge/px4_comm_bridge/cuvslam_pose_node.py',ROOT/'scripts/px4_vio_pose_audit.py',
            ROOT/'src/isaac_ros_nitros/isaac_ros_nitros/lib/cuvslam/include/cuvslam/cuvslam2.h',
            ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/cuvslam_ros_conversion.cpp',
            ROOT/'src/isaac_ros_visual_slam/isaac_ros_visual_slam/src/impl/visual_slam_impl.cpp']
    write_json(out/'calibration.json',dict(schema=1,profile=profile,frames=frames,parameters=params,
        source_contract='cuvslam15_right_tangent_base_link_v1',quality_policy=args.quality_policy,render_device=args.render_device,headless_rendering=args.headless_rendering,
        native_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in inputs
                       if 'libvisual_slam_node.so' in str(p) or 'libcuvslam.so' in str(p)
                       or 'cuvslam2.h' in str(p) or str(p).endswith('cuvslam_ros_conversion.cpp')
                       or str(p).endswith('visual_slam_impl.cpp')}))
    calibration = file_hash(out/'calibration.json')
    inputs.append(out/'calibration.json')
    if args.fuse_pose:
        anchor=dict(schema=1,scope='fixed configured local launch origin, disarmed same-aircraft audit only',
            position_enu=[0.,0.,0.],yaw_enu=0.,height_reference='launch body origin, not terrain altitude',
            calibration_id=calibration,scene_sha256=file_hash(out/'assets/default.sdf'),
            model_sha256=file_hash(out/'assets'/profile['model']/'model.sdf'),
            fusion_profile_sha256=file_hash(ROOT/'simulation/px4/vio/pose_fusion.json'))
        write_json(out/'anchor.json',anchor)
        inputs += [out/'anchor.json',ROOT/'simulation/px4/vio/pose_fusion.json',
            ROOT/'scripts/px4_vio_real_fusion.py',ROOT/'src/px4_comm_bridge/px4_comm_bridge/pose_stream.py',
            ROOT/'src/px4_comm_bridge/px4_comm_bridge/pose_fusion.py',ROOT/'src/px4_comm_bridge/px4_comm_bridge/vio_input.py',
            ROOT/'src/uav_mission/uav_mission/vio_gate.py',ROOT/'scripts/build_px4_vio.py',
            build/'bin/px4',build/'vio-build.json']
    manifest = dict(run_id=run_id,scope='disarmed stereo/IMU VIO reference, NOT Pro W calibration or VIO flight',
        partition=env['GZ_PARTITION'],domain=78,duration_s=args.duration,ui=args.ui,model=profile['model'] if args.motion else profile['model']+'_7',versions=lock,profile=profile,
        render_device=args.render_device,headless_rendering=args.headless_rendering,quality_policy=args.quality_policy,requested_sdk_image_depth=args.sdk_image_depth,requested_ekf_delay_max_ms=args.ekf_delay_max_ms,requested_real_time_factor=args.real_time_factor,normalize=args.normalize,fuse_pose=args.fuse_pose,vio_build=vio_build,
        px4_parameter_overrides={k:v for k,v in env.items() if k.startswith('PX4_PARAM_')},motion=args.motion,sdk_debug_dump=args.sdk_debug_dump,diagnostic_visual_only=args.diagnostic_visual_only,reset_source=args.reset_source,calibration_id=calibration,
        input_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in inputs},processes={})
    processes,logs = [],[]
    def launch(name,command,environment=env):
        log = (out/(name+'.log')).open('w');logs.append(log)
        process = subprocess.Popen(list(map(str,command)),cwd=ROOT,env=environment,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
        manifest['processes'][name] = dict(pid=process.pid,command=list(map(str,command)),started_system_ns=time.time_ns())
        write_json(out/'manifest.json',manifest)
        return process
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node = rclpy.create_node('vio_sensor_observer')
    audit = SensorAudit(node,profile,frames,allow_pose_fusion=args.fuse_pose)
    runtime_receipt=SdkRuntimeReceipt(node,{k:params[k] for k in (
        "image_buffer_size","imu_buffer_size","image_qos","image_qos_depth",
        "tracking_mode","num_cameras","min_num_images","camera_optical_frames")})
    fusion_audit=None
    pose_audit = PoseAudit(node) if args.normalize or args.diagnostic_visual_only else None
    motion_audit = MotionAudit(node) if args.motion else None
    result = dict(passed=False)
    try:
        launch('agent',[ROOT/'.deps/microxrce-install/bin/MicroXRCEAgent','udp4','-p','8898'])
        gazebo_env=render_environment(env,args.render_device)
        launch('gazebo',['gz','sim','-r','-s']+(['--headless-rendering'] if args.headless_rendering else [])+[out/'assets/default.sdf'],gazebo_env)
        if args.ui: launch('gazebo_gui',['gz','sim','-g'],gazebo_env)
        launch('sensor_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/vio/left/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/vio/right/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/vio/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/vio/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/vio/imu@sensor_msgs/msg/Imu[gz.msgs.IMU']+
            (['/vio/truth@nav_msgs/msg/Odometry[gz.msgs.Odometry'] if args.motion else []))
        normalizer=None
        def launch_vio():
            nonlocal normalizer
            reset_remap = ['-r','visual_slam/reset:=/visual_slam/internal/reset'] if args.normalize else []
            launch('vio',[binary,'--ros-args','--params-file',out/'vio-params.yaml',
                '-r','visual_slam/image_0:=/vio/left/image','-r','visual_slam/image_1:=/vio/right/image',
                '-r','visual_slam/camera_info_0:=/vio/left/camera_info','-r','visual_slam/camera_info_1:=/vio/right/camera_info',
                '-r','visual_slam/imu:=/vio/imu']+reset_remap)
            if args.normalize:
                normalizer=launch('normalizer',['ros2','run','px4_comm_bridge','cuvslam_pose_node','--ros-args',
                    '-p','use_sim_time:=true','-p','calibration_id:='+calibration,
                    '-p','source_contract:=cuvslam15_right_tangent_base_link_v1','-p','quality_policy:='+args.quality_policy]+
                    (['-p','timing_path:='+str(out/'normalizer-timing.json')] if args.fuse_pose else []))
        # A stationary VIO-only SDK can retain its unknown initial covariance.
        # Observe the aircraft's physical spawn/settling; never fabricate motion.
        if not args.motion: launch_vio()
        launch('px4',[build/'bin/px4','-d','-i','7','-w',out/'rootfs',build/'etc'])
        config = out/'qgc-config/QGroundControl';config.mkdir(parents=True)
        (config/'QGroundControl.ini').write_text('[AutoConnect]\nautoConnectUDP=true\nautoConnectPixhawk=false\nautoConnectSiKRadio=false\nautoConnectRTKGPS=false\nautoConnectLibrePilot=false\n')
        qgc_env = dict(env,XDG_CONFIG_HOME=str(out/'qgc-config'),XDG_CACHE_HOME=str(out/'qgc-cache'),
                       QT_QPA_PLATFORM='xcb' if args.ui else 'offscreen')
        launch('qgc',[lock['artifacts']['qgc']['path'],'--allow-multiple','--log-output',
            '--logging','Vehicle.MultiVehicleManager,Vehicle.VehicleLinkManager'],qgc_env)
        if args.motion or args.fuse_pose:
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
            if args.fuse_pose:
                from px4_vio_real_fusion import RealPoseFusionAudit
                fusion_audit=RealPoseFusionAudit(node,calibration,anchor,file_hash(out/'anchor.json'))
            if args.motion:launch_vio()
        until = time.monotonic()+args.duration
        before_reset = None
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.05)
            if args.reset_source and pose_audit.reset_time is None and time.monotonic() > until-8.:
                before_reset = audit.result()
                before_reset['normalized_pose'] = pose_audit.result(calibration)
                result = before_reset
                if not before_reset['passed'] or not before_reset['normalized_pose']['passed']:
                    raise RuntimeError('Sensor/normalized source not ready before reset')
                pose_audit.request_reset()
        result = before_reset or audit.result()
        if motion_audit is not None:
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
        if args.normalize:
            result['normalized_pose'] = pose_audit.result(calibration)
            result['passed'] &= result['normalized_pose']['passed']
        if args.diagnostic_visual_only:
            result['diagnostic_passed']=result['passed']
            result['passed']=False  # A diagnostic never qualifies warehouse VIO.
        result['qgc_connected'] = bool(re.search(r'Adding new vehicle.*\"UDP Link \(AutoConnect\)\" 8 1 12 2',
                                                     (out/'qgc.log').read_text(errors='replace')))
        result['passed'] &= result['qgc_connected'] and all(p.poll() is None for p in processes)
        if fusion_audit is not None:
            fusion_audit.mark_stop()
            # Actually terminate the normalized source, keep the SDK running.
            os.killpg(normalizer.pid,signal.SIGINT)
            drain_until=time.monotonic()+6.
            while time.monotonic()<drain_until:rclpy.spin_once(node,timeout_sec=.02)
        for side in ('left','right'):
            image = audit.last.get(side)
            if image is not None and image.encoding == 'rgb8' and len(image.data) == image.step*image.height:
                pixels = bytes(image.data)
                rows = b''.join(pixels[i*image.step:i*image.step+3*image.width] for i in range(image.height))
                (out/(side+'.ppm')).write_bytes(f'P6\n{image.width} {image.height}\n255\n'.encode()+rows)
    except Exception as exc:
        result.update(passed=False,error=str(exc))
    finally:
        if fusion_audit is not None:
            renderer_log=Path.home()/'.gz/rendering/ogre2.log'
            renderer=capture_renderer(renderer_log,manifest['processes']['gazebo']['started_system_ns'],args.render_device)
            write_json(out/'renderer-info.json',renderer)
            if renderer_log.is_file():(out/'renderer.log').write_bytes(renderer_log.read_bytes())
            result['renderer']=renderer
            result['passed'] &= renderer['passed']
            result['real_pose_fusion']=fusion_audit.result()
            result['passed'] &= result['real_pose_fusion']['passed']
        node.destroy_node();rclpy.try_shutdown()
        for process in reversed(processes): stop(process)
        for log in logs: log.close()
        try:
            if audit.timing is not None:audit.timing.write(out/'sensor-timing.json')
            if fusion_audit is not None:
                fusion_audit.timing.write(out/'fusion-timing.json')
                for name,evidence in fusion_audit.evidence().items():write_json(out/name,evidence)
            write_json(out/'odometry.json',list(audit.records))
            write_json(out/'samples.json',{k:list(v) for k,v in audit.samples.items()})
            if pose_audit is not None:
                write_json(out/'normalized-poses.json',list(pose_audit.poses))
                write_json(out/'normalized-status.json',list(pose_audit.statuses))
                if motion_audit is not None:
                    write_json(out/'truth.json',motion_audit.truth)
                    write_json(out/'motion-imu.json',motion_audit.imu)
                write_json(out/'sdk-poses.json',[dict(stamp=t,covariance=list(m.pose.covariance),
                    position=[m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z],
                    quaternion=[m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w])
                    for t,m in pose_audit.raw.items()])
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
        result['passed'] &= result['cleanup_confirmed']
        result['sdk_runtime_receipt']=runtime_receipt.result()
        write_json(out/'sdk-runtime-receipt.json',result['sdk_runtime_receipt'])
        result['passed'] &= result['sdk_runtime_receipt']['passed']
        if args.sdk_debug_dump:
            result['sdk_dump_checks_passed']=result['passed']
            result['passed']=False  # Recording load is diagnostic, never qualification.
        write_json(out/'result.json',result)
    print('Stereo/IMU VIO:', 'PASS' if result['passed'] else 'FAIL',out,flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    guard = ROOT/'.cache/simulation/px4-smoke.lock'
    guard.parent.mkdir(parents=True,exist_ok=True)
    with guard.open('a') as handle:
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        raise SystemExit(main())
