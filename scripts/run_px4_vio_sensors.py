#!/usr/bin/env python3
"""Owned disarmed official PX4/Gazebo/QGC with independent stereo + IMU VIO."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration',type=float,default=45.)
    parser.add_argument('--ui',action='store_true')
    parser.add_argument('--normalize',action='store_true',help='Audit reviewed SDK pose normalization; no EV output')
    parser.add_argument('--reset-source',action='store_true',help='Retire normalized source then reset actual SDK; disarmed only')
    args = parser.parse_args()
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
    models = px4/'Tools/simulation/gz/models'
    upstream_world = px4/'Tools/simulation/gz/worlds/default.sdf'
    profile,frames = assets(out/'assets',upstream_world)
    env = dict(os.environ,ROS_DOMAIN_ID='78',ROS_LOCALHOST_ONLY='1',GZ_DISTRO='harmonic',
        GZ_PARTITION='uav_vio_sensors_'+run_id,GZ_IP='127.0.0.1',
        PX4_SIM_MODEL='gz_'+profile['model'],PX4_SYS_AUTOSTART='4001',
        PX4_GZ_STANDALONE='1',PX4_GZ_WORLD='default',HEADLESS='1',
        PX4_UXRCE_DDS_PORT='8898',PX4_PARAM_UXRCE_DDS_SYNCT='0',
        PX4_GZ_MODELS=str(out/'assets'),PX4_GZ_WORLDS=str(out/'assets'),
        GZ_SIM_RESOURCE_PATH=str(out/'assets')+':'+str(models),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(build/'src/modules/simulation/gz_plugins'),
        GZ_SIM_SERVER_CONFIG_PATH=str(ROOT/'simulation/px4/server_control.config'))
    os.environ.update({k:env[k] for k in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','GZ_PARTITION','GZ_IP')})
    params = dict(use_sim_time=True,num_cameras=2,min_num_images=2,tracking_mode=1,
        image_qos='DEFAULT',
        enable_localization_n_mapping=False,rectified_images=True,
        sync_matching_threshold_ms=1.,image_jitter_threshold_ms=60.,imu_jitter_threshold_ms=12.,
        calibration_frequency=float(profile['imu_rate_hz']),image_buffer_size=100,imu_buffer_size=400,
        gyro_noise_density=.000244,gyro_random_walk=.000019393,
        accel_noise_density=.001862,accel_random_walk=.003,
        base_frame='base_link',odom_frame='odom',map_frame='map',imu_frame='vio_imu',
        camera_optical_frames=['vio_left_optical','vio_right_optical'],
        publish_map_to_odom_tf=False,publish_odom_to_base_tf=False)
    (out/'vio-params.yaml').write_text(json.dumps({'visual_slam':{'ros__parameters':params}},indent=2)+'\n')
    binary = ROOT/'install_uav/isaac_ros_visual_slam/lib/isaac_ros_visual_slam/isaac_ros_visual_slam'
    inputs = [Path(__file__),ROOT/'scripts/px4_vio_sensor_audit.py',ROOT/'scripts/prepare_vio_sensor_assets.py',
              ROOT/'scripts/vio_sensor_quality.py',ROOT/'scripts/run_px4_vio_sensors.sh',
              PROFILE,upstream_world,models/'x500/model.sdf',models/'x500_base/model.sdf',
              out/'assets/default.sdf',out/'assets'/profile['model']/'model.sdf',out/'assets/frames.json',
              out/'vio-params.yaml',ROOT/'simulation/px4/server_control.config',binary,
              ROOT/'install_uav/isaac_ros_visual_slam/lib/libvisual_slam_node.so',
              ROOT/'install_uav/isaac_ros_visual_slam/lib/libcuvslam.so']
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
        source_contract='cuvslam15_right_tangent_base_link_v1',
        native_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in inputs
                       if 'libvisual_slam_node.so' in str(p) or 'libcuvslam.so' in str(p)
                       or 'cuvslam2.h' in str(p) or str(p).endswith('cuvslam_ros_conversion.cpp')
                       or str(p).endswith('visual_slam_impl.cpp')}))
    calibration = file_hash(out/'calibration.json')
    inputs.append(out/'calibration.json')
    manifest = dict(run_id=run_id,scope='disarmed stereo/IMU VIO reference, NOT Pro W calibration or VIO flight',
        partition=env['GZ_PARTITION'],domain=78,duration_s=args.duration,ui=args.ui,model=profile['model']+'_7',versions=lock,profile=profile,
        normalize=args.normalize,reset_source=args.reset_source,calibration_id=calibration,
        input_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in inputs},processes={})
    processes,logs = [],[]
    def launch(name,command,environment=env):
        log = (out/(name+'.log')).open('w');logs.append(log)
        process = subprocess.Popen(list(map(str,command)),cwd=ROOT,env=environment,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(process)
        manifest['processes'][name] = dict(pid=process.pid,command=list(map(str,command)))
        write_json(out/'manifest.json',manifest)
    rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
    node = rclpy.create_node('vio_sensor_observer')
    audit = SensorAudit(node,profile,frames)
    pose_audit = PoseAudit(node) if args.normalize else None
    result = dict(passed=False)
    try:
        launch('agent',[ROOT/'.deps/microxrce-install/bin/MicroXRCEAgent','udp4','-p','8898'])
        launch('gazebo',['gz','sim','-r','-s',out/'assets/default.sdf'])
        if args.ui: launch('gazebo_gui',['gz','sim','-g'])
        launch('sensor_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/vio/left/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/vio/right/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/vio/left/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/vio/right/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/vio/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'])
        reset_remap = ['-r','visual_slam/reset:=/visual_slam/internal/reset'] if args.normalize else []
        launch('vio',[binary,'--ros-args','--params-file',out/'vio-params.yaml',
            '-r','visual_slam/image_0:=/vio/left/image','-r','visual_slam/image_1:=/vio/right/image',
            '-r','visual_slam/camera_info_0:=/vio/left/camera_info','-r','visual_slam/camera_info_1:=/vio/right/camera_info',
            '-r','visual_slam/imu:=/vio/imu']+reset_remap)
        if args.normalize:
            launch('normalizer',['ros2','run','px4_comm_bridge','cuvslam_pose_node','--ros-args',
                '-p','use_sim_time:=true','-p','calibration_id:='+calibration,
                '-p','source_contract:=cuvslam15_right_tangent_base_link_v1'])
        launch('px4',[build/'bin/px4','-d','-i','7','-w',out/'rootfs',build/'etc'])
        config = out/'qgc-config/QGroundControl';config.mkdir(parents=True)
        (config/'QGroundControl.ini').write_text('[AutoConnect]\nautoConnectUDP=true\nautoConnectPixhawk=false\nautoConnectSiKRadio=false\nautoConnectRTKGPS=false\nautoConnectLibrePilot=false\n')
        qgc_env = dict(env,XDG_CONFIG_HOME=str(out/'qgc-config'),XDG_CACHE_HOME=str(out/'qgc-cache'),
                       QT_QPA_PLATFORM='xcb' if args.ui else 'offscreen')
        launch('qgc',[lock['artifacts']['qgc']['path'],'--allow-multiple','--log-output',
            '--logging','Vehicle.MultiVehicleManager,Vehicle.VehicleLinkManager'],qgc_env)
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
        if pose_audit is not None:
            result['normalized_pose'] = pose_audit.result(calibration)
            result['passed'] &= result['normalized_pose']['passed']
        result['qgc_connected'] = bool(re.search(r'Adding new vehicle.*\"UDP Link \(AutoConnect\)\" 8 1 12 2',
                                                     (out/'qgc.log').read_text(errors='replace')))
        result['passed'] &= result['qgc_connected'] and all(p.poll() is None for p in processes)
        for side in ('left','right'):
            image = audit.last.get(side)
            if image is not None and image.encoding == 'rgb8' and len(image.data) == image.step*image.height:
                pixels = bytes(image.data)
                rows = b''.join(pixels[i*image.step:i*image.step+3*image.width] for i in range(image.height))
                (out/(side+'.ppm')).write_bytes(f'P6\n{image.width} {image.height}\n255\n'.encode()+rows)
    except Exception as exc:
        result.update(passed=False,error=str(exc))
    finally:
        node.destroy_node();rclpy.try_shutdown()
        for process in reversed(processes): stop(process)
        for log in logs: log.close()
        try:
            write_json(out/'odometry.json',list(audit.records))
            write_json(out/'samples.json',{k:list(v) for k,v in audit.samples.items()})
            if pose_audit is not None:
                write_json(out/'normalized-poses.json',list(pose_audit.poses))
                write_json(out/'normalized-status.json',list(pose_audit.statuses))
                write_json(out/'sdk-poses.json',[dict(stamp=t,covariance=list(m.pose.covariance),
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
        write_json(out/'result.json',result)
    print('Stereo/IMU VIO:', 'PASS' if result['passed'] else 'FAIL',out,flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    guard = ROOT/'.cache/simulation/px4-smoke.lock'
    guard.parent.mkdir(parents=True,exist_ok=True)
    with guard.open('a') as handle:
        fcntl.flock(handle,fcntl.LOCK_EX|fcntl.LOCK_NB)
        raise SystemExit(main())
