#!/usr/bin/env python3
"""Check real cuVSLAM node/CUDA loading without camera or FMU publishers."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import rclpy

ROOT = Path(__file__).resolve().parents[1]


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '94':
        raise RuntimeError('Use isolated ROS_DOMAIN_ID=94 for the VIO load smoke')
    out = ROOT/'.cache/simulation/vio-node'/str(uuid.uuid4())
    out.mkdir(parents=True)
    binary = ROOT/'install_uav/isaac_ros_visual_slam/lib/isaac_ros_visual_slam/isaac_ros_visual_slam'
    library = ROOT/'install_uav/isaac_ros_visual_slam/lib/libvisual_slam_node.so'
    sdk = ROOT/'install_uav/isaac_ros_visual_slam/lib/libcuvslam.so'
    rclpy.init()
    node = rclpy.create_node('vio_load_observer')
    process = None
    result = dict(passed=False,scope='cuVSLAM executable/CUDA and DDS subscriptions only; no sensor, tracking or flight acceptance')
    try:
        with (out/'node.log').open('w') as log:
            process = subprocess.Popen([str(binary),'--ros-args','-r','__node:=visual_slam',
                '-p','use_sim_time:=true','-p','enable_localization_n_mapping:=false',
                '-p','tracking_mode:=1','-p','publish_odom_to_base_tf:=false'],
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            deadline = time.monotonic()+15
            subscriptions = set()
            while time.monotonic()<deadline and process.poll() is None:
                rclpy.spin_once(node,timeout_sec=.1)
                try:
                    subscriptions = {name for name,_ in node.get_subscriber_names_and_types_by_node('visual_slam','/')}
                except Exception:
                    continue
                if 'WarmUpGPU()' in (out/'node.log').read_text(errors='replace'):
                    break
            text = (out/'node.log').read_text(errors='replace')
            required = {'/visual_slam/image_0','/visual_slam/image_1',
                        '/visual_slam/camera_info_0','/visual_slam/camera_info_1','/visual_slam/imu'}
            fmu = [topic for topic,_ in node.get_topic_names_and_types() if '/fmu/in/' in topic
                   and node.get_publishers_info_by_topic(topic)]
            result.update(subscriptions=sorted(subscriptions),fmu_input_publishers=fmu,
                executable_alive=process.poll() is None,
                cuvslam_version=next((line.split('cuVSLAM version: ')[1] for line in text.splitlines()
                                      if 'cuVSLAM version: ' in line),None),
                cuda_warmup='WarmUpGPU()' in text,tracking_mode_vio='Tracking mode: VIO (IMU fusion)' in text,
                hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest()
                        for p in (binary,library,sdk,Path(__file__),ROOT/'scripts/build_vio_node.sh',
                                  ROOT/'simulation/px4/versions.lock.yaml')},
                source_commits={name:subprocess.check_output(['git','-C',str(ROOT/name),
                    'rev-parse','HEAD'],text=True).strip() for name in
                    ('src/isaac_ros_visual_slam','src/isaac_ros_nitros')})
            result['passed'] = (result['executable_alive'] and result['cuda_warmup']
                                and result['tracking_mode_vio'] and required <= subscriptions and not fmu)
    finally:
        if process is not None:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try: process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL)
                    process.wait(timeout=5)
            result['exit_code'] = process.returncode
            result['cleanup_confirmed'] = process.poll() is not None
            result['passed'] = result['passed'] and result['cleanup_confirmed'] and process.returncode in (0,-2)
        node.destroy_node()
        rclpy.shutdown()
        (out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print('VIO node load:', 'PASS' if result['passed'] else 'FAIL', out)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
