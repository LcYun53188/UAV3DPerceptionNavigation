#!/usr/bin/env python3
"""Owned Gazebo RGB-D person test with known static camera pose; no PX4 or flight."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import StaticTransformBroadcaster
from visualization_msgs.msg import MarkerArray
from uav_nav_interfaces.msg import MapSnapshot, PersonObservationArray
from vision_msgs.msg import Detection2DArray
from uav_target_perception.gazebo import (asset_spec, mesh_triangles, person_element,
                                          pose_matrix, surface_distance, verify_asset)
from uav_target_perception.node import PersonMapper

ROOT = Path(__file__).resolve().parents[3]
CASES = {'front': [0., 0., 1., 0., 0., 0.],
         'offset': [0., -1., 1.25, 0., .095, math.atan2(1., 3.)],
         'far': [-2., 0., 1., 0., 0., 0.],
         'empty': [0., 0., 1., 0., 0., 0.]}
PERSON_POSE = [3., 0., 0., 0., 0., -math.pi/2]


def world_file(path, asset, case, posture='standing'):
    tree = ET.parse(ROOT/'.deps/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf')
    world = tree.getroot().find('world')
    world.set('name', 'person_fixture')
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'),
                            ('scene-broadcaster', 'SceneBroadcaster'), ('sensors', 'Sensors')]:
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{filename}-system', name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    if case != 'empty':
        model = person_element(asset, PERSON_POSE, posture)
        world.append(model)
    camera = ET.SubElement(world, 'model', name='fixture_camera')
    ET.SubElement(camera, 'static').text = 'true'
    ET.SubElement(camera, 'pose').text = ' '.join(map(str, CASES[case]))
    sensor = ET.SubElement(ET.SubElement(camera, 'link', name='sensor'), 'sensor', name='rgbd', type='rgbd_camera')
    for name, value in [('always_on', 'true'), ('update_rate', '10'), ('topic', '/person_fixture/rgbd'), ('gz_frame_id', 'fixture_optical')]:
        ET.SubElement(sensor, name).text = value
    content = ET.SubElement(sensor, 'camera')
    ET.SubElement(content, 'horizontal_fov').text = '1.21'
    ET.SubElement(content, 'optical_frame_id').text = 'fixture_optical'
    # RGBD publishes its default <topic>/camera_info, unlike CameraSensor.
    image = ET.SubElement(content, 'image')
    for name, value in [('width', '320'), ('height', '240'), ('format', 'R8G8B8')]:
        ET.SubElement(image, name).text = value
    clip = ET.SubElement(content, 'clip')
    ET.SubElement(clip, 'near').text = '0.1'
    ET.SubElement(clip, 'far').text = '20'
    tree.write(path, encoding='utf-8', xml_declaration=True)
    return list(map(float, model.findtext('pose').split())) if case != 'empty' else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--case', choices=CASES, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New evidence directory')
    parser.add_argument('--posture', choices=['standing', 'sitting', 'lying'], default='standing')
    parser.add_argument('--asset', type=Path)
    parser.add_argument('--model', type=Path, default=ROOT/'.cache/models/person/yolox_tiny.onnx')
    parser.add_argument('--person-rotations', action='store_true')
    args = parser.parse_args()
    kind = 'sitting' if args.posture == 'sitting' else 'standing'
    spec = asset_spec(kind)
    asset = verify_asset(args.asset or ROOT/'.cache/models/gazebo'/spec['directory']/f"v{spec['version']}", kind)
    if args.posture == 'sitting':
        for pose in CASES.values():
            pose[2] = .55
    if args.posture == 'lying':
        PERSON_POSE[5] = 0.
        CASES['front'] = [0., .8, 1.2, 0., .32, 0.]
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    world = args.output/'world.sdf'
    actual_person_pose = world_file(world, asset, args.case, args.posture)
    truth = mesh_triangles(asset, actual_person_pose, kind) if actual_person_pose is not None else None
    horizontal_limit = (max(.6, float(np.linalg.norm(np.ptp(truth[:, :, :2], axis=(0, 1)))/2)+.2)
                        if truth is not None else .6)
    partition = 'person_fixture_'+uuid.uuid4().hex
    env = dict(os.environ, GZ_PARTITION=partition, GZ_IP='127.0.0.1',
               GZ_SIM_RESOURCE_PATH=str(asset.parent), __NV_PRIME_RENDER_OFFLOAD='1',
               __GLX_VENDOR_LIBRARY_NAME='nvidia', __VK_LAYER_NV_optimus='NVIDIA_only')
    processes, handles = [], []
    node = mapper = executor = None
    result = dict(passed=False, case=args.case, posture=args.posture, camera_pose_world=CASES[args.case],
                  person_pose_world=actual_person_pose,
                  archive_sha256=spec['sha256'], partition=partition,
                  rotation_augmentation=args.person_rotations,
                  scope='actual Gazebo mesh RGB and depth; exact static fixture TF and synthetic map context; no VIO/PX4 acceptance',
                  limits=dict(surface_distance_m=.20, horizontal_mesh_center_distance_m=horizontal_limit, minimum_unique_observations=20))
    def launch(name, command):
        handle = (args.output/(name+'.log')).open('w')
        handles.append(handle)
        process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, process))
        return process
    try:
        launch('gazebo', ['gz', 'sim', '-r', '-s', '--headless-rendering', str(world)])
        launch('bridge', ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/person_fixture/rgbd/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/person_fixture/rgbd/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/person_fixture/rgbd/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '--ros-args', '-r', '/person_fixture/rgbd/image:=/rgbd_camera/image',
            '-r', '/person_fixture/rgbd/depth_image:=/rgbd_camera/depth_image',
            '-r', '/person_fixture/rgbd/camera_info:=/rgbd_camera/camera_info'])
        rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', f'model_path:={args.model.resolve()}',
                          '-p', f'rotation_augmentation:={str(args.person_rotations).lower()}',
                          '-p', f'snapshot_path:={args.output/"snapshot.json"}'])
        node, mapper = Node('person_fixture_observer'), PersonMapper()
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.add_node(mapper)
        context = node.create_publisher(MapSnapshot, '/uav/map/snapshot', QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        broadcaster = StaticTransformBroadcaster(node)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'fixture_optical'
        optical = pose_matrix(CASES[args.case]) @ pose_matrix([0, 0, 0, -math.pi/2, 0, -math.pi/2])
        t, q = transform.transform.translation, transform.transform.rotation
        t.x, t.y, t.z = map(float, optical[:3, 3])
        q.x, q.y, q.z, q.w = map(float, Rotation.from_matrix(optical[:3, :3]).as_quat())
        bridge = CvBridge()
        observations, keys, counts, markers, diagnostics, history_markers = [], set(), {}, [], {}, []
        def people(message):
            counts['people_arrays'] = counts.get('people_arrays', 0)+1
            counts['live_people'] = len(message.observations)
            diagnostics[message.reason] = diagnostics.get(message.reason, 0)+1
            for item in message.observations:
                key = (item.track_id, item.last_observed.sec, item.last_observed.nanosec)
                if key in keys:
                    continue
                keys.add(key)
                observations.append(dict(track_id=item.track_id, stamp=item.last_observed.sec+item.last_observed.nanosec/1e9,
                    position=[item.position.x, item.position.y, item.position.z], confidence=item.detection_confidence,
                    depth_samples=item.depth_samples, depth_spread_m=item.depth_spread_m))
        def detections(message):
            counts['detected_boxes'] = counts.get('detected_boxes', 0)+len(message.detections)
        def image(message, kind):
            counts[kind] = counts.get(kind, 0)+1
            target = args.output/(kind+'.jpg')
            if not target.exists() and counts[kind] >= 30 and (kind == 'rgb' or counts.get('detected_boxes', 0)):
                cv2.imwrite(str(target), bridge.imgmsg_to_cv2(message, desired_encoding='bgr8'))
        def camera_info(message):
            counts['camera_info'] = counts.get('camera_info', 0)+1
            if not (args.output/'camera-info.json').exists():
                (args.output/'camera-info.json').write_text(json.dumps(dict(frame=message.header.frame_id,
                    width=message.width, height=message.height, k=list(message.k), p=list(message.p), r=list(message.r), d=list(message.d)), indent=2)+'\n')
        def depth(message):
            counts['depth'] = counts.get('depth', 0)+1
            if not (args.output/'depth.npy').exists():
                np.save(args.output/'depth.npy', bridge.imgmsg_to_cv2(message, desired_encoding='passthrough'))
        node.create_subscription(PersonObservationArray, '/uav/people/observations', people, 10)
        node.create_subscription(Detection2DArray, '/uav/people/detections', detections, 10)
        node.create_subscription(MarkerArray, '/uav/people/markers', lambda m: markers.append(any(v.action == v.ADD for v in m.markers)), 10)
        node.create_subscription(MarkerArray, '/uav/people/history_markers', history_markers.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        node.create_subscription(Image, '/rgbd_camera/image', lambda m: image(m, 'rgb'), qos_profile_sensor_data)
        node.create_subscription(Image, '/uav/people/image', lambda m: image(m, 'detections'), qos_profile_sensor_data)
        node.create_subscription(Image, '/rgbd_camera/depth_image', depth, qos_profile_sensor_data)
        node.create_subscription(CameraInfo, '/rgbd_camera/camera_info', camera_info, qos_profile_sensor_data)
        start, last_context = time.monotonic(), 0.
        while time.monotonic()-start < 45:
            if any(p.poll() is not None for _, p in processes):
                raise RuntimeError('Gazebo or bridge exited early')
            executor.spin_once(timeout_sec=.02)
            now = time.monotonic()
            if now-last_context > .2 and node.get_clock().now().nanoseconds:
                stamp = node.get_clock().now().to_msg()
                snapshot = MapSnapshot(map_id='static-person-fixture-'+args.case, epoch=1, valid=True)
                snapshot.header.frame_id, snapshot.header.stamp = 'map', stamp
                context.publish(snapshot)
                transform.header.stamp = stamp
                broadcaster.sendTransform(transform)
                last_context = now
            if args.case != 'empty' and len(observations) >= 30 and any(markers):
                break
            if args.case == 'empty' and now-start > 15 and counts.get('depth', 0) >= 30:
                break
        if observations:
            # Remove actual RGB/depth transport and pause ROS clock publication.
            # Keep the known fixture coordinate context, then wait past live TTL.
            transport = next(p for name, p in processes if name == 'bridge')
            os.killpg(transport.pid, signal.SIGINT)
            transport.wait(timeout=5)
            until = time.monotonic()+3.
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=.02)
                if time.monotonic()-last_context > .2:
                    snapshot.header.stamp = node.get_clock().now().to_msg()
                    context.publish(snapshot)
                    last_context = time.monotonic()
            retained = [m for m in history_markers[-1].markers if m.action == m.ADD] if history_markers else []
            result['history_persistence_passed'] = bool(retained and counts.get('live_people') == 0
                and all(m.lifetime.sec == m.lifetime.nanosec == 0 for m in retained))
            result['retained_history_markers'] = len(retained)
        from std_srvs.srv import Trigger
        saved = mapper.save_snapshot(Trigger.Request(), Trigger.Response())
        result.update(counts=counts, observations=observations, diagnostic_counts=diagnostics,
                      marker_add_seen=any(markers), snapshot_saved=saved.success, reason=mapper.reason)
        if args.case != 'empty':
            triangles = truth
            for item in observations:
                item['surface_distance_m'] = surface_distance(item['position'], triangles)
                center = (triangles.min(axis=(0, 1))+triangles.max(axis=(0, 1)))/2
                item['horizontal_mesh_center_distance_m'] = float(np.linalg.norm(np.asarray(item['position'])[:2]-center[:2]))
            errors = [item['surface_distance_m'] for item in observations]
            result.update(mesh_bounds_min=triangles.min(axis=(0, 1)).tolist(), mesh_bounds_max=triangles.max(axis=(0, 1)).tolist(),
                          unique_observations=len(observations), max_surface_distance_m=max(errors, default=None),
                          median_surface_distance_m=float(np.median(errors)) if errors else None)
            result['passed'] = (len(observations) >= 20 and any(markers) and saved.success and result.get('history_persistence_passed')
                and all(item['surface_distance_m'] <= .20 and item['horizontal_mesh_center_distance_m'] <= horizontal_limit for item in observations))
        else:
            result['passed'] = (counts.get('depth', 0) >= 30 and counts.get('rgb', 0) >= 30
                and counts.get('camera_info', 0) >= 30 and mapper.reason == 'NO_VALID_PERSON_DEPTH'
                and not observations and not counts.get('detected_boxes', 0) and not any(markers))
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        if executor is not None:
            for child in (mapper, node):
                if child is not None:
                    executor.remove_node(child)
                    child.destroy_node()
            executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()
        for _, process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
        for _, process in reversed(processes):
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        for handle in handles:
            handle.close()
        result['root_exit_codes'] = {name: p.returncode for name, p in processes}
        groups = {p.pid for _, p in processes}
        remaining = []
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                fields = (entry/'stat').read_text().rsplit(')', 1)[1].split()
                if int(fields[2]) in groups and fields[0] != 'Z':
                    remaining.append(int(entry.name))
            except (OSError, ValueError):
                continue
        result['remaining_owned_processes'] = remaining
        result['cleanup_confirmed'] = not remaining and all(p.poll() is not None for _, p in processes)
        result['passed'] &= result['cleanup_confirmed']
        (args.output/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('observations', 'diagnostic_counts')}, indent=2))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
