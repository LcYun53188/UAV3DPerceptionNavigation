"""Opt-in actual detector + ROS graph smoke, with explicitly synthetic depth/TF.

Set UAV_PERSON_MODEL and UAV_PERSON_IMAGE to local verified assets. This does
not qualify Gazebo geometry, physical RGB-D accuracy or flight performance.
"""
import json
import os
from pathlib import Path
import time

import numpy as np
import pytest

pytestmark = pytest.mark.skipif(not (os.environ.get('UAV_PERSON_MODEL') and os.environ.get('UAV_PERSON_IMAGE')),
                                reason='Opt-in test requires local model and person image assets')


def test_actual_rgb_inference_to_map_marker_and_context_reset(tmp_path):
    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from geometry_msgs.msg import TransformStamped
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image
    from std_srvs.srv import Trigger
    from tf2_ros import StaticTransformBroadcaster
    from uav_nav_interfaces.msg import MapSnapshot, PersonObservationArray
    from vision_msgs.msg import Detection2DArray
    from visualization_msgs.msg import MarkerArray
    from uav_target_perception.detector import PersonDetector
    from uav_target_perception.node import PersonMapper

    rclpy.init(args=['--ros-args', '-p', f'model_path:={os.environ["UAV_PERSON_MODEL"]}',
                      '-p', f'snapshot_path:={tmp_path/"people.json"}'])
    peer = mapper = None
    executor = SingleThreadedExecutor()
    try:
        mapper, peer = PersonMapper(), Node('person_pipeline_fixture')
        executor.add_node(mapper)
        executor.add_node(peer)
        rgb_pub = peer.create_publisher(Image, '/rgbd_camera/image', qos_profile_sensor_data)
        depth_pub = peer.create_publisher(Image, '/rgbd_camera/depth_image', qos_profile_sensor_data)
        info_pub = peer.create_publisher(CameraInfo, '/rgbd_camera/camera_info', qos_profile_sensor_data)
        context_pub = peer.create_publisher(MapSnapshot, '/uav/map/snapshot',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        results, detections, markers, history_markers = [], [], [], []
        peer.create_subscription(PersonObservationArray, '/uav/people/observations', results.append, 10)
        peer.create_subscription(Detection2DArray, '/uav/people/detections', detections.append, 10)
        peer.create_subscription(MarkerArray, '/uav/people/markers', markers.append, 10)
        peer.create_subscription(MarkerArray, '/uav/people/history_markers', history_markers.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        broadcaster = StaticTransformBroadcaster(peer)
        transform = TransformStamped()
        transform.header.frame_id, transform.child_frame_id = 'map', 'fixture_optical'
        transform.transform.translation.x = 1.
        transform.transform.translation.y = 2.
        transform.transform.translation.z = 3.
        transform.transform.rotation.z = transform.transform.rotation.w = float(np.sqrt(.5))
        image = cv2.imread(os.environ['UAV_PERSON_IMAGE'])
        assert image is not None
        height, width = image.shape[:2]
        bridge = CvBridge()
        # Constant 3 m registered depth is injected. It is not measured photo depth.
        raw_depth = np.full((height, width), 3000, dtype=np.uint16)
        expected = PersonDetector(os.environ['UAV_PERSON_MODEL']).detect(image)
        assert expected, 'Fixture image must contain detectable people'

        def send(epoch=1, valid=True, camera='fixture_optical', images=True):
            stamp = peer.get_clock().now().to_msg()
            context = MapSnapshot(map_id=f'fixture-map-{epoch}', epoch=epoch, valid=valid)
            context.header.frame_id, context.header.stamp = 'map', stamp
            context_pub.publish(context)
            transform.header.stamp = stamp
            broadcaster.sendTransform(transform)
            if images:
                rgb = bridge.cv2_to_imgmsg(image, encoding='bgr8')
                depth = bridge.cv2_to_imgmsg(raw_depth, encoding='16UC1')
                for message in (rgb, depth):
                    message.header.frame_id, message.header.stamp = camera, stamp
                info = CameraInfo(header=rgb.header, width=width, height=height)
                info.k = [500., 0., width/2, 0., 500., height/2, 0., 0., 1.]
                info_pub.publish(info)
                depth_pub.publish(depth)
                rgb_pub.publish(rgb)

        def until(predicate, sender, timeout=12.):
            deadline = time.monotonic()+timeout
            next_send = 0.
            while time.monotonic() < deadline:
                if time.monotonic() >= next_send:
                    sender()
                    next_send = time.monotonic()+.15
                executor.spin_once(timeout_sec=.02)
                if predicate():
                    return
            pytest.fail(f'Pipeline timed out: {mapper.reason}')

        until(lambda: results and len(results[-1].observations) == len(expected)
              and markers and any(m.action == m.ADD for m in markers[-1].markers), send)
        result = results[-1]
        assert result.valid and result.header.frame_id == 'map'
        assert result.map_id == 'fixture-map-1' and result.point_semantics == 'visible_torso_surface'
        assert detections[-1].header.frame_id == 'fixture_optical'
        for item, (box, score) in zip(result.observations, expected):
            # Independent analytical projection for a uniform-depth torso patch.
            u, v = (box[0]+box[2])/2, box[1]+.4*(box[3]-box[1])
            expected_map = [1.-(v-height/2)*3/500, 2.+(u-width/2)*3/500, 6.]
            assert np.allclose([item.position.x, item.position.y, item.position.z], expected_map, atol=.02)
            assert item.detection_confidence == pytest.approx(score)
        assert any(marker.action == marker.ADD for marker in markers[-1].markers)
        response = mapper.save_snapshot(Trigger.Request(), Trigger.Response())
        assert response.success
        assert json.loads((tmp_path/'people.json').read_text())['kind'] == 'historical_person_observations'
        assert not mapper.save_snapshot(Trigger.Request(), Trigger.Response()).success

        # Stop images but keep the same coordinate context. Live people expire;
        # confirmed historical places must remain, without pretending to be live.
        until(lambda: not results[-1].observations and history_markers and
              sum(m.action == m.ADD for m in history_markers[-1].markers) == 2*len(expected),
              lambda: send(images=False))
        assert all(m.lifetime.sec == m.lifetime.nanosec == 0
                   for m in history_markers[-1].markers if m.action == m.ADD)
        mapper.inference_timer.cancel()
        late_history = []
        late_subscription = peer.create_subscription(MarkerArray, '/uav/people/history_markers', late_history.append,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        until(lambda: bool(late_history), lambda: send(images=False))
        assert sum(m.action == m.ADD for m in late_history[-1].markers) == 2*len(expected)
        peer.destroy_subscription(late_subscription)
        mapper.inference_timer.reset()

        # A new context immediately removes the old people and blocks old images.
        until(lambda: results[-1].map_epoch == 2 and not results[-1].observations
              and all(m.action == m.DELETEALL for m in markers[-1].markers),
              lambda: send(epoch=2, images=False))
        assert all(marker.action == marker.DELETEALL for marker in markers[-1].markers)
        until(lambda: history_markers and all(m.action == m.DELETEALL for m in history_markers[-1].markers),
              lambda: send(epoch=2, images=False))

        # A new optical frame without TF still yields 2D, never a map position.
        count = len(detections)
        until(lambda: len(detections) > count and mapper.reason == 'WAITING_HISTORICAL_TF',
              lambda: send(epoch=2, camera='unbound_optical'))
        assert not results[-1].observations
        if os.environ.get('UAV_PERSON_EVIDENCE'):
            folder = Path(os.environ['UAV_PERSON_EVIDENCE'])
            folder.mkdir(parents=True, exist_ok=True)
            snapshot = json.loads((tmp_path/'people.json').read_text())
            (folder/'snapshot.json').write_text(json.dumps(snapshot, indent=2)+'\n')
            evidence = dict(passed=True, scope='actual RGB inference with synthetic registered depth and TF',
                people=len(expected), map_frame=result.header.frame_id,
                synthetic_depth_m=3., synthetic_translation_m=[1., 2., 3.], synthetic_yaw_deg=90,
                analytical_projection_tolerance_m=.02, map_change_clears_tracks=True,
                missing_tf_allows_only_2d=True, snapshot_overwrite_rejected=True,
                history_survives_live_ttl=True, history_transient_local_late_join=True,
                map_change_clears_history=True,
                observations=snapshot['observations'])
            (folder/'ros-result.json').write_text(json.dumps(evidence, indent=2)+'\n')
    finally:
        for node in (mapper, peer):
            if node is not None:
                executor.remove_node(node)
                node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
