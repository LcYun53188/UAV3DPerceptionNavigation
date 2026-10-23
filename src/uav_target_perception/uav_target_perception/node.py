"""Person observations and RViz markers; never publishes navigation or FMU commands."""
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
import time

import message_filters
import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from uav_nav_interfaces.msg import (MapSnapshot, PlanningContext, PersonObservation,
                                    PersonObservationArray)

from .core import ContextGate, PersonTracker, SurfacePoint, fresh, locate_surface, transform_point
from .detector import MODEL_SHA256, PersonDetector


def stamp_seconds(stamp):
    return stamp.sec+stamp.nanosec/1e9


def check_registration(rgb, depth, info, slop):
    """Strict first-version contract: one rectified, registered pixel grid."""
    if info is None:
        raise ValueError('WAITING_CAMERA_INFO')
    if not rgb.header.frame_id or not (rgb.header.frame_id == depth.header.frame_id == info.header.frame_id):
        raise ValueError('RGB_DEPTH_FRAME_MISMATCH')
    if not (rgb.width == depth.width == info.width and rgb.height == depth.height == info.height):
        raise ValueError('RGB_DEPTH_SIZE_MISMATCH')
    if abs(stamp_seconds(rgb.header.stamp)-stamp_seconds(depth.header.stamp)) > slop:
        raise ValueError('RGB_DEPTH_TIME_MISMATCH')
    if stamp_seconds(info.header.stamp) > stamp_seconds(rgb.header.stamp)+slop:
        raise ValueError('FUTURE_CAMERA_INFO')
    if rgb.encoding not in ('rgb8', 'bgr8', 'rgba8', 'bgra8'):
        raise ValueError('UNSUPPORTED_RGB_ENCODING')
    if depth.encoding not in ('32FC1', '16UC1'):
        raise ValueError('UNSUPPORTED_DEPTH_ENCODING')
    if (not np.isfinite(info.k).all() or min(info.k[0], info.k[4]) <= 0
            or not np.isfinite(info.d).all() or any(abs(x) > 1e-9 for x in info.d)
            or info.binning_x > 1 or info.binning_y > 1
            or any((info.roi.x_offset, info.roi.y_offset, info.roi.width, info.roi.height))):
        raise ValueError('REQUIRE_RECTIFIED_FULL_FRAME_INTRINSICS')
    if (any(abs(info.k[index]) > 1e-9 for index in (1, 3, 6, 7))
            or abs(info.k[8]-1.) > 1e-9):
        raise ValueError('REQUIRE_PINHOLE_INTRINSICS')
    expected_projection = [info.k[0], 0., info.k[2], 0., 0., info.k[4], info.k[5], 0., 0., 0., 1., 0.]
    if (not np.isfinite(info.p).all() or not np.isfinite(info.r).all()
            or (any(info.p) and not np.allclose(info.p, expected_projection, atol=1e-9, rtol=0))
            or (any(info.r) and not np.allclose(info.r, np.eye(3).ravel(), atol=1e-9, rtol=0))):
        raise ValueError('RECTIFICATION_PROJECTION_MISMATCH')
    return (info.k[0], info.k[4], info.k[2], info.k[5])


class PersonMapper(Node):
    def __init__(self, detector=None):
        super().__init__('person_mapper')
        defaults = dict(model_path='', rgb_topic='/rgbd_camera/image',
                        depth_topic='/rgbd_camera/depth_image',
                        camera_info_topic='/rgbd_camera/camera_info',
                        context_source='map_snapshot', context_topic='/uav/map/snapshot',
                        inference_hz=5., score_threshold=.5, cpu_threads=2,
                        sync_slop=.03, max_image_age=.5, context_timeout=2.,
                        near_m=.3, far_m=8., depth_spread_m=.35,
                        association_radius_m=.8, track_ttl=1.5, confirmation_frames=3,
                        snapshot_path='', publish_image=True)
        self.options = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        o = self.options
        positive = ('inference_hz', 'sync_slop', 'max_image_age', 'context_timeout',
                    'near_m', 'far_m', 'depth_spread_m', 'association_radius_m', 'track_ttl')
        if not all(math.isfinite(o[key]) and o[key] > 0 for key in positive):
            raise ValueError('Timing, depth and association parameters must be finite and positive')
        if o['near_m'] >= o['far_m'] or o['sync_slop'] > o['max_image_age'] or o['inference_hz'] > 30:
            raise ValueError('Invalid range, synchronization or inference rate')
        if o['context_source'] not in ('map_snapshot', 'planning_context'):
            raise ValueError('context_source must be map_snapshot or planning_context')
        self.detector = detector if detector is not None else PersonDetector(
            o['model_path'], o['score_threshold'], o['cpu_threads'])
        self.bridge = CvBridge()
        self.gate = ContextGate(o['context_timeout'])
        self.tracker = PersonTracker(o['association_radius_m'], o['track_ttl'], o['confirmation_frames'])
        self.pending = self.info = self.info_key = self.map_context = None
        self.last_processed = None
        self.reason = 'WAITING_CONTEXT'
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.detections_pub = self.create_publisher(Detection2DArray, '/uav/people/detections', 1)
        self.observations_pub = self.create_publisher(PersonObservationArray, '/uav/people/observations', 1)
        self.markers_pub = self.create_publisher(MarkerArray, '/uav/people/markers', 1)
        self.image_pub = self.create_publisher(Image, '/uav/people/image', qos_profile_sensor_data)
        self.diagnostics_pub = self.create_publisher(String, '/uav/people/diagnostics', 1)
        self.create_subscription(CameraInfo, o['camera_info_topic'], self.info_cb, qos_profile_sensor_data)
        kind = MapSnapshot if o['context_source'] == 'map_snapshot' else PlanningContext
        self.create_subscription(kind, o['context_topic'], self.context_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.rgb_sub = message_filters.Subscriber(self, Image, o['rgb_topic'], qos_profile=qos_profile_sensor_data)
        self.depth_sub = message_filters.Subscriber(self, Image, o['depth_topic'], qos_profile=qos_profile_sensor_data)
        self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], 4, o['sync_slop'])
        self.sync.registerCallback(self.images_cb)
        self.create_service(Trigger, '/uav/people/save_snapshot', self.save_snapshot)
        self.create_timer(1/o['inference_hz'], self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().info('Person map observations enabled; temporary visible torso surfaces, no control output')

    def now_s(self):
        return self.get_clock().now().nanoseconds/1e9

    def info_cb(self, message):
        key = (message.header.frame_id, message.width, message.height, tuple(message.k),
               tuple(message.d), tuple(message.p), tuple(message.r), message.binning_x, message.binning_y,
               message.roi.x_offset, message.roi.y_offset, message.roi.width, message.roi.height)
        if key != self.info_key:
            self.pending = None
            self.tracker.clear()
        self.info_key, self.info = key, copy.deepcopy(message)

    def context_cb(self, message):
        if self.options['context_source'] == 'map_snapshot':
            key = (message.map_id, message.epoch, 'algorithm_identity', 'gazebo_world_identity', 0, '', ())
        else:
            key = (message.map_id, message.map_epoch, message.localization_session, message.alignment_id,
                   message.alignment_generation, message.context_id, tuple(message.reset_counters))
        if self.gate.key is not None:
            if key[1] < self.gate.key[1] or (key[1] == self.gate.key[1] and key[0] != self.gate.key[0]):
                return
        stamp = stamp_seconds(message.header.stamp)
        valid = message.valid and bool(message.map_id) and message.header.frame_id == 'map'
        changed = self.gate.update(key, valid, stamp, time.monotonic())
        if self.gate.key != key or self.gate.stamp != stamp:
            return
        self.map_context = message
        if changed:
            self.pending = None
            self.tracker.clear()
            self.last_processed = None
            self.publish_state()

    def images_cb(self, rgb, depth):
        stamp = stamp_seconds(rgb.header.stamp)
        if (self.last_processed is not None and stamp <= self.last_processed) or not fresh(
                stamp, self.now_s(), self.options['max_image_age']):
            return
        if self.pending is not None and stamp <= stamp_seconds(self.pending['rgb'].header.stamp):
            return
        # One latest pair, no inference backlog. Calibration and context travel
        # with the pair so pending data cannot silently change coordinate systems.
        self.pending = dict(rgb=rgb, depth=depth, info=self.info, key=self.gate.key, detections=None)

    def process_pending(self, allow_map=True):
        pair = self.pending
        if pair is None:
            return
        rgb, depth, info = pair['rgb'], pair['depth'], pair['info']
        stamp = stamp_seconds(rgb.header.stamp)
        if not all(fresh(value, self.now_s(), self.options['max_image_age']) for value in
                   (stamp, stamp_seconds(depth.header.stamp))):
            self.pending = None
            raise ValueError('STALE_IMAGE')
        intrinsic = check_registration(rgb, depth, info, self.options['sync_slop'])
        if pair['detections'] is None:
            bgr = self.bridge.imgmsg_to_cv2(rgb, desired_encoding='bgr8')
            pair['detections'] = self.detector.detect(bgr)
            # Check again after inference; slow inference must not revive old images.
            if not all(fresh(value, self.now_s(), self.options['max_image_age']) for value in
                       (stamp, stamp_seconds(depth.header.stamp))):
                self.pending = None
                raise ValueError('INFERENCE_DEADLINE_EXCEEDED')
            self.publish_detections(rgb, bgr, pair['detections'])
        if not allow_map or pair['key'] != self.gate.key or not self.gate.accepts(
                stamp, self.now_s(), time.monotonic(), self.options['max_image_age']):
            self.pending = None
            raise ValueError('INVALID_OR_CHANGED_CONTEXT')
        # A missing historical transform is retried only within the image deadline.
        transform = self.tf.lookup_transform('map', rgb.header.frame_id, Time.from_msg(rgb.header.stamp))
        raw = self.bridge.imgmsg_to_cv2(depth, desired_encoding='passthrough')
        metres = np.asarray(raw, dtype=np.float32)*(0.001 if depth.encoding == '16UC1' else 1.)
        observations = []
        t, q = transform.transform.translation, transform.transform.rotation
        for box, score in pair['detections']:
            surface = locate_surface(metres, box, intrinsic, self.options['near_m'],
                                     self.options['far_m'], max_spread=self.options['depth_spread_m'])
            if surface is not None:
                point = transform_point(surface.position, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
                observations.append((SurfacePoint(point, surface.samples, surface.spread), score))
        self.tracker.update(observations, stamp, time.monotonic())
        self.pending, self.last_processed = None, stamp
        self.reason = 'OK' if observations else 'NO_VALID_PERSON_DEPTH'

    def publish_detections(self, rgb, bgr, detections):
        message = Detection2DArray(header=rgb.header)
        for box, score in detections:
            item = Detection2D(header=rgb.header)
            item.bbox.center.position.x = float((box[0]+box[2])/2)
            item.bbox.center.position.y = float((box[1]+box[3])/2)
            item.bbox.size_x, item.bbox.size_y = float(box[2]-box[0]), float(box[3]-box[1])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id, hypothesis.hypothesis.score = 'person', float(score)
            item.results = [hypothesis]
            message.detections.append(item)
        self.detections_pub.publish(message)
        if self.options['publish_image']:
            import cv2
            for box, score in detections:
                x1, y1, x2, y2 = map(int, box)
                cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 220, 0), 2)
                cv2.putText(bgr, f'person {score:.2f}', (x1, max(16, y1-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 220, 0), 1)
            overlay = self.bridge.cv2_to_imgmsg(bgr, encoding='bgr8')
            overlay.header = rgb.header
            self.image_pub.publish(overlay)

    def tick(self):
        valid_context = self.gate.ready(self.now_s(), time.monotonic())
        if len(self.get_publishers_info_by_topic(self.options['context_topic'])) != 1:
            valid_context = False
        if not valid_context:
            self.tracker.clear()
            self.reason = 'CONTEXT_UNAVAILABLE'
        try:
            # Even without map admission 2D detections are useful; 3D is gated.
            self.process_pending(valid_context)
        except TransformException:
            self.reason = 'WAITING_HISTORICAL_TF'
        except (ValueError, CvBridgeError, cv2.error) as error:
            self.reason = str(error)
            self.pending = None
        self.tracker.expire(self.now_s(), time.monotonic())
        self.publish_state(valid_context)

    def state_message(self, valid):
        message = PersonObservationArray(valid=bool(valid), reason=self.reason,
                                         point_semantics='visible_torso_surface')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'map'
        if self.gate.key:
            (message.map_id, message.map_epoch, message.localization_session, message.alignment_id,
             message.alignment_generation, message.context_id, _) = self.gate.key
        if valid:
            for track in self.tracker.confirmed():
                item = PersonObservation(track_id=f'{message.map_epoch}:{track.id}',
                                         detection_confidence=float(track.confidence),
                                         depth_spread_m=track.surface.spread, depth_samples=track.surface.samples)
                item.position.x, item.position.y, item.position.z = map(float, track.surface.position)
                item.last_observed = Time(seconds=track.stamp).to_msg()
                message.observations.append(item)
        return message

    def publish_state(self, valid=False):
        message = self.state_message(valid)
        self.observations_pub.publish(message)
        # This topic belongs only to person_mapper. Replacing its entire marker
        # set prevents ghosts, including tentative/unmatched tracks and resets.
        markers = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        for index, observation in enumerate(message.observations):
            for label in (False, True):
                marker = Marker(header=message.header, ns='people', id=2*index+int(label),
                                action=Marker.ADD, type=Marker.TEXT_VIEW_FACING if label else Marker.SPHERE)
                marker.pose.position = copy.deepcopy(observation.position)
                marker.pose.position.z += .25 if label else 0.
                marker.pose.orientation.w = 1.
                marker.scale.x = marker.scale.y = marker.scale.z = .18
                marker.color.g, marker.color.a = 1., 1.
                marker.lifetime = Duration(seconds=self.options['track_ttl']).to_msg()
                marker.text = f'person {observation.track_id} {observation.detection_confidence:.2f}' if label else ''
                markers.markers.append(marker)
        self.markers_pub.publish(markers)
        self.diagnostics_pub.publish(String(data=json.dumps(dict(reason=self.reason,
            valid=bool(valid), confirmed=len(message.observations), tracks=len(self.tracker.tracks),
            pending=self.pending is not None))))

    def save_snapshot(self, request, response):
        self.tracker.expire(self.now_s(), time.monotonic())
        if (not self.gate.ready(self.now_s(), time.monotonic())
                or len(self.get_publishers_info_by_topic(self.options['context_topic'])) != 1):
            response.message = 'No valid current map context'
            return response
        if not self.options['snapshot_path']:
            response.message = 'Set snapshot_path to a new JSON destination'
            return response
        message = self.state_message(True)
        data = dict(schema=1, kind='historical_person_observations', frame_id='map',
                    snapshot_time=self.now_s(), model_sha256=MODEL_SHA256,
                    calibration_sha256=hashlib.sha256(json.dumps(self.info_key).encode()).hexdigest(),
                    map_id=message.map_id, map_epoch=message.map_epoch,
                    localization_session=message.localization_session, alignment_id=message.alignment_id,
                    alignment_generation=message.alignment_generation, context_id=message.context_id,
                    point_semantics=message.point_semantics,
                    observations=[dict(track_id=o.track_id, position=[o.position.x, o.position.y, o.position.z],
                                       capture_time=stamp_seconds(o.last_observed),
                                       detection_confidence=o.detection_confidence,
                                       depth_spread_m=o.depth_spread_m, depth_samples=o.depth_samples)
                                  for o in message.observations])
        destination = Path(self.options['snapshot_path']).expanduser()
        temporary = None
        try:
            destination.parent.mkdir(parents=True, exist_ok=True)
            with tempfile.NamedTemporaryFile(mode='w', dir=destination.parent, prefix='.people-', delete=False) as handle:
                temporary = Path(handle.name)
                json.dump(data, handle, indent=2, allow_nan=False)
                handle.write('\n')
            os.link(temporary, destination)
            response.success, response.message = True, str(destination)
        except (OSError, ValueError) as error:
            response.message = str(error)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = PersonMapper()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
