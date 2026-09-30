"""Single-writer simulation map session: depth gate, ESDF cache and map bundles."""
import copy
from collections import deque
import json
import os
from pathlib import Path
import shutil
import tempfile
import time
import uuid

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.task import Future
from rclpy.time import Time
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from std_srvs.srv import SetBool
from nvblox_msgs.srv import EsdfAndGradients, FilePath
from tf2_ros import Buffer, TransformListener, TransformException
from uav_nav_interfaces.msg import MapSnapshot
from .core import parse_esdf, sha256, validate_bundle


class MapSession(Node):
    def __init__(self):
        super().__init__('uav_map_session')
        for key, val in [('mode', 'mapping'), ('scene_id', ''), ('build_id', ''), ('resolution', 0.1),
                         ('aabb_min', [-5.0, -5.0, 0.0]), ('aabb_size', [10.0, 10.0, 4.0]),
                         ('map_timeout', 2.0)]:
            self.declare_parameter(key, val)
        self.mode = self.get_parameter('mode').value
        if self.mode not in ('mapping', 'localization'):
            raise ValueError('mode must be mapping or localization')
        self.scene_id = self.get_parameter('scene_id').value
        self.build_id = self.get_parameter('build_id').value
        if not self.scene_id:
            raise ValueError('scene_id must bind the map to a specific Gazebo scene')
        self.resolution = self.get_parameter('resolution').value
        self.map_id = str(uuid.uuid4())
        self.epoch = time.time_ns()
        self.version = 0
        self.busy = False
        self.loaded = False
        self.input_enabled = True
        self.query_pending = False
        self.latest = None
        self.executor_state = None
        self.executor_received = 0.0
        self.camera_info = None
        self.last_depth_stamp = None
        self.pending_depth = deque(maxlen=8)
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        self.group = ReentrantCallbackGroup()
        self.esdf = self.create_client(EsdfAndGradients, '/nvblox_node/get_esdf_and_gradient', callback_group=self.group)
        self.save_client = self.create_client(FilePath, '/nvblox_node/save_map', callback_group=self.group)
        self.load_client = self.create_client(FilePath, '/nvblox_node/load_map', callback_group=self.group)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(MapSnapshot, '/uav/map/snapshot', qos)
        self.depth_pub = self.create_publisher(Image, '/uav/mapping/depth', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/uav/mapping/camera_info', qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/rgbd_camera/camera_info', self.info_cb, qos_profile_sensor_data)
        self.create_subscription(Image, '/rgbd_camera/depth_image', self.depth_cb, qos_profile_sensor_data)
        self.create_subscription(String, '/uav/executor/state', self.executor_cb, 10)
        self.create_service(SetBool, '/uav/map/input_enabled', self.set_input_enabled)
        self.create_service(FilePath, '/uav/map/save', self.save, callback_group=self.group)
        self.create_service(FilePath, '/uav/map/load', self.load, callback_group=self.group)
        self.create_timer(0.02, self.flush_depth)
        self.create_timer(0.5, self.query)
        self.invalidate()

    def executor_cb(self, msg):
        self.executor_state = msg.data
        self.executor_received = time.monotonic()

    def set_input_enabled(self, req, response):
        if self.busy or self.mode != 'mapping' or self.executor_state != 'HOLD':
            response.success = False
            response.message = 'Input gate requires mapping mode, HOLD, and no map operation'
            return response
        self.pending_depth.clear()
        self.input_enabled = req.data
        response.success = True
        return response

    def info_cb(self, msg):
        self.camera_info = msg

    def depth_cb(self, msg):
        if self.busy or not self.input_enabled or self.mode != 'mapping' or self.camera_info is None:
            return
        if msg.encoding not in ('32FC1', '16UC1') or msg.header.frame_id != 'oakd_camera_optical_frame':
            return
        info = self.camera_info
        if msg.width != info.width or msg.height != info.height or info.k[0] <= 0 or info.k[4] <= 0:
            return
        stamp = Time.from_msg(msg.header.stamp)
        age = (self.get_clock().now()-stamp).nanoseconds/1e9
        if age < 0 or age > 0.5:
            return
        if self.last_depth_stamp is not None and stamp.nanoseconds <= self.last_depth_stamp:
            return
        if self.pending_depth and stamp.nanoseconds <= Time.from_msg(self.pending_depth[-1][0].header.stamp).nanoseconds:
            return
        aligned = copy.deepcopy(info)
        aligned.header = msg.header
        self.pending_depth.append((msg, aligned))
        self.flush_depth()

    def flush_depth(self):
        # Depth and TF arrive on independent topics. Wait briefly for the exact
        # historical transform instead of dropping every image that wins that race.
        # Never substitute the latest pose or forward frames across an input gate.
        if self.busy or not self.input_enabled or self.mode != 'mapping':
            self.pending_depth.clear()
            return
        while self.pending_depth:
            msg, info = self.pending_depth[0]
            stamp = Time.from_msg(msg.header.stamp)
            age = (self.get_clock().now()-stamp).nanoseconds/1e9
            if not 0 <= age <= 0.5:
                self.pending_depth.popleft()
                continue
            try:
                self.tf.lookup_transform('map', msg.header.frame_id, stamp)
            except TransformException:
                return
            self.pending_depth.popleft()
            self.last_depth_stamp = stamp.nanoseconds
            self.info_pub.publish(info)
            self.depth_pub.publish(msg)

    def invalidate(self):
        msg = MapSnapshot()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.map_id = self.map_id
        msg.epoch = self.epoch
        msg.version = self.version
        msg.valid = False
        self.latest = None
        self.pub.publish(msg)

    def query(self):
        if self.busy or self.query_pending or not self.esdf.service_is_ready() or (self.mode == 'localization' and not self.loaded):
            return
        req = EsdfAndGradients.Request()
        req.frame_id = 'map'
        req.use_aabb = True
        req.update_esdf = True
        for axis, val in zip('xyz', self.get_parameter('aabb_min').value):
            setattr(req.aabb_min_m, axis, val)
        for axis, val in zip('xyz', self.get_parameter('aabb_size').value):
            setattr(req.aabb_size_m, axis, val)
        self.query_pending = True
        epoch = self.epoch
        future = self.esdf.call_async(req)
        def done(f):
            self.query_pending = False
            if self.busy or epoch != self.epoch:
                return
            try:
                res = f.result()
                grid = parse_esdf(res)
                if not np.any(grid.observed):
                    raise ValueError('No observed voxels')
                if abs(grid.resolution-self.resolution) > 1e-6:
                    raise ValueError('Map resolution mismatch')
                age = (self.get_clock().now()-Time.from_msg(res.header.stamp)).nanoseconds/1e9
                if self.mode == 'mapping' and not 0 <= age <= self.get_parameter('map_timeout').value:
                    raise ValueError('Map source data stale')
                msg = MapSnapshot()
                msg.header.frame_id = 'map'
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.source_stamp = res.header.stamp
                msg.map_id = self.map_id
                msg.epoch = self.epoch
                self.version += 1
                msg.version = self.version
                msg.valid = True
                msg.static_map = self.mode == 'localization'
                msg.origin = res.origin_m
                msg.resolution = grid.resolution
                msg.shape = list(grid.distance.shape)
                msg.distance = grid.distance.ravel().tolist()
                msg.observed = grid.observed.astype(np.uint8).ravel().tolist()
                self.latest = msg
                self.pub.publish(msg)
            except Exception as exc:
                self.invalidate()
                self.get_logger().warning(f'ESDF unavailable: {exc}')
        future.add_done_callback(done)

    async def delay(self, seconds):
        future = Future()
        timer = self.create_timer(seconds, lambda: future.set_result(True) if not future.done() else None,
                                  callback_group=self.group)
        try:
            await future
        finally:
            self.destroy_timer(timer)

    def begin_operation(self, map_id=None):
        if self.busy or self.executor_state != 'HOLD' or time.monotonic()-self.executor_received > 1.0:
            raise ValueError('Map operation requires fresh HOLD state and no pending operation')
        self.busy = True
        self.epoch += 1
        self.version = 0
        if map_id is not None:
            self.map_id = map_id
        self.invalidate()

    async def save(self, req, response):
        temp = None
        started = False
        try:
            if (self.mode != 'mapping' or self.latest is None or
                    (self.get_clock().now()-Time.from_msg(self.latest.header.stamp)).nanoseconds/1e9 > self.get_parameter('map_timeout').value):
                raise ValueError('Save requires a valid live map')
            dest = Path(req.file_path).expanduser().resolve()
            if dest.exists():
                raise ValueError('Destination exists; choose a new map bundle directory')
            if not self.save_client.service_is_ready():
                raise ValueError('nvblox save service unavailable')
            self.begin_operation()
            started = True
            # Gate depth before draining. Nvblox serializes integration and save
            # tasks on its processing queue; quiet depth prevents new writes.
            await self.delay(1.0)
            dest.parent.mkdir(parents=True, exist_ok=True)
            temp = Path(tempfile.mkdtemp(prefix='.map-', dir=dest.parent))
            filename = temp/'static_map.nvblx'
            result = await self.save_client.call_async(FilePath.Request(file_path=str(filename)))
            if not result.success or not filename.is_file() or filename.stat().st_size == 0:
                raise ValueError('nvblox save failed')
            manifest = dict(schema=1, map_id=self.map_id, scene_id=self.scene_id, build_id=self.build_id,
                            frame_id='map', localization='gazebo_world_identity',
                            resolution=self.resolution, sha256=sha256(filename),
                            created_unix=time.time(), nvblox_commit='6362295e581ef243773c8a348ac46711e4a1fca4',
                            ego_commit='23a8d5a191711dd65633df689bd00f55d4dea8f9',
                            aabb_min=self.get_parameter('aabb_min').value,
                            aabb_size=self.get_parameter('aabb_size').value)
            (temp/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
            os.rename(temp, dest)
            temp = None
            response.success = True
            self.get_logger().info(f'Saved map bundle: {dest}')
        except Exception as exc:
            self.get_logger().error(f'Save rejected: {exc}')
            response.success = False
        finally:
            if temp is not None:
                shutil.rmtree(temp)
            if started:
                self.busy = False
        return response

    async def load(self, req, response):
        started = False
        try:
            # Separate mode ensures no old live observations can race into loaded map.
            if self.mode != 'localization' or not self.load_client.service_is_ready():
                raise ValueError('Load requires localization mode and ready nvblox')
            manifest = validate_bundle(req.file_path, self.scene_id, self.resolution, self.build_id)
            # The invalidation and subsequent snapshots must identify the same
            # session. Consumers reject map-ID changes within one epoch.
            self.begin_operation(map_id=manifest['map_id'])
            started = True
            self.loaded = False
            await self.delay(1.0)
            filename = Path(req.file_path).expanduser().resolve()/'static_map.nvblx'
            result = await self.load_client.call_async(FilePath.Request(file_path=str(filename)))
            if not result.success:
                raise ValueError('nvblox load failed')
            self.loaded = True
            self.version = 0
            response.success = True
            self.get_logger().info('Map loaded; waiting for new ESDF snapshot before planning')
        except Exception as exc:
            self.get_logger().error(f'Load rejected: {exc}')
            response.success = False
        finally:
            if started:
                self.busy = False
        return response


def main():
    rclpy.init()
    node = MapSession()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
