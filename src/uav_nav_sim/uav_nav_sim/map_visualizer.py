"""Derived RViz colors from admitted map data; no control publishers or services."""
import copy
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray
from nvblox_msgs.msg import Mesh, VoxelBlockLayer
from uav_nav_interfaces.msg import MapSnapshot, PersonObservationArray

from .map_visualization_core import ramp, snapshot_slice, stamp_seconds, surface_colors


class MapVisualizer(Node):
    def __init__(self):
        super().__init__('uav_map_visualizer')
        defaults = dict(mode='esdf', distance_max=3., slice_height=1., clearance=.8,
                        caution_margin=.3, age_max=30., semantic_radius=.4,
                        map_timeout=3., person_timeout=2.)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.options = {key: self.get_parameter(key).value for key in defaults}
        self.mode = self.options['mode']
        if self.mode not in ('esdf', 'semantic', 'weight', 'age', 'traversability'):
            raise ValueError('Unsupported derived map color mode')
        for key, value in self.options.items():
            if key != 'mode' and (not math.isfinite(value) or (key != 'slice_height' and value <= 0)):
                raise ValueError('Invalid visualization parameter: ' + key)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.snapshot = None
        self.received = 0.
        self.key = None
        self.source_boundary = 0.
        self.blocks = {}
        self.voxels = {}
        self.block_size = None
        self.voxel_size = None
        self.layer_type = 0
        self.people = None
        self.people_received = 0.
        self.last_clock = None
        self.reason = 'WAITING_FOR_ADMITTED_MAP'
        self.mesh_pub = self.create_publisher(Mesh, '/uav/map/colored_mesh', 1)
        self.voxel_pub = self.create_publisher(VoxelBlockLayer, '/uav/map/colored_voxels', 1)
        self.slice_pub = self.create_publisher(MarkerArray, '/uav/map/color_slice', 1)
        self.legend_pub = self.create_publisher(MarkerArray, '/uav/map/color_legend', 1)
        self.status_pub = self.create_publisher(String, '/uav/map/color_status', 1)
        self.create_subscription(MapSnapshot, '/uav/map/snapshot', self.snapshot_cb, qos)
        if self.mode in ('age', 'semantic'):
            self.create_subscription(Mesh, '/nvblox_node/mesh', self.mesh_cb, 1)
        if self.mode == 'semantic':
            self.create_subscription(PersonObservationArray, '/uav/people/observations', self.people_cb, 1)
        if self.mode == 'weight':
            self.create_subscription(VoxelBlockLayer, '/nvblox_node/tsdf_layer', self.weight_cb, 1)
        self.create_timer(2., self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds/1e9

    def clear(self, reason):
        self.blocks.clear()
        self.voxels.clear()
        self.block_size = None
        self.voxel_size = None
        self.people = None
        self.reason = reason
        header = self.header()
        self.mesh_pub.publish(Mesh(header=header, clear=True))
        self.voxel_pub.publish(VoxelBlockLayer(header=header, clear=True))
        self.slice_pub.publish(MarkerArray(markers=[Marker(header=header, action=Marker.DELETEALL)]))
        self.status_pub.publish(String(data=json.dumps(dict(mode=self.mode, reason=reason, read_only=True))))
        if reason.startswith('INVALID_') or reason == 'DISPLAY_CACHE_LIMIT':
            self.get_logger().warning(reason)

    def header(self):
        from std_msgs.msg import Header
        return Header(frame_id='map', stamp=self.get_clock().now().to_msg())

    def snapshot_cb(self, msg):
        key = (msg.map_id, msg.epoch)
        if msg.valid and self.snapshot is not None and key == self.key and msg.version < self.snapshot.version:
            return
        if key != self.key or not msg.valid or msg.header.frame_id != 'map' or not msg.map_id:
            self.clear('MAP_INVALID_OR_CHANGED')
        if key != self.key and msg.valid:
            self.source_boundary = stamp_seconds(msg.source_stamp)
        self.key = key
        self.snapshot = msg if msg.valid and msg.header.frame_id == 'map' and msg.map_id else None
        self.received = time.monotonic()

    def ready(self):
        msg = self.snapshot
        if msg is None or time.monotonic()-self.received > self.options['map_timeout']:
            return False
        # Stored maps have no fresh sensor source stamp; receive freshness still applies.
        age = self.now() - stamp_seconds(msg.source_stamp)
        return msg.static_map or 0 <= age <= self.options['map_timeout']

    def people_cb(self, msg):
        context = (msg.localization_session, msg.alignment_id, msg.alignment_generation, msg.context_id)
        old = self.people
        if old is not None and context != (old.localization_session, old.alignment_id,
                                            old.alignment_generation, old.context_id):
            self.people = None
        if (self.ready() and msg.valid and msg.header.frame_id == 'map'
                and (msg.map_id, msg.map_epoch) == self.key and msg.point_semantics == 'visible_torso_surface'):
            self.people = msg
            self.people_received = time.monotonic()
        else:
            self.people = None

    def person_points(self):
        msg = self.people
        if msg is None or time.monotonic()-self.people_received > self.options['person_timeout']:
            return []
        now = self.now()
        if not 0 <= now-stamp_seconds(msg.header.stamp) <= self.options['person_timeout']:
            return []
        return [(p.position.x, p.position.y, p.position.z) for p in msg.observations
                if 0 <= now-stamp_seconds(p.last_observed) <= self.options['person_timeout']]

    def mesh_cb(self, msg):
        if not self.ready() or msg.header.frame_id != 'map':
            return
        if (not math.isfinite(msg.block_size_m) or msg.block_size_m <= 0
                or len(msg.blocks) != len(msg.block_indices)):
            self.clear('INVALID_MESH')
            return
        stamp = stamp_seconds(msg.header.stamp)
        if stamp < self.source_boundary or not 0 <= self.now()-stamp <= self.options['map_timeout']:
            return
        if msg.clear or self.block_size != msg.block_size_m:
            self.blocks.clear()
        self.block_size = msg.block_size_m
        for index, block in zip(msg.block_indices, msg.blocks):
            key = (index.x, index.y, index.z)
            if block.vertices:
                if (not all(math.isfinite(v.x) and math.isfinite(v.y) and math.isfinite(v.z) for v in block.vertices)
                        or len(block.triangles) % 3 or any(i < 0 or i >= len(block.vertices) for i in block.triangles)):
                    self.clear('INVALID_MESH')
                    return
                self.blocks[key] = (copy.deepcopy(index), copy.deepcopy(block), stamp)
            else:
                self.blocks.pop(key, None)
        # Fail visibly rather than retaining an unbounded map in a display process.
        if len(self.blocks) > 4096 or sum(len(b.vertices) for _, b, _ in self.blocks.values()) > 1_000_000:
            self.clear('DISPLAY_CACHE_LIMIT')

    def weight_cb(self, msg):
        if not self.ready() or msg.header.frame_id != 'map':
            return
        stamp = stamp_seconds(msg.header.stamp)
        if stamp < self.source_boundary or not 0 <= self.now()-stamp <= self.options['map_timeout']:
            return
        if (len(msg.blocks) != len(msg.block_indices) or not math.isfinite(msg.voxel_size_m)
                or msg.voxel_size_m <= 0 or not math.isfinite(msg.block_size_m) or msg.block_size_m <= 0):
            self.clear('INVALID_WEIGHT_LAYER')
            return
        converted = copy.deepcopy(msg)
        # nvblox tsdfVoxelToRgb publishes actual clamp(weight/5,0,1) in RGB.
        # The configured projective_integrator_max_weight is 5; no proxy count.
        for block in converted.blocks:
            if len(block.colors) != len(block.centers):
                self.clear('INVALID_WEIGHT_LAYER')
                return
            values = [c.r for c in block.colors]
            if (not np.all(np.isfinite(values)) or
                    not all(math.isfinite(p.x) and math.isfinite(p.y) and math.isfinite(p.z) for p in block.centers)):
                self.clear('INVALID_WEIGHT_LAYER')
                return
            block.colors = [ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
                            for r, g, b, a in ramp(values)]
        if converted.clear or self.block_size != msg.block_size_m or self.voxel_size != msg.voxel_size_m:
            self.voxels.clear()
        self.block_size, self.voxel_size = msg.block_size_m, msg.voxel_size_m
        self.layer_type = msg.layer_type
        for index, block in zip(converted.block_indices, converted.blocks):
            key = (index.x, index.y, index.z)
            if block.centers:
                self.voxels[key] = (index, block)
            else:
                self.voxels.pop(key, None)
        if len(self.voxels) > 4096 or sum(len(b.centers) for _, b in self.voxels.values()) > 1_000_000:
            self.clear('DISPLAY_CACHE_LIMIT')

    def publish_weight(self):
        msg = VoxelBlockLayer(header=self.header(), clear=True, block_size_m=self.block_size or .0,
                              voxel_size_m=self.voxel_size or .0, layer_type=self.layer_type)
        for index, block in self.voxels.values():
            msg.block_indices.append(index)
            msg.blocks.append(block)
        self.voxel_pub.publish(msg)

    def publish_mesh(self):
        msg = Mesh(header=self.header(), clear=True, block_size_m=self.block_size or .0)
        people = self.person_points()
        for index, block, stamp in self.blocks.values():
            block = copy.deepcopy(block)
            colors = surface_colors([(v.x, v.y, v.z) for v in block.vertices], self.mode,
                                    age=self.now()-stamp, age_max=self.options['age_max'], people=people,
                                    radius=self.options['semantic_radius'])
            block.colors = [ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))
                            for r, g, b, a in colors]
            msg.block_indices.append(index)
            msg.blocks.append(block)
        self.mesh_pub.publish(msg)

    def publish_slice(self):
        points, colors = snapshot_slice(self.snapshot, self.options['slice_height'], self.mode,
                                       distance_max=self.options['distance_max'], clearance=self.options['clearance'],
                                       caution_margin=self.options['caution_margin'])
        marker = Marker(header=self.header(), ns='map_color_slice', id=0, type=Marker.CUBE_LIST,
                        action=Marker.ADD)
        marker.pose.orientation.w = 1.
        marker.scale.x = marker.scale.y = self.snapshot.resolution
        marker.scale.z = min(self.snapshot.resolution, .05)
        marker.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in points]
        marker.colors = [ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a)) for r, g, b, a in colors]
        self.slice_pub.publish(MarkerArray(markers=[marker]))
        self.reason = 'OK' if len(points) else 'SLICE_OUTSIDE_MAP_OR_UNOBSERVED'

    def tick(self):
        now = self.now()
        if self.last_clock is not None and now < self.last_clock:
            self.snapshot = None
            self.clear('CLOCK_RESET')
        self.last_clock = now
        if not self.ready():
            self.clear('MAP_INVALID_OR_STALE')
        else:
            try:
                if self.mode in ('age', 'semantic'):
                    self.publish_mesh()
                    self.reason = 'OK' if self.blocks else 'WAITING_FOR_MESH'
                elif self.mode in ('esdf', 'traversability'):
                    self.publish_slice()
                else:
                    self.publish_weight()
                    self.reason = 'OK' if self.voxels else 'WAITING_FOR_TSDF_WEIGHTS'
            except (ValueError, OverflowError) as exc:
                self.clear('INVALID_DISPLAY_DATA: ' + str(exc))
        descriptions = {
            'esdf': f'ESDF z={self.options["slice_height"]:g} m: red <=0, blue >= {self.options["distance_max"]:g} m; unknown hidden',
            'traversability': f'Clearance z={self.options["slice_height"]:g} m: red blocked, yellow margin, green clear; radius={self.options["clearance"]:g} m; unknown hidden',
            'weight': 'TSDF fusion weight: blue=0, red>=5; occupied observed voxels only; not probability',
            'age': f'Mesh block publication age: blue=0 s, red>={self.options["age_max"]:g} s; not per-voxel observation time',
            'semantic': f'Magenta=person torso vicinity ({self.options["semantic_radius"]:g} m), gray=unclassified; not segmentation',
        }
        text = descriptions[self.mode] + '\n' + self.reason
        legend = Marker(header=self.header(), ns='map_color_legend', id=0, type=Marker.TEXT_VIEW_FACING,
                        action=Marker.ADD, text=text)
        legend.pose.orientation.w = 1.
        legend.pose.position.z = 4.2
        legend.scale.z = .15
        legend.color = ColorRGBA(r=1., g=1., b=1., a=1.)
        self.legend_pub.publish(MarkerArray(markers=[legend]))
        self.status_pub.publish(String(data=json.dumps(dict(mode=self.mode, reason=self.reason,
                                                            read_only=True, description=descriptions[self.mode]))))


def main(args=None):
    rclpy.init(args=args)
    node = MapVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
