"""Managed source binding for an explicitly defined map <- odom frame."""
import math
import time
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Transform
from uav_nav_interfaces.msg import MapSnapshot, LocalizedOdometry, LocalizationAlignment
from .planning_sources import SourceBinding


class PlanningSources(Node):
    def __init__(self):
        super().__init__('planning_source_binding')
        session = self.declare_parameter('localization_session', '').value
        alignment_id = self.declare_parameter('alignment_id', '').value
        translation = self.declare_parameter('map_translation', [0., 0., 0.]).value
        yaw = self.declare_parameter('map_yaw_rad', 0.).value
        if len(translation) != 3 or not all(math.isfinite(v) for v in [*translation, yaw]):
            raise ValueError('INVALID_EXPLICIT_TRANSFORM')
        transform = Transform()
        transform.translation.x, transform.translation.y, transform.translation.z = translation
        transform.rotation.z = math.sin(yaw/2); transform.rotation.w = math.cos(yaw/2)
        self.binding = SourceBinding(session, alignment_id, transform)
        self.gids = {}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_pub = self.create_publisher(MapSnapshot, '/planning/source/map', qos)
        self.alignment_pub = self.create_publisher(LocalizationAlignment, '/planning/source/alignment', 10)
        self.topics = {'map': '/uav/map/snapshot', 'odom': '/uav/px4/localized_odometry'}
        self.create_subscription(MapSnapshot, self.topics['map'], lambda m:self.receive('map', m), qos)
        self.create_subscription(LocalizedOdometry, self.topics['odom'], lambda m:self.receive('odom', m), qos_profile_sensor_data)
        self.create_timer(.1, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def unique(self):
        for name, topic in self.topics.items():
            endpoints = self.get_publishers_info_by_topic(topic)
            if len(endpoints) != 1:
                if self.binding.identity:self.binding.retire('SOURCE_WRITER_LOST_OR_DUPLICATED')
                return False
            gid = bytes(endpoints[0].endpoint_gid)
            if name in self.gids and self.gids[name] != gid:
                self.binding.retire('SOURCE_WRITER_REPLACED'); return False
            self.gids[name] = gid
        return True

    def receive(self, name, message):
        if not self.unique():return
        accepted = self.binding.update(name, message, time.monotonic())
        if name == 'map' and accepted:
            # Forward original source identity, bytes and timestamps unchanged.
            self.map_pub.publish(message)
        if self.binding.retired:self.tick()

    def tick(self):
        unique = self.unique()
        now = self.get_clock().now()
        message = self.binding.alignment(now.nanoseconds/1e9, time.monotonic(), now.to_msg())
        if not unique:message.valid = False
        self.alignment_pub.publish(message)


def main(args=None):
    rclpy.init(args=args); node = PlanningSources()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
