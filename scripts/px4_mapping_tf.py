#!/usr/bin/env python3
"""Disarmed-only mapping TF observer. No PX4 input publishers or truth poses."""
import json
import math
from pathlib import Path
import sys
import time
import uuid
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from px4_msgs.msg import VehicleOdometry, VehicleStatus, VehicleLocalPosition
from px4_comm_bridge.converters import vehicle_odometry_to_ros
from uav_nav_interfaces.msg import MapSnapshot, LocalizedOdometry
from rclpy.qos import QoSProfile, DurabilityPolicy


def output_topic(namespace, name, kind):
    version = getattr(kind, "MESSAGE_VERSION", 0)
    return namespace + "/fmu/out/" + name + (f"_v{version}" if version else "")


class MappingTf(Node):
    def __init__(self, frames, evidence, session=None):
        super().__init__('px4_disarmed_mapping_tf')
        self.set_parameters([Parameter('use_sim_time', value=True)])
        self.evidence = evidence
        self.allowed = False
        self.status_received = 0.
        self.reset_stable_since = None
        self.started = False
        self.reason = "SETTLING"
        self.reset = None
        self.local_counters = None
        self.local_received = 0.
        self.local_baseline = None
        self.localization_session = session or str(uuid.uuid4())
        self.localized_pub = self.create_publisher(LocalizedOdometry, '/uav/px4/localized_odometry', qos_profile_sensor_data)
        self.failed = False
        self.dynamic = TransformBroadcaster(self)
        self.static = StaticTransformBroadcaster(self)
        transforms = []
        identity = TransformStamped()
        identity.header.frame_id = 'map'
        identity.child_frame_id = 'odom'
        identity.transform.rotation.w = 1.
        transforms.append(identity)
        for name, frame in frames.items():
            t = TransformStamped()
            t.header.frame_id = 'base_link'
            t.child_frame_id = name
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = frame['position']
            r, p, y = (v/2 for v in frame['rpy'])
            q = t.transform.rotation
            q.x = math.sin(r)*math.cos(p)*math.cos(y)-math.cos(r)*math.sin(p)*math.sin(y)
            q.y = math.cos(r)*math.sin(p)*math.cos(y)+math.sin(r)*math.cos(p)*math.sin(y)
            q.z = math.cos(r)*math.cos(p)*math.sin(y)-math.sin(r)*math.sin(p)*math.cos(y)
            q.w = math.cos(r)*math.cos(p)*math.cos(y)+math.sin(r)*math.sin(p)*math.sin(y)
            transforms.append(t)
        self.static.sendTransform(transforms)
        self.create_subscription(VehicleStatus, output_topic('/px4_7','vehicle_status',VehicleStatus), self.status, qos_profile_sensor_data)
        self.create_subscription(VehicleOdometry, output_topic('/px4_7','vehicle_odometry',VehicleOdometry), self.pose, qos_profile_sensor_data)

        self.create_subscription(VehicleLocalPosition, output_topic('/px4_7','vehicle_local_position',VehicleLocalPosition), self.local, qos_profile_sensor_data)
        self.create_subscription(MapSnapshot, '/uav/map/snapshot', self.snapshot,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def snapshot(self, message):
        mask = np.asarray(message.observed, dtype=bool)
        distances = np.asarray(message.distance)
        observed = int(np.count_nonzero(mask))
        age = self.get_clock().now().nanoseconds/1e9 - (message.source_stamp.sec + message.source_stamp.nanosec/1e9)
        passed = bool(self.allowed and not self.failed and message.valid and self.started and time.monotonic() - self.status_received <= 1. and observed > 0 and 0 <= age <= 2.)
        data = dict(passed=passed, map_id=message.map_id, epoch=message.epoch,
                    version=message.version, valid=message.valid, observed_voxels=observed,
                    observed_positive_distance_voxels=int(np.count_nonzero(mask & (distances > 0))),
                    shape=[int(v) for v in message.shape], resolution=message.resolution,
                    odometry_reset_counter=self.reset,
                    voxels=len(message.observed), source_age_s=age,
                    captured_monotonic=time.monotonic(), disarmed=self.allowed,
                    reason=self.reason,
                    scope='actual depth to nvblox ESDF using PX4 EKF TF; no planning or flight qualification')
        temporary = self.evidence.with_suffix('.tmp')
        temporary.write_text(json.dumps(data, indent=2)+'\n')
        temporary.replace(self.evidence)

    def status(self, message):
        self.status_received = time.monotonic()
        if message.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            self.failed = True
            self.reason = 'ARMED'
        self.allowed = message.arming_state == 1 and not self.failed

    def local(self, message):
        age = self.get_clock().now().nanoseconds/1e9 - message.timestamp/1e6
        if not -.05 <= age <= .5 or not message.xy_valid or not message.z_valid:return
        counters = (message.xy_reset_counter, message.z_reset_counter,
                    message.vxy_reset_counter, message.vz_reset_counter, message.heading_reset_counter)
        if self.started and counters != self.local_baseline:
            self.failed = True; self.reason = 'LOCAL_POSITION_RESET'; return
        self.local_counters = counters
        self.local_received = time.monotonic()

    def pose(self, message):
        if self.failed or not self.allowed or time.monotonic() - self.status_received > 1.:
            return
        if self.local_counters is None or time.monotonic() - self.local_received > .5:return
        if self.reset is None or message.reset_counter != self.reset or self.local_counters != self.local_baseline:
            if self.started:
                self.failed = True
                self.reason = 'ODOMETRY_RESET'
                return
            self.local_baseline = self.local_counters
            self.reset = message.reset_counter
            self.reset_stable_since = time.monotonic()
        if time.monotonic() - self.reset_stable_since < 5.:
            return
        try:
            odom = vehicle_odometry_to_ros(message)
        except ValueError:
            self.failed = True
            return
        age = self.get_clock().now().nanoseconds/1e9 - (odom.header.stamp.sec + odom.header.stamp.nanosec/1e9)
        if not -.05 <= age <= .5:
            return
        t = TransformStamped()
        t.header = odom.header
        t.child_frame_id = odom.child_frame_id
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.started = True
        self.reason = 'READY'
        self.dynamic.sendTransform(t)
        self.localized_pub.publish(LocalizedOdometry(header=odom.header, odometry=odom,
            localization_session=self.localization_session, reset_counters=list(self.local_baseline) + [self.reset]))


if __name__ == '__main__':
    frames = json.loads(Path(sys.argv[1]).read_text())
    rclpy.init(args=[])
    node = MappingTf(frames, Path(sys.argv[2]), sys.argv[3] if len(sys.argv)>3 else None)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
