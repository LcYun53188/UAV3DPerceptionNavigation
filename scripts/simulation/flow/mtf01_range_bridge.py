#!/usr/bin/env python3
"""Single-ray Gazebo range -> PX4, fixed down-facing body installation only."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[2])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import DistanceSensor


def valid_scan(stamp_us, minimum, maximum, ranges):
    return (stamp_us>0 and len(ranges)==1 and math.isfinite(minimum) and math.isfinite(maximum)
            and abs(minimum-.02)<1e-6 and abs(maximum-8.)<1e-6
            and math.isfinite(ranges[0]) and minimum<=ranges[0]<=maximum)


class RangeBridge(Node):
    def __init__(self):
        super().__init__('mtf01_range_bridge')
        self.last_stamp = 0
        self.publisher = self.create_publisher(DistanceSensor, '/px4_7/fmu/in/distance_sensor', qos_profile_sensor_data)
        self.subscription = self.create_subscription(LaserScan, '/mtf01/range/scan', self.receive, qos_profile_sensor_data)

    def receive(self, scan):
        stamp = scan.header.stamp.sec*1000000 + scan.header.stamp.nanosec//1000
        # Never refresh or clamp invalid / repeated / out-of-range samples.
        if stamp<=self.last_stamp or not valid_scan(stamp,scan.range_min,scan.range_max,scan.ranges):
            return
        self.last_stamp = stamp
        msg = DistanceSensor()
        msg.timestamp = stamp
        msg.device_id = 10093068  # SIMULATION bus 1, address 2, distance SIM type
        msg.min_distance = float(scan.range_min)
        msg.max_distance = float(scan.range_max)
        msg.current_distance = float(scan.ranges[0])
        msg.variance = 0.0  # Ideal single-ray reference; hardware noise not modeled
        msg.signal_quality = -1
        msg.type = DistanceSensor.MAV_DISTANCE_SENSOR_LASER
        msg.orientation = DistanceSensor.ROTATION_DOWNWARD_FACING
        msg.mode = DistanceSensor.MODE_ENABLED
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = RangeBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':
    main()
