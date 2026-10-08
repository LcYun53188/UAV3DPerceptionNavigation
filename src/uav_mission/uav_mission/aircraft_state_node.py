"""Read-only PX4 1.16 adapter. Run in the isolated px4_msgs overlay."""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from px4_msgs.msg import VehicleStatus, VehicleLandDetected, VehicleLocalPosition
from uav_nav_interfaces.msg import AircraftState, StateDimension

from .aircraft_state import AircraftStateAggregator, Dimension


class AircraftStateNode(Node):
    def __init__(self):
        super().__init__('aircraft_state')
        prefix = self.declare_parameter('px4_namespace', '/px4_7').value.rstrip('/')
        max_age = self.declare_parameter('max_age_s', .5).value
        clock_stall = self.declare_parameter('clock_stall_s', .5).value
        self.aggregator = AircraftStateAggregator(max_age, clock_stall)
        self.publisher = self.create_publisher(AircraftState, 'aircraft_state', 10)
        sources = (
            ('status', 'vehicle_status', VehicleStatus,
             ('arming_state', 'nav_state', 'failsafe', 'failure_detector_status')),
            ('land', 'vehicle_land_detected', VehicleLandDetected,
             ('landed', 'ground_contact', 'maybe_landed', 'freefall', 'vertical_movement',
              'horizontal_movement', 'rotational_movement')),
            ('local', 'vehicle_local_position', VehicleLocalPosition,
             ('xy_valid', 'z_valid', 'x', 'y', 'z', 'dead_reckoning', 'eph', 'epv',
              'xy_reset_counter', 'z_reset_counter', 'vxy_reset_counter', 'vz_reset_counter', 'heading_reset_counter')))
        for source, topic, kind, fields in sources:
            suffix = f'_v{kind.MESSAGE_VERSION}' if kind.MESSAGE_VERSION else ''
            self.create_subscription(kind, f'{prefix}/fmu/out/{topic}{suffix}',
                                     self.callback(source, fields), qos_profile_sensor_data)
        # ROS time timers stop when /clock stops; safety reporting must continue.
        self.create_timer(.05, self.publish_state, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def callback(self, source, fields):
        def receive(msg):
            self.aggregator.observe(source, msg.timestamp / 1e6,
                                    {field: getattr(msg, field) for field in fields},
                                    self.get_clock().now().nanoseconds / 1e9, time.monotonic())
        return receive

    def publish_state(self):
        now = self.get_clock().now()
        state = self.aggregator.snapshot(now.nanoseconds / 1e9, time.monotonic())
        msg = AircraftState()
        msg.header.stamp = now.to_msg()
        for key, value in state.items():
            if isinstance(value, Dimension):
                value = StateDimension(**vars(value))
            setattr(msg, key, value)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AircraftStateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
