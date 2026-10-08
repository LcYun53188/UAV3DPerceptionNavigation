"""cuVSLAM source audit; PX4 output is opt-in and owned-SITL-only."""
import os
import re
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from uav_nav_interfaces.msg import VioStatus
from .vio_input import convert_vio, SourceContinuity, stamp_s, validate_source_contract


class VioInput(Node):
    def __init__(self):
        super().__init__('vio_input')
        self.declare_parameter('input_topic', '/uav/vio/odometry')
        self.declare_parameter('source_contract', 'unverified')
        self.declare_parameter('tracking_topic', '/visual_slam/status')
        self.declare_parameter('calibration_id', '')
        self.declare_parameter('emit_px4', False)
        self.calibration = str(self.get_parameter('calibration_id').value)
        if not re.fullmatch('[0-9a-f]{64}', self.calibration):
            raise ValueError('calibration_id must identify the reviewed Pro W calibration/config SHA256')
        self.emit = bool(self.get_parameter('emit_px4').value)
        if self.emit and (not os.environ.get('UAV_SITL_AUTHORIZATION')
                         or os.environ.get('ROS_DOMAIN_ID') != '78'
                         or not os.environ.get('GZ_PARTITION', '').startswith('uav_px4_s0_')
                         or not self.get_parameter('use_sim_time').value):
            raise RuntimeError('PX4 VIO output requires owned SITL and shared simulation clock')
        # Import lazily so isolated transport tests do not require the camera stack.
        from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
        self.session = str(uuid.uuid4())
        self.continuity = SourceContinuity()
        self.tracking = None
        self.tracking_receive = None
        self.last_sample = None
        self.last_receive = None
        self.reason = 'VIO_MISSING'
        self.input_topic = str(self.get_parameter('input_topic').value)
        self.source_contract = str(self.get_parameter('source_contract').value)
        self.tracking_topic = str(self.get_parameter('tracking_topic').value)
        self.status_pub = self.create_publisher(VioStatus, '/uav/vio/status', 10)
        self.output = self.create_publisher(VehicleOdometry, '/px4_7/fmu/in/vehicle_visual_odometry', 10) if self.emit else None
        self.create_subscription(VisualSlamStatus, self.tracking_topic, self.on_tracking, 10)
        self.create_subscription(Odometry, self.input_topic, self.on_odometry, qos_profile_sensor_data)
        self.create_timer(.05, self.publish_status)

    def on_tracking(self, message):
        self.tracking = message
        self.tracking_receive = time.monotonic()

    def validate_tracking(self, sample=None):
        now = self.get_clock().now().nanoseconds / 1e9
        if self.output is not None and len(self.get_publishers_info_by_topic('/px4_7/fmu/in/vehicle_visual_odometry')) != 1:
            raise ValueError('VIO_OUTPUT_WRITER_COUNT')
        if (len(self.get_publishers_info_by_topic(self.input_topic)) != 1
                or len(self.get_publishers_info_by_topic(self.tracking_topic)) != 1):
            raise ValueError('VIO_WRITER_COUNT')
        if (self.tracking is None or self.tracking.vo_state != 1
                or time.monotonic()-self.tracking_receive > .2
                or not -.05 <= now-stamp_s(self.tracking.header.stamp) <= .2
                or (sample is not None and abs(sample-stamp_s(self.tracking.header.stamp)) > .05)):
            raise ValueError('VIO_TRACKING_INVALID')

    def on_odometry(self, message, info):
        try:
            validate_source_contract(self.input_topic, self.source_contract)
            converted = convert_vio(message, self.get_clock().now().nanoseconds / 1e9)
            # Check continuity even if tracking is not yet discovered.
            gid = info.get('publisher_gid') if isinstance(info, dict) else getattr(info, 'publisher_gid', None)
            if gid is None:
                endpoints = self.get_publishers_info_by_topic(self.input_topic)
                if len(endpoints) != 1:
                    raise ValueError('VIO_WRITER_COUNT')
                gid = endpoints[0].endpoint_gid
            self.continuity.accept(message, bytes(gid).hex())
            self.validate_tracking(stamp_s(message.header.stamp))
            self.last_sample = message.header.stamp
            self.last_receive = time.monotonic()
            self.reason = ''
            if self.output:
                self.output.publish(converted)
        except ValueError as exc:
            self.reason = str(exc)
        self.publish_status()

    def publish_status(self):
        reason = self.continuity.fault or self.reason
        try:
            self.validate_tracking()
            if (self.last_receive is None or time.monotonic()-self.last_receive > .2
                    or not -.05 <= self.get_clock().now().nanoseconds/1e9-stamp_s(self.last_sample) <= .2):
                reason = reason or 'VIO_SAMPLE_STALE'
        except ValueError as exc:
            reason = reason or str(exc)
        message = VioStatus(localization_session=self.session, calibration_id=self.calibration,
                            valid=not bool(reason), reason=reason)
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'odom'
        if self.last_sample is not None:
            message.sample_stamp = self.last_sample
        self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = VioInput()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
