"""Read-only USB driver. No firmware configuration commands are sent."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
import serial

from .protocol import Parser, unit_scales


class UsbImu(Node):
    def __init__(self):
        super().__init__('damiao_imu')
        defaults = dict(port='/dev/ttyACM0', baudrate=921600, device_id=1,
                        frame_id='damiao_imu_link', gyro_unit='unset', accel_unit='unset',
                        crc_mode='include_header', gyro_variance=0.0004,
                        accel_variance=0.04, stale_timeout=0.5)
        for key, value in defaults.items():
            self.declare_parameter(key, value)
        self.cfg = {key: self.get_parameter(key).value for key in defaults}
        self.gyro_scale, self.accel_scale = unit_scales(
            self.cfg['gyro_unit'], self.cfg['accel_unit'])
        for key in ('gyro_variance', 'accel_variance', 'stale_timeout'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
        self.parser = Parser(self.cfg['device_id'], self.cfg['crc_mode'])
        self.port = None
        self.retry_after = 0.0
        self.last_gyro = time.monotonic()
        self.stale_reported = False
        # Reliable publisher is compatible with reliable EKF and best-effort listeners.
        self.gyro_pub = self.create_publisher(Imu, '/damiao/imu/gyro', QoSProfile(depth=20))
        self.accel_pub = self.create_publisher(Imu, '/damiao/imu/accel', QoSProfile(depth=20))
        self.timer = self.create_timer(0.002, self.poll)
        self.get_logger().warning(
            'USB reception timestamps only; gyro/accel are independent samples. '
            'Not a synchronized camera IMU stream. Covariances require calibration.')

    def poll(self):
        now = time.monotonic()
        if self.port is None:
            if now < self.retry_after:
                return
            try:
                self.port = serial.Serial(self.cfg['port'], self.cfg['baudrate'], timeout=0,
                                          exclusive=True)
                self.port.reset_input_buffer()
                self.parser.buffer.clear()
                self.get_logger().info(f"Connected to {self.cfg['port']}")
            except (serial.SerialException, OSError) as error:
                self.get_logger().error(f'USB open failed: {error}; retrying in 2 s')
                self.retry_after = now + 2.0
                return
        try:
            # Bound each callback; never republish cached values as fresh measurements.
            data = self.port.read(min(self.port.in_waiting, 4096))
            stamp = self.get_clock().now().to_msg()
            for rid, values in self.parser.feed(data):
                if rid not in (1, 2):
                    continue
                msg = Imu()
                msg.header.stamp = stamp
                msg.header.frame_id = self.cfg['frame_id']
                msg.orientation_covariance[0] = -1.0
                msg.angular_velocity_covariance[0] = -1.0
                msg.linear_acceleration_covariance[0] = -1.0
                if rid == 2:
                    target, covariance = msg.angular_velocity, msg.angular_velocity_covariance
                    scale, variance = self.gyro_scale, self.cfg['gyro_variance']
                    publisher = self.gyro_pub
                    self.last_gyro = now
                    self.stale_reported = False
                else:
                    target, covariance = msg.linear_acceleration, msg.linear_acceleration_covariance
                    scale, variance = self.accel_scale, self.cfg['accel_variance']
                    publisher = self.accel_pub
                target.x, target.y, target.z = (v * scale for v in values)
                covariance[0] = covariance[4] = covariance[8] = variance
                publisher.publish(msg)
        except (serial.SerialException, OSError) as error:
            self.get_logger().error(f'USB disconnected: {error}')
            self.port.close()
            self.port = None
            self.parser.buffer.clear()
            self.retry_after = now + 2.0
        if now - self.last_gyro > self.cfg['stale_timeout'] and not self.stale_reported:
            self.get_logger().warning('No fresh gyro frames; IMU updates stopped')
            self.stale_reported = True

    def destroy_node(self):
        if self.port is not None:
            self.port.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = UsbImu()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
