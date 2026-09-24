"""Exercise actual pyserial and ROS publication using a pseudo-terminal."""
import os
import time

import pytest
import rclpy
from sensor_msgs.msg import Imu
from damiao_imu.node import UsbImu
from test_protocol import frame


def test_usb_publication_no_replay_and_reconnect(tmp_path):
    master, slave = os.openpty()
    link = tmp_path / 'imu'
    link.symlink_to(os.ttyname(slave))
    rclpy.init(args=['--ros-args', '-p', f'port:={link}',
                     '-p', 'gyro_unit:=deg_s', '-p', 'accel_unit:=g'])
    node = UsbImu()
    listener = rclpy.create_node('dm_test_listener')
    gyro, accel = [], []
    listener.create_subscription(Imu, '/damiao/imu/gyro', gyro.append, 20)
    listener.create_subscription(Imu, '/damiao/imu/accel', accel.append, 20)

    def spin(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.005)
            rclpy.spin_once(listener, timeout_sec=0.005)

    try:
        spin(0.5)
        os.write(master, frame(1, (0., 0., 1.)) + frame(2, (0., 0., 180.)))
        spin(0.2)
        assert len(gyro) == len(accel) == 1
        assert gyro[0].angular_velocity.z == pytest.approx(3.141592653589793)
        assert accel[0].linear_acceleration.z == pytest.approx(9.80665)
        assert gyro[0].orientation_covariance[0] == -1
        assert gyro[0].linear_acceleration_covariance[0] == -1
        assert gyro[0].angular_velocity_covariance[0] > 0
        spin(0.6)
        assert len(gyro) == 1
        assert node.stale_reported
        os.close(master)
        os.close(slave)
        master, slave = os.openpty()
        link.unlink()
        link.symlink_to(os.ttyname(slave))
        spin(2.5)
        os.write(master, frame())
        spin(0.2)
        assert len(gyro) == 2
        assert not node.stale_reported
    finally:
        node.destroy_node()
        listener.destroy_node()
        rclpy.shutdown()
        os.close(master)
        os.close(slave)
