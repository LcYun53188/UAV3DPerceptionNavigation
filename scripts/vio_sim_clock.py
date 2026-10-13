#!/usr/bin/env python3
"""Simulation-only clock from unchanged Gazebo IMU physics timestamps.

World statistics clock delivery can lag native PX4 sensor time. Never extrapolate,
retimestamp sensor data, or emit a heartbeat without a real Gazebo IMU sample.
"""
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def clock_message(imu):
    result=Clock()
    result.clock.sec=imu.header.stamp.sec
    result.clock.nanosec=imu.header.stamp.nanosec
    return result


class SimulationClock(Node):
    def __init__(self):
        super().__init__('gazebo_imu_simulation_clock')
        if self.get_parameter('use_sim_time').value:
            raise RuntimeError('Clock publisher must use wall time; simulation-only owned IMU input')
        self.publisher=self.create_publisher(Clock,'/clock',QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.subscription=self.create_subscription(Imu,'/vio/imu',self.receive,qos_profile_sensor_data)
        self.last=None

    def receive(self,message):
        value=message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
        if value<=0:return
        if value==self.last:return
        if self.last is not None and value<self.last:
            self.get_logger().error('Original Gazebo IMU time moved backward; downstream reset guards remain active')
        self.last=value
        self.publisher.publish(clock_message(message))


def main():
    rclpy.init()
    node=SimulationClock()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
