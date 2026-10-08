"""Shadow-only EGO bridge; publishes no actuator or flight-control topics."""
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from uav_nav_interfaces.msg import (MapSnapshot, TimedTrajectory, LocalizationAlignment,
                                    PlanningContext, ContextTrajectory, LocalizedOdometry)
from .planning_context import PlanningGate


class PlanningContextNode(Node):
    def __init__(self):
        super().__init__('planning_context_bridge')
        radius=self.declare_parameter('body_radius',.3).value
        limits=tuple(self.declare_parameter(name,value).value for name,value in
                     [('max_velocity',.5),('max_acceleration',1.),('max_jerk',2.)])
        self.gate=PlanningGate(radius,limits)
        self.last_error=''
        self.map_pub=self.create_publisher(MapSnapshot,'/planning/ego/map',QoSProfile(
            depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.odom_pub=self.create_publisher(Odometry,'/planning/ego/odometry',qos_profile_sensor_data)
        self.goal_pub=self.create_publisher(PoseStamped,'/planning/ego/goal',10)
        self.context_pub=self.create_publisher(PlanningContext,'/planning/context',QoSProfile(
            depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.bound_pub=self.create_publisher(ContextTrajectory,'/planning/bound_trajectory',10)
        for name,kind,topic,qos in (
                ('map',MapSnapshot,'/planning/source/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),
                ('odom',LocalizedOdometry,'/planning/source/odometry',qos_profile_sensor_data),
                ('alignment',LocalizationAlignment,'/planning/source/alignment',10)):
            self.create_subscription(kind,topic,lambda m,n=name:self.receive(n,m),qos)
        self.create_subscription(PoseStamped,'/planning/source/goal',self.goal,10)
        self.create_subscription(TimedTrajectory,'/planning/ego/trajectory',self.trajectory,10)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def now_s(self):return self.get_clock().now().nanoseconds/1e9

    def unique_sources(self):
        return all(len(self.get_publishers_info_by_topic(topic))==1 for topic in
            ('/planning/source/map','/planning/source/odometry','/planning/source/alignment'))

    def ready(self):
        if not self.unique_sources():
            self.gate.invalidate('NON_UNIQUE_SOURCE');return False
        return self.gate.ready(self.now_s(),time.monotonic())

    def context_message(self, valid):
        result=PlanningContext(context_id=self.gate.context_id,valid=valid,reason=self.gate.reason)
        result.header.stamp=self.get_clock().now().to_msg();result.header.frame_id='map'
        if 'map' in self.gate.inputs:
            m=self.gate.inputs['map'][0]
            result.map_id=m.map_id;result.map_epoch=m.epoch;result.map_version=m.version
        if 'alignment' in self.gate.inputs:
            a=self.gate.inputs['alignment'][0]
            result.localization_session=a.localization_session;result.alignment_id=a.alignment_id
            result.alignment_generation=a.generation;result.reset_counters=a.reset_counters
        return result

    def receive(self,name,message):
        self.gate.update(name,message,time.monotonic())
        self.tick()

    def reject(self,error):
        text=str(error)
        if text!=self.last_error:self.get_logger().warning(text);self.last_error=text

    def goal(self,message):
        try:
            if not self.ready():raise ValueError(self.gate.reason)
            if self.goal_pub.get_subscription_count()!=1:raise ValueError('PLANNER_NOT_UNIQUELY_READY')
            self.gate.dispatch(message,self.now_s(),time.monotonic())
            self.map_pub.publish(self.gate.inputs['map'][0])
            self.odom_pub.publish(self.gate.map_odometry)
            self.goal_pub.publish(message)
        except ValueError as error:self.reject(error)

    def trajectory(self,message):
        try:
            if not self.ready():raise ValueError(self.gate.reason)
            bound=self.gate.bind(message,self.now_s(),time.monotonic())
            self.bound_pub.publish(ContextTrajectory(context=self.context_message(True),trajectory=bound))
        except ValueError as error:self.reject(error)

    def tick(self):
        valid=self.ready()
        self.context_pub.publish(self.context_message(valid))
        if valid:
            self.map_pub.publish(self.gate.inputs['map'][0])
            self.odom_pub.publish(self.gate.map_odometry)


def main(args=None):
    rclpy.init(args=args);node=PlanningContextNode()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
