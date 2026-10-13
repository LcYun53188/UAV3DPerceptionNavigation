#!/usr/bin/env python3
"""Add smoothed 3-D body velocity to navigation odometry from pose samples.

Works with Gazebo or VIO poses. This changes twist only, never pose or TF.
Filtering happens in the world frame so a turn cannot rotate stored velocity.
"""
import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile


def quaternion(orientation):
    q=[orientation.x,orientation.y,orientation.z,orientation.w]
    length=math.sqrt(sum(v*v for v in q))
    if not math.isfinite(length) or length<1e-8:raise ValueError('Invalid orientation')
    return [v/length for v in q]


def rotate(q, vector):
    x,y,z,w=q;vx,vy,vz=vector
    tx,ty,tz=2*(y*vz-z*vy),2*(z*vx-x*vz),2*(x*vy-y*vx)
    return [vx+w*tx+y*tz-z*ty,vy+w*ty+z*tx-x*tz,vz+w*tz+x*ty-y*tx]


def world_angular_velocity(previous, current, dt):
    # current * inverse(previous): shortest rotation in the world frame.
    x,y,z,w=current;a,b,c,d=previous;a,b,c=-a,-b,-c
    q=[w*a+x*d+y*c-z*b,w*b-x*c+y*d+z*a,w*c+x*b-y*a+z*d,w*d-x*a-y*b-z*c]
    if q[3]<0:q=[-v for v in q]
    length=math.sqrt(sum(v*v for v in q[:3]))
    scale=2/dt if length<1e-10 else 2*math.atan2(length,q[3])/length/dt
    return [v*scale for v in q[:3]]


class GazeboOdometryVelocity(Node):
    """Navigation twist with full attitude, bounded history and source resets."""
    def __init__(self):
        super().__init__('gazebo_odometry_velocity')
        self.declare_parameter('input_topic','/visual_slam/tracking/odometry')
        self.declare_parameter('output_topic','/navigation/odometry')
        self.declare_parameter('smoothing_factor',.45)
        self.declare_parameter('max_sample_period_sec',.25)
        self.declare_parameter('warmup_sec',0.)
        self._alpha=float(self.get_parameter('smoothing_factor').value)
        self._max_dt=float(self.get_parameter('max_sample_period_sec').value)
        if not (0<self._alpha<=1 and math.isfinite(self._max_dt) and self._max_dt>0):
            raise ValueError('Invalid smoothing factor or maximum sample period')
        self._previous=None;self._filtered_world_twist=None
        self._warmup=float(self.get_parameter('warmup_sec').value)
        if not math.isfinite(self._warmup) or self._warmup<0:raise ValueError('Invalid warmup period')
        self._first_sample_time=None
        self._publisher=self.create_publisher(Odometry,self.get_parameter('output_topic').value,QoSProfile(depth=20))
        self.create_subscription(Odometry,self.get_parameter('input_topic').value,self._on_odometry,qos_profile_sensor_data)

    @staticmethod
    def _stamp_seconds(message):
        return message.header.stamp.sec+message.header.stamp.nanosec*1e-9

    def _on_odometry(self,message):
        p=message.pose.pose.position;position=[p.x,p.y,p.z]
        try:q=quaternion(message.pose.pose.orientation)
        except ValueError:
            self._previous=None;self._filtered_world_twist=None;return
        if not all(math.isfinite(v) for v in position):
            self._previous=None;self._filtered_world_twist=None;return
        current_time=self._stamp_seconds(message);frames=(message.header.frame_id,message.child_frame_id)
        if self._first_sample_time is None or current_time<self._first_sample_time:self._first_sample_time=current_time
        if current_time-self._first_sample_time<self._warmup:
            self._previous=None;self._filtered_world_twist=None;return
        output=copy.deepcopy(message)
        if self._previous is not None:
            previous_time,previous_position,previous_q,previous_frames=self._previous
            dt=current_time-previous_time
            if frames==previous_frames and dt<=1e-4:return
            if frames==previous_frames and dt<=self._max_dt:
                measured=[(p1-p0)/dt for p1,p0 in zip(position,previous_position)]+world_angular_velocity(previous_q,q,dt)
                old=self._filtered_world_twist
                self._filtered_world_twist=measured if old is None else [self._alpha*new+(1-self._alpha)*prior for new,prior in zip(measured,old)]
            else:self._filtered_world_twist=None
        # Never forward an old velocity after a gap or source/frame reset.
        world=self._filtered_world_twist or [0.]*6
        inverse=[-q[0],-q[1],-q[2],q[3]]
        linear=rotate(inverse,world[:3]);angular=rotate(inverse,world[3:])
        output.twist.twist.linear.x,output.twist.twist.linear.y,output.twist.twist.linear.z=linear
        output.twist.twist.angular.x,output.twist.twist.angular.y,output.twist.twist.angular.z=angular
        # Pose-derived twist is not independent of pose. Navigation EKF uses
        # pose only; retain source twist covariance rather than invent certainty.
        if self._filtered_world_twist is None:
            output.twist.covariance=[0.]*36
            for i in range(6):output.twist.covariance[i*7]=1.
        self._previous=(current_time,position,q,frames)
        self._publisher.publish(output)


def main(args=None):
    rclpy.init(args=args);node=GazeboOdometryVelocity()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
