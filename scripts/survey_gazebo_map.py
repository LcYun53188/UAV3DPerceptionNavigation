#!/usr/bin/env python3
"""Offline Gazebo depth survey (teleported camera poses, NOT flight validation).
Run only in mapping mode, with no active goal. The world identity stays fixed.
"""
import argparse
import math
from pathlib import Path
import time
import rclpy
from nvblox_msgs.srv import FilePath
from std_srvs.srv import SetBool, Trigger
from ros_gz_interfaces.srv import SetEntityPose
from ros_gz_interfaces.msg import Entity
from uav_nav_interfaces.msg import MapSnapshot
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--output',required=True)
    parser.add_argument('--dwell',type=float,default=1.5)
    parser.add_argument('--layout',choices=['lab','expanded'],default='lab')
    args=parser.parse_args()
    if Path(args.output).expanduser().exists():raise ValueError('Output directory already exists')
    rclpy.init();node=rclpy.create_node('offline_gazebo_survey')
    latest=[None]
    node.create_subscription(MapSnapshot,'/uav/map/snapshot',lambda m:latest.__setitem__(0,m),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
    world_name = 'uav_ego_expanded' if args.layout == 'expanded' else 'uav_ego_lab'
    pose_client=node.create_client(SetEntityPose,f'/world/{world_name}/set_pose')
    if not pose_client.wait_for_service(timeout_sec=20):raise RuntimeError('Gazebo pose bridge unavailable')
    gate=node.create_client(SetBool,'/uav/map/input_enabled')
    if not gate.wait_for_service(timeout_sec=20):raise RuntimeError('Depth input gate unavailable')
    def set_gate(enabled):
        f=gate.call_async(SetBool.Request(data=enabled))
        rclpy.spin_until_future_complete(node,f,timeout_sec=5)
        if not f.done() or not f.result().success:raise RuntimeError('Depth gate rejected: require HOLD')
    def pose(x,y,z,yaw):
        set_gate(False)
        spin(0.2)
        request=SetEntityPose.Request()
        request.entity.name='uav_quad';request.entity.type=Entity.MODEL
        request.pose.position.x=float(x);request.pose.position.y=float(y);request.pose.position.z=float(z)
        request.pose.orientation.z=math.sin(yaw/2);request.pose.orientation.w=math.cos(yaw/2)
        for attempt in range(3):
            future=pose_client.call_async(request)
            rclpy.spin_until_future_complete(node,future,timeout_sec=12)
            if future.done() and future.result().success:break
            future.cancel();spin(0.5)
        else:raise RuntimeError('Gazebo pose service failed after 3 attempts')
        spin(0.3)
        set_gate(True)
    cancel=node.create_client(Trigger,'/uav/cancel')
    if not cancel.wait_for_service(timeout_sec=10):raise RuntimeError('Executor unavailable')
    f=cancel.call_async(Trigger.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=5)
    if not f.done() or not f.result().success:raise RuntimeError('Cannot cancel previous goal')
    spin(3)
    locations=[(-3,-3),(-3,0),(-3,3),(0,3),(3,3),(3,0),(3,-3),(0,-3)]
    if args.layout == 'expanded':
        locations=[(x,y) for x in [-7.5,-2.5,2.5,7.5] for y in [-7.5,-2.5,2.5,7.5]]
    for z in [1.2,2.2]:
        for x,y in locations:
            for yaw in [0,math.pi/2,math.pi,-math.pi/2]:
                pose(x,y,z,yaw);spin(args.dwell)
            print(f'Surveyed ({x},{y},{z}), observed={sum(latest[0].observed) if latest[0] else 0}',flush=True)
    home = (-7.5,-7.5) if args.layout == 'expanded' else (-3,0)
    pose(*home,1.2,0);spin(3)
    if latest[0] is None or not latest[0].valid:raise RuntimeError('No valid ESDF after survey')
    client=node.create_client(FilePath,'/uav/map/save')
    if not client.wait_for_service(timeout_sec=10):raise RuntimeError('Map manager absent')
    f=client.call_async(FilePath.Request(file_path=str(Path(args.output).expanduser().resolve())))
    rclpy.spin_until_future_complete(node,f,timeout_sec=60)
    if not f.done() or not f.result().success:raise RuntimeError('Map save failed')
    print(f'Saved map bundle {args.output}',flush=True)
    node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
