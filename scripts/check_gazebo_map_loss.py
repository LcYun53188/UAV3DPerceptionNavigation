#!/usr/bin/env python3
"""Fault injection for the dedicated Gazebo algorithm test process only."""
import argparse
import json
import os
from pathlib import Path
import signal
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger
from nvblox_msgs.srv import FilePath


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--map-session-pid',required=True,type=int)
    parser.add_argument('--map-directory',required=True)
    parser.add_argument('--output',default='/tmp/uav_map_loss.json')
    args=parser.parse_args()
    command=Path(f'/proc/{args.map_session_pid}/cmdline').read_bytes()
    if b'/uav_nav_sim/map_session' not in command:raise ValueError('PID is not the expected simulation map session')
    rclpy.init();node=rclpy.create_node('gazebo_map_loss_check')
    state={'executor':'HOLD','events':[],'command':None}
    node.create_subscription(String,'/uav/executor/state',lambda m:state.__setitem__('executor',m.data),10)
    node.create_subscription(String,'/uav/executor/event',lambda m:state['events'].append(m.data),10)
    node.create_subscription(Twist,'/cmd_vel',lambda m:state.__setitem__('command',[m.linear.x,m.linear.y,m.linear.z,m.angular.z]),10)
    publisher=node.create_publisher(PoseStamped,'/uav/goal',10)
    def spin(seconds,condition=lambda:False):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.05)
            if condition():return True
        return False
    frozen=False
    result={}
    try:
        spin(3)
        goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x=3.;goal.pose.position.y=0.;goal.pose.position.z=1.2;publisher.publish(goal)
        if not spin(15,lambda:state['executor']=='EXECUTING'):raise RuntimeError('No execution to fault-inject')
        client=node.create_client(FilePath,'/uav/map/load')
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Map service missing')
        future=client.call_async(FilePath.Request(file_path=str(Path(args.map_directory).resolve())))
        rclpy.spin_until_future_complete(node,future,timeout_sec=5)
        result['load_during_execution_rejected']=future.done() and not future.result().success
        os.kill(args.map_session_pid,signal.SIGSTOP);frozen=True
        result['map_loss_stops']=spin(8,lambda:'STALE_MAP_OR_ODOMETRY' in state['events'])
        spin(.5)
        result['zero_command_after_loss']=state['command'] is not None and all(abs(v)<1e-8 for v in state['command'])
        result['state_after_loss']=state['executor']
        result['events']=state['events']
        result['passed']=result['load_during_execution_rejected'] and result['map_loss_stops'] and result['zero_command_after_loss'] and state['executor']=='HOLD'
    finally:
        cancel=node.create_client(Trigger,'/uav/cancel')
        if cancel.wait_for_service(timeout_sec=2):
            rclpy.spin_until_future_complete(node,cancel.call_async(Trigger.Request()),timeout_sec=3)
        if frozen:os.kill(args.map_session_pid,signal.SIGCONT)
        Path(args.output).write_text(json.dumps(result,indent=2)+'\n')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result.get('passed') else 1)


if __name__=='__main__':main()
