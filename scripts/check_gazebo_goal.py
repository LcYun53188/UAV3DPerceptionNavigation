#!/usr/bin/env python3
"""One goal regression in uav_ego_lab; ground-truth cylinder/wall clearance."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from uav_nav_interfaces.msg import TimedTrajectory
from uav_nav_sim.core import spline, derivative_bounds


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--goal',nargs=3,type=float,default=[3.0,0.0,1.2]);parser.add_argument('--output',default='/tmp/uav_goal_result.json');parser.add_argument('--timeout',type=float,default=120)
    args=parser.parse_args();rclpy.init();node=rclpy.create_node('gazebo_goal_check')
    state={'odometry':None,'events':[],'planner':None,'trajectory':None,'positions':[]}
    def odom(m):
        p=m.pose.pose.position;state['odometry']=[p.x,p.y,p.z]
        if state['trajectory'] is not None:state['positions'].append([p.x,p.y,p.z])
    def event(m):
        state['events'].append(m.data);print(m.data,flush=True)
    def plan(m):
        if m.data!=state['planner']: print('planner:',m.data,flush=True)
        state['planner']=m.data
    def traj(m):
        curve=spline([[p.x,p.y,p.z] for p in m.control_points],m.knot_interval)
        state['trajectory']={'id':m.trajectory_id,'map_id':m.map_id,'epoch':m.epoch,'map_version':m.map_version,'duration':float(curve.t[-4]),'bounds':derivative_bounds(curve)}
    node.create_subscription(Odometry,'/uav/localization/odometry',odom,qos_profile_sensor_data)
    node.create_subscription(String,'/uav/executor/event',event,10)
    node.create_subscription(String,'/uav/planner/state',plan,10)
    node.create_subscription(TimedTrajectory,'/uav/trajectory',traj,10)
    pub=node.create_publisher(PoseStamped,'/uav/goal',10)
    end=time.monotonic()+5
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.1)
    goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=args.goal;goal.pose.orientation.w=1.;pub.publish(goal)
    end=time.monotonic()+args.timeout
    while time.monotonic()<end:
        rclpy.spin_once(node,timeout_sec=.1)
        if 'GOAL_REACHED' in state['events']:break
        if any(e.startswith(('REJECTED','TRACKING_ERROR','MAP_RECHECK_FAILED','STALE','CURRENT_VOLUME')) for e in state['events']):break
    positions=state.pop('positions')
    # Independent analytic truth for this fixed SDF world (body sphere r=0.3).
    margins=[min(math.hypot(p[0],p[1])-0.45,4.51-abs(p[0]),4.21-abs(p[1]),p[2])-0.3 for p in positions]
    state.update(goal=args.goal, samples=len(positions), min_truth_clearance=min(margins) if margins else None,
                 final_error=float(np.linalg.norm(np.array(state['odometry'])-args.goal)) if state['odometry'] else None)
    state['passed']=('GOAL_REACHED' in state['events'] and state['final_error']<.1 and state['min_truth_clearance'] is not None and state['min_truth_clearance']>0)
    Path(args.output).write_text(json.dumps(state,indent=2)+'\n');print(json.dumps(state,indent=2),flush=True)
    node.destroy_node();rclpy.shutdown()
    raise SystemExit(0 if state['passed'] else 1)


if __name__=='__main__':main()
