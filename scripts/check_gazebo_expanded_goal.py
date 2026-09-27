#!/usr/bin/env python3
"""Expanded-world RViz goal regression with independent SDF geometry checks.

Requires uav_ego_expanded, a loaded map, and rviz_fixed_height_goal at 1.2 m.
Publishes a real simulation goal through /uav/rviz/goal_2d. This deliberately
uses the expanded world's geometry, rather than the small lab's old validator.
"""
import argparse,json,math,time,xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
from uav_nav_interfaces.msg import TimedTrajectory,MapSnapshot
from uav_nav_sim.core import grid_from_message,spline,validate_trajectory
p=argparse.ArgumentParser();p.add_argument('--goal',nargs=2,type=float,default=[7.99,4.25]);p.add_argument('--timeout',type=float,default=240);p.add_argument('--output',default='/tmp/uav_expanded_goal_result.json');a=p.parse_args()
w=ET.parse('src/uav_bringup/gazebo/worlds/uav_ego_expanded.sdf').getroot().find('world');obstacles=[]
for m in w.findall('model'):
 if m.get('name')=='ground_plane':continue
 pose=list(map(float,m.findtext('pose').split()));g=m.find('link/collision/geometry')
 obstacles.append((pose,g))
def clearance(point):
 margin=point[2]
 for pose,g in obstacles:
  d=np.array(point)-pose[:3];yaw=pose[5];d=np.array([math.cos(yaw)*d[0]+math.sin(yaw)*d[1],-math.sin(yaw)*d[0]+math.cos(yaw)*d[1],d[2]])
  if g.find('box') is not None:q=np.abs(d)-np.array(list(map(float,g.findtext('box/size').split())))/2
  else:q=np.array([np.linalg.norm(d[:2])-float(g.findtext('cylinder/radius')),abs(d[2])-float(g.findtext('cylinder/length'))/2])
  distance=np.linalg.norm(np.maximum(q,0))+min(float(np.max(q)),0.)
  margin=min(margin,distance)
 return float(margin-.3)
rclpy.init();n=rclpy.create_node('expanded_navigation_regression');state={'events':[],'planner':[],'positions':[],'trajectories':[]};grid=[None]
def onmap(m):
 if m.valid and grid[0] is None:grid[0]=grid_from_message(m)
def odom(m):
 pos=m.pose.pose.position;state['position']=[pos.x,pos.y,pos.z]
 if state['trajectories']:state['positions'].append(state['position'])
def event(m):
 state['events'].append(m.data);print('event:',m.data,flush=True)
def status(m):
 if not state['planner'] or state['planner'][-1]!=m.data:state['planner'].append(m.data);print('planner:',m.data,flush=True)
def trajectory(m):
 controls=[[v.x,v.y,v.z] for v in m.control_points];curve=spline(controls,m.knot_interval)
 entry={'id':m.trajectory_id,'duration':float(curve.t[-4]),'controls':controls,'interval':m.knot_interval}
 try:entry['bounds']=validate_trajectory(curve,grid[0],.3,[.5,1.,2.]);entry['valid']=True
 except ValueError as e:entry.update(valid=False,error=str(e))
 entry['truth_clearance']=min(clearance(p) for p in curve(np.linspace(0,entry['duration'],max(100,int(entry['duration']*20)))))
 state['trajectories'].append(entry);print('trajectory:',{k:v for k,v in entry.items() if k!='controls'},flush=True)
subs=[n.create_subscription(MapSnapshot,'/uav/map/snapshot',onmap,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)),n.create_subscription(Odometry,'/uav/localization/odometry',odom,qos_profile_sensor_data),n.create_subscription(String,'/uav/planner/state',status,10),n.create_subscription(String,'/uav/executor/event',event,10),n.create_subscription(TimedTrajectory,'/uav/trajectory',trajectory,10)]
pub=n.create_publisher(PoseStamped,'/uav/rviz/goal_2d',10)
end=time.monotonic()+15
while time.monotonic()<end and (grid[0] is None or 'position' not in state or pub.get_subscription_count()==0):rclpy.spin_once(n,timeout_sec=.1)
assert grid[0] is not None and 'position' in state
state['start']=state['position'];goal=PoseStamped();goal.header.frame_id='map';goal.pose.position.x,goal.pose.position.y=a.goal;goal.pose.orientation.w=1.;pub.publish(goal)
end=time.monotonic()+a.timeout
while time.monotonic()<end:
 rclpy.spin_once(n,timeout_sec=.1)
 if 'GOAL_REACHED' in state['events']:break
 if len(state['events'])>10:break
state['final_error']=float(np.linalg.norm(np.array(state['position'])-[*a.goal,1.2]));positions=state.pop('positions');state['samples']=len(positions);state['min_truth_clearance']=min(map(clearance,positions)) if positions else None
state['passed']='GOAL_REACHED' in state['events'] and state['final_error']<.1 and bool(positions) and state['min_truth_clearance']>0 and all(t['valid'] and t['truth_clearance']>0 for t in state['trajectories'])
Path(a.output).write_text(json.dumps(state,indent=2));print('result:',{k:v for k,v in state.items() if k!='trajectories'},flush=True)
n.destroy_node();rclpy.shutdown();raise SystemExit(0 if state['passed'] else 1)
