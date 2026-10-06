#!/usr/bin/env python3
"""Deterministic ROS planner/executor check with an incrementally observed corridor.

No Gazebo, camera, or hardware commands: runs in a separate ROS domain and drives
an ideal velocity model. This checks control handovers, not perception or SLAM.
Run through scripts/with_venv.sh. Compare --mode baseline and --mode continuous.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from uav_nav_interfaces.msg import MapSnapshot, TimedTrajectory
from uav_nav_sim.core import Grid, pack_grid_data, spline, validate_handover


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['baseline', 'continuous'], default='continuous')
    parser.add_argument('--scenario', choices=['straight','bend'], default='straight')
    parser.add_argument('--background-replan', choices=['true','false'], default='true')
    parser.add_argument('--domain', type=int, default=79)
    parser.add_argument('--timeout', type=float, default=100.)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    root = Path(__file__).resolve().parents[1]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    enabled = str(args.mode == 'continuous').lower()
    processes, logs = [], []
    report = dict(mode=args.mode, scenario='synthetic_incremental_'+args.scenario, background_replan=args.background_replan, events=[], states=[], handovers=[], samples=[])
    try:
        for package, executable, params in [
            ('uav_ego_adapter', 'ego_nvblox_planner', ['--params-file', str(root/'src/uav_bringup/config/uav_ego_nvblox.yaml')]),
            ('uav_nav_sim', 'gazebo_executor', ['-p', 'explore_unknown:=true', '-p', f'moving_handover:={enabled}',
                                                        '-p', f'background_replan:={args.background_replan}'])]:
            log = args.output.with_suffix('.'+executable+'.log').open('w')
            logs.append(log)
            command = ['ros2','run',package,executable,'--ros-args',*params,
                       '-p','managed_goals:=true','-p',f'continuous_navigation:={enabled}']
            if package == 'uav_ego_adapter':
                command += ['-r','/uav/goal:=/uav/local_goal']
            processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        rclpy.init()
        node = rclpy.create_node('continuous_navigation_validation')
        durable = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        map_pub = node.create_publisher(MapSnapshot,'/uav/map/snapshot',durable)
        odom_pub = node.create_publisher(Odometry,'/uav/localization/odometry',qos_profile_sensor_data)
        goal_pub = node.create_publisher(PoseStamped,'/uav/goal',10)
        position = np.zeros(3)
        velocity = np.zeros(3)
        yaw = 0.
        command = Twist()
        command_at = 0.
        trajectories = {}
        started = None
        state = ''
        def command_cb(msg):
            nonlocal command, command_at
            command, command_at = msg, time.monotonic()
        def event_cb(msg):
            report['events'].append(msg.data)
            if msg.data.startswith('HANDOVER:'):
                parent, child = map(int,msg.data.split(':')[1].split('->'))
                before, after = trajectories[parent], trajectories[child]
                old = spline([[p.x,p.y,p.z] for p in before.control_points],before.knot_interval)
                new = spline([[p.x,p.y,p.z] for p in after.control_points],after.knot_interval)
                stamp = lambda t: t.sec+t.nanosec/1e9
                offset = stamp(after.start_time)-stamp(before.start_time)
                validate_handover(old,new,offset)
                report['handovers'].append(dict(speed=float(np.linalg.norm(new(0,1))),
                    continuity_errors=[float(np.linalg.norm(old(offset,d)-new(0,d))) for d in (0,1,2)]))
        def state_cb(msg):
            nonlocal state
            state = msg.data
            if not report['states'] or report['states'][-1] != state:
                report['states'].append(state)
                print(state,flush=True)
        node.create_subscription(Twist,'/cmd_vel',command_cb,10)
        node.create_subscription(String,'/uav/executor/event',event_cb,10)
        node.create_subscription(String,'/uav/navigation/state',state_cb,durable)
        node.create_subscription(TimedTrajectory,'/uav/trajectory',lambda m: trajectories.__setitem__(m.trajectory_id,m),10)
        distance = np.full((80,50 if args.scenario=='bend' else 20,20),3.,dtype=np.float32)
        x = -2+(np.arange(80)+.5)*.2
        def obstacle_distance(p):
            q=np.abs(np.asarray(p)[:2]-[3.7,-.65])-[.5,1.35]
            return float(np.linalg.norm(np.maximum(q,0))+min(max(q),0))
        if args.scenario=='bend':
            y=-2+(np.arange(distance.shape[1])+.5)*.2
            for ix,px in enumerate(x):
                for iy,py in enumerate(y):
                    distance[ix,iy,:]=obstacle_distance([px,py])
        next_map = 0.; version = 0
        previous = begin = time.monotonic()
        known_until = 3.
        while time.monotonic()-begin < args.timeout:
            if any(process.poll() is not None for process in processes):
                raise RuntimeError('Planner/executor exited; inspect the companion logs')
            rclpy.spin_once(node,timeout_sec=.002)
            now = time.monotonic()
            if now-previous < .02:
                continue
            dt, previous = now-previous, now
            if now-command_at < .3:
                c,s = math.cos(yaw),math.sin(yaw)
                v = command.linear
                velocity = np.array([c*v.x-s*v.y,s*v.x+c*v.y,v.z])
                position += velocity*dt
                yaw += command.angular.z*dt
            else:
                velocity[:] = 0.
            odom = Odometry()
            odom.header.stamp = node.get_clock().now().to_msg()
            odom.header.frame_id,odom.child_frame_id = 'odom','base_link'
            odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z = map(float,position)
            odom.pose.pose.orientation.z,odom.pose.pose.orientation.w = math.sin(yaw/2),math.cos(yaw/2)
            odom.twist.twist.linear.x,odom.twist.twist.linear.y,odom.twist.twist.linear.z = map(float,velocity)
            odom_pub.publish(odom)
            if now >= next_map:
                known_until = max(known_until,position[0]+3.)
                observed = np.broadcast_to((x<known_until)[:,None,None],distance.shape).copy()
                grid = Grid(np.array([-2.,-2.,-2.]),.2,distance,observed)
                msg = MapSnapshot(map_id='synthetic',epoch=1,version=version+1,valid=True,resolution=.2)
                msg.header.frame_id='map'; msg.header.stamp=node.get_clock().now().to_msg()
                msg.origin.x=msg.origin.y=msg.origin.z=-2.
                msg.shape=list(distance.shape); msg.distance,msg.observed=pack_grid_data(grid)
                map_pub.publish(msg);version+=1;next_map=now+.5
            if started is None and now-begin>3 and goal_pub.get_subscription_count()>0:
                goal = PoseStamped();goal.header.frame_id='map';goal.pose.position.x=8.
                goal.pose.orientation.w=1.;goal_pub.publish(goal);started=now
            if started is not None:
                report['samples'].append([round(now-started,3),*position.tolist(),float(np.linalg.norm(velocity))])
                if state=='REACHED' or state.startswith(('STOPPED','BLOCKED')):
                    break
        if not report['samples']:
            raise RuntimeError('No goal subscriber became ready before timeout')
        samples=np.array(report['samples'])
        moving=samples[:,4]>.05
        moving_indices=np.flatnonzero(moving)
        interior=samples[moving_indices[0]:moving_indices[-1]+1] if len(moving_indices) else samples
        speed=interior[:,4]
        report.update(completed=state=='REACHED',duration=float(samples[-1,0]),
            stop_count=int(np.count_nonzero((speed[:-1]>.05)&(speed[1:]<=.05))),
            stopped_seconds=float(np.sum(np.diff(interior[:,0])*(speed[:-1]<=.05))),
            final_position=position.tolist(),accepted_segments=sum(e.startswith('ACCEPTED:') for e in report['events']))
        args.output.write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({k:v for k,v in report.items() if k not in ('samples','states','events')},indent=2))
        if args.scenario=='bend':
            report['minimum_obstacle_clearance']=min(obstacle_distance(p)-.3 for p in samples[:,1:4])
            args.output.write_text(json.dumps(report,indent=2)+'\n')
            if report['minimum_obstacle_clearance']<=0:
                raise RuntimeError('Sampled path violated obstacle clearance')
        if not report['completed'] or (args.mode=='continuous' and not report['handovers']):
            raise RuntimeError('Scenario did not complete with the expected moving handovers')
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        for process in processes:
            if process.poll() is None:
                os.killpg(process.pid,signal.SIGINT)
        for process in processes:
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGTERM)
                process.wait(timeout=5)
        for log in logs:
            log.close()


if __name__ == '__main__':
    main()
