#!/usr/bin/env python3
"""Real EGO + context bridge DDS regression on a synthetic ESDF, no flight outputs."""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry
from uav_nav_interfaces.msg import (MapSnapshot, LocalizationAlignment, PlanningContext,
                                    ContextTrajectory, TimedTrajectory, LocalizedOdometry)
from uav_nav_sim.core import grid_from_message, spline, validate_trajectory

ROOT=Path(__file__).resolve().parents[1]


def main():
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--domain',type=int,default=91,choices=range(80,101))
    cli.add_argument('--output',type=Path,default=ROOT/'.cache/simulation'/('planning-context-'+str(time.time_ns())))
    args=cli.parse_args()
    if args.output.exists() and any(args.output.iterdir()):cli.error('Output directory must be empty')
    args.output.mkdir(parents=True,exist_ok=True)
    os.environ['ROS_DOMAIN_ID']=str(args.domain)
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
    rclpy.init();node=rclpy.create_node('planning_context_smoke')
    children=[];logs=[];contexts=[];bound=[];mapped=[]
    result=dict(passed=False,scope='Synthetic ESDF, real EGO/DDS; shadow planning only',domain=args.domain)
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    pubs=dict(map=node.create_publisher(MapSnapshot,'/planning/source/map',qos),
              odom=node.create_publisher(LocalizedOdometry,'/planning/source/odometry',qos_profile_sensor_data),
              alignment=node.create_publisher(LocalizationAlignment,'/planning/source/alignment',10))
    goal_pub=node.create_publisher(PoseStamped,'/planning/source/goal',10)
    raw_pub=node.create_publisher(TimedTrajectory,'/planning/ego/trajectory',10)
    node.create_subscription(PlanningContext,'/planning/context',contexts.append,qos)
    node.create_subscription(ContextTrajectory,'/planning/bound_trajectory',bound.append,10)
    node.create_subscription(Odometry,'/planning/ego/odometry',mapped.append,qos_profile_sensor_data)
    m=MapSnapshot(map_id='synthetic-obstacle',epoch=1,version=1,valid=True,resolution=.2,shape=[40,40,25])
    m.header.frame_id='map';m.origin=Point(x=0.,y=-6.,z=0.)
    distance=np.full((40,40,25),2.,dtype=np.float32)
    distance[24:26,16:24,:]=0.  # Solid wall across the direct start/goal chord.
    m.distance=distance.ravel();m.observed=[1]*40000
    o=Odometry();o.header.frame_id='odom';o.child_frame_id='base_link'
    o.pose.pose.position=Point(x=1.,y=0.,z=1.);o.pose.pose.orientation.w=1.
    a=LocalizationAlignment(map_id=m.map_id,map_epoch=1,localization_session='synthetic-ekf',
        alignment_id='explicit-nonidentity',generation=1,valid=True)
    a.header.frame_id='map';a.map_to_odom.translation.x=4.;a.map_to_odom.translation.y=-3.
    a.map_to_odom.translation.z=1.;a.map_to_odom.rotation.z=math.sin(math.pi/4)
    a.map_to_odom.rotation.w=math.cos(math.pi/4)
    flags=dict(alignment=True)
    def publish():
        current=node.get_clock().now().to_msg()
        m.version+=1;m.header.stamp=current;o.header.stamp=current
        pubs['map'].publish(m);pubs['odom'].publish(LocalizedOdometry(
            header=deepcopy(o.header),localization_session=a.localization_session,
            reset_counters=a.reset_counters,odometry=o))
        if flags['alignment']:
            a.header.stamp=current;pubs['alignment'].publish(a)
    node.create_timer(.1,publish)
    def spin_until(check,timeout=8):
        until=time.monotonic()+timeout
        while time.monotonic()<until:
            rclpy.spin_once(node,timeout_sec=.03)
            if check():return
            if any(p.poll() is not None for p in children):raise RuntimeError('Owned planning process exited')
        raise RuntimeError('Timed out; latest context: '+str(contexts[-1] if contexts else None))
    def start(name,command):
        log=(args.output/(name+'.log')).open('w');logs.append(log)
        children.append(subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
    try:
        start('bridge',['ros2','run','uav_nav_sim','planning_context'])
        remaps={'/uav/map/snapshot':'/planning/ego/map','/uav/localization/odometry':'/planning/ego/odometry',
                '/uav/goal':'/planning/ego/goal','/uav/trajectory':'/planning/ego/trajectory',
                '/uav/planned_path':'/planning/ego/path'}
        command=[str(ROOT/'install_uav/uav_ego_adapter/lib/uav_ego_adapter/ego_nvblox_planner'),
                 '--ros-args','-p','coordinate_frame:=map','-p','managed_goals:=true']
        for source,target in remaps.items():command.extend(['-r',source+':='+target])
        start('ego',command)
        spin_until(lambda:contexts and contexts[-1].valid and mapped and
                   node.count_subscribers('/planning/ego/goal')==1 and
                   node.count_publishers('/planning/ego/trajectory')==2)
        result['planner_discovery_confirmed']=True
        initial=deepcopy(contexts[-1]);goal=PoseStamped()
        goal.header.frame_id='map';goal.header.stamp=node.get_clock().now().to_msg()
        goal.pose.position=Point(x=6.,y=-2.,z=2.);goal.pose.orientation.w=1.
        goal_pub.publish(goal);spin_until(lambda:bool(bound))
        accepted=bound[0];t=accepted.trajectory
        assert accepted.context.context_id==initial.context_id and t.header.frame_id=='map'
        assert t.goal_stamp==goal.header.stamp and (t.map_id,t.epoch)==(m.map_id,m.epoch)
        curve=spline([[p.x,p.y,p.z] for p in t.control_points],t.knot_interval)
        grid=grid_from_message(m);validate_trajectory(curve,grid,.3,(.5,1.,2.))
        samples=curve(np.linspace(0,curve.t[-4],300))
        assert max(abs(samples[:,1]+2.))>.9,'No obstacle detour'
        assert any(grid.collision(p,.3) for p in np.linspace([4.,-2.,2.],[6.,-2.,2.],40))
        actual=mapped[-1].pose.pose.position
        assert np.allclose([actual.x,actual.y,actual.z],[4.,-2.,2.])
        result['normal']=dict(context_id=initial.context_id,controls=len(t.control_points),
            duration_s=float(curve.t[-4]),detour_y_m=float(max(abs(samples[:,1]+2.))),
            start=samples[0].tolist(),end=samples[-1].tolist())
        def reject_old():
            before=len(bound);raw_pub.publish(t)
            until=time.monotonic()+.3
            while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
            assert len(bound)==before,'Stale trajectory was bound'
        reject_old();result['duplicate_result_rejected']=True
        # A second source invalidates the context even before it emits data.
        extra=node.create_publisher(LocalizationAlignment,'/planning/source/alignment',10)
        spin_until(lambda:contexts and contexts[-1].reason=='NON_UNIQUE_SOURCE')
        node.destroy_publisher(extra);spin_until(lambda:contexts[-1].valid)
        result['multiple_source_rejected']=True
        a.map_to_odom.translation.x+=.1
        spin_until(lambda:not contexts[-1].valid and contexts[-1].reason=='ALIGNMENT_GENERATION_CONFLICT')
        reject_old();result['same_generation_change_rejected']=True
        a.generation=2
        spin_until(lambda:contexts[-1].valid and contexts[-1].context_id!=initial.context_id)
        reject_old();result['new_alignment_retires_old_goal']=True
        m.epoch=2;m.version=1
        spin_until(lambda:not contexts[-1].valid and contexts[-1].reason=='MAP_ALIGNMENT_MISMATCH')
        a.map_epoch=2;a.generation=3
        spin_until(lambda:contexts[-1].valid and contexts[-1].map_epoch==2)
        reject_old();result['map_epoch_requires_rebind']=True
        flags['alignment']=False
        spin_until(lambda:not contexts[-1].valid and contexts[-1].reason=='STALE_ALIGNMENT')
        reject_old();result['alignment_timeout_rejected']=True
        forbidden=[]
        for topic,_ in node.get_topic_names_and_types():
            if topic=='/cmd_vel' or '/fmu/' in topic:forbidden.append(topic)
        assert not forbidden,forbidden
        # Exercise the default legacy identity profile with the same rebuilt C++ binary.
        legacy=[]
        legacy_map=node.create_publisher(MapSnapshot,'/legacy/map',qos)
        legacy_odom=node.create_publisher(Odometry,'/legacy/odometry',qos_profile_sensor_data)
        legacy_goal=node.create_publisher(PoseStamped,'/legacy/goal',10)
        node.create_subscription(TimedTrajectory,'/legacy/trajectory',legacy.append,10)
        local=Odometry();local.header.frame_id='odom';local.child_frame_id='base_link'
        local.pose.pose.position=Point(x=4.,y=-2.,z=2.);local.pose.pose.orientation.w=1.
        def legacy_publish():
            local.header.stamp=node.get_clock().now().to_msg()
            legacy_map.publish(m);legacy_odom.publish(local)
        node.create_timer(.1,legacy_publish)
        command=[str(ROOT/'install_uav/uav_ego_adapter/lib/uav_ego_adapter/ego_nvblox_planner'),
                 '--ros-args','-p','managed_goals:=true']
        for source,target in {'/uav/map/snapshot':'/legacy/map','/uav/localization/odometry':'/legacy/odometry',
                '/uav/goal':'/legacy/goal','/uav/trajectory':'/legacy/trajectory','/uav/planned_path':'/legacy/path'}.items():
            command.extend(['-r',source+':='+target])
        start('legacy-ego',command)
        spin_until(lambda:legacy_goal.get_subscription_count()==1)
        # Wait for map/odometry discovery before the one-shot goal.
        until=time.monotonic()+.4
        while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
        goal.header.stamp=node.get_clock().now().to_msg();legacy_goal.publish(goal)
        spin_until(lambda:bool(legacy))
        assert legacy[0].header.frame_id=='odom' and legacy[0].goal_stamp==goal.header.stamp
        legacy_curve=spline([[p.x,p.y,p.z] for p in legacy[0].control_points],legacy[0].knot_interval)
        validate_trajectory(legacy_curve,grid_from_message(m),.3,(.5,1.,2.))
        result['legacy_identity_profile_passed']=True
        result['flight_control_topics']=forbidden;result['bound_count']=len(bound)
        assert len(bound)==1
        result['passed']=True
        from rosidl_runtime_py.convert import message_to_ordereddict
        (args.output/'bound-trajectory.json').write_text(json.dumps(message_to_ordereddict(accepted),indent=2)+'\n')
        (args.output/'contexts.json').write_text(json.dumps([message_to_ordereddict(x) for x in contexts],indent=2)+'\n')
    except Exception as error:result['error']=str(error)
    finally:
        for child in children:
            if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        for child in children:
            try:child.wait(timeout=5)
            except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
        result['cleanup_exit_codes']=[child.returncode for child in children]
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
        files=['src/uav_nav_sim/uav_nav_sim/planning_context.py','src/uav_nav_sim/uav_nav_sim/planning_context_node.py',
               'src/uav_ego_adapter/src/planner.cpp','scripts/run_planning_context_smoke.py',
               'install_uav/uav_ego_adapter/lib/uav_ego_adapter/ego_nvblox_planner']
        result['source_sha256']={name:hashlib.sha256((ROOT/name).read_bytes()).hexdigest() for name in files}
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
