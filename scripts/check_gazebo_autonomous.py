#!/usr/bin/env python3
"""Bounded autonomous mapping trial with independent Gazebo geometry checks."""
import argparse
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from uav_nav_interfaces.msg import TimedTrajectory, MapSnapshot
from uav_nav_sim.core import spline
from check_gazebo_exploration import expanded_clearance


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=180.)
    parser.add_argument('--output', required=True)
    parser.add_argument('--require-reuse', action='store_true')
    args = parser.parse_args()
    if not np.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be finite and positive')
    rclpy.init()
    node = rclpy.create_node('gazebo_autonomous_check')
    client = node.create_client(SetBool, '/uav/exploration/enabled')
    truth = expanded_clearance()
    report = dict(layout='expanded', requested_duration=args.duration,
                  states=[], events=[], trajectories=[], positions=0, unsafe_abort=False,
                  minimum_clearance=None, completed=False)
    active = False
    position = None
    observed = None

    def mapping(msg):
        nonlocal observed
        if msg.valid:
            observed = int(np.count_nonzero(msg.observed))
            if active:
                report['final_observed'] = observed

    def diagnostics(msg):
        if active:
            entry = json.loads(msg.data)
            summary = report.setdefault('mapping_diagnostics', dict(samples=0, errors=[], max_source_age_at_reply=0.))
            summary['samples'] += 1
            if entry.get('error'):
                summary['errors'].append(entry['error'])
            summary['max_source_age_at_reply'] = max(summary['max_source_age_at_reply'],
                                                    entry.get('source_age_at_reply') or 0.)

    def save():
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix('.tmp')
        temporary.write_text(json.dumps(report, indent=2)+'\n')
        temporary.replace(path)

    def abort():
        report['unsafe_abort'] = True
        client.call_async(SetBool.Request(data=False))
        save()

    def odometry(msg):
        nonlocal position
        p = msg.pose.pose.position
        position = np.array([p.x,p.y,p.z])
        if active:
            distance = float(truth(position))
            old = report['minimum_clearance']
            report['minimum_clearance'] = distance if old is None else min(old,distance)
            report['positions'] += 1
            if distance <= 0 and not report['unsafe_abort']:
                abort()

    def state(msg):
        if active and (not report['states'] or report['states'][-1] != msg.data):
            report['states'].append(msg.data)
            print(msg.data,flush=True)
            save()

    def event(msg):
        if active:
            report['events'].append(msg.data)
            save()

    def trajectory(msg):
        if not active:
            return
        curve = spline([[p.x,p.y,p.z] for p in msg.control_points],msg.knot_interval)
        duration = float(curve.t[-4])
        clearance = min(float(truth(p)) for p in curve(np.linspace(0,duration,max(100,int(duration*20)))))
        report['trajectories'].append(dict(id=msg.trajectory_id,parent_id=msg.parent_trajectory_id,
                                         duration=duration,clearance=clearance,
                                         endpoint=curve(duration).tolist()))
        if clearance <= 0:
            abort()
        save()

    qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Odometry,'/uav/localization/odometry',odometry,qos_profile_sensor_data)
    node.create_subscription(String,'/uav/exploration/state',state,qos)
    node.create_subscription(String,'/uav/executor/event',event,10)
    node.create_subscription(TimedTrajectory,'/uav/trajectory',trajectory,10)
    node.create_subscription(MapSnapshot,'/uav/map/snapshot',mapping,qos)
    node.create_subscription(String,'/uav/map/diagnostics',diagnostics,10)
    try:
        if not client.wait_for_service(timeout_sec=20):
            raise RuntimeError('No exploration service')
        deadline=time.monotonic()+10
        while (position is None or observed is None) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
        if position is None or observed is None:
            raise RuntimeError('No odometry or valid map')
        report['start']=position.tolist()
        report['initial_observed']=observed
        report['final_observed']=observed
        active=True
        future=client.call_async(SetBool.Request(data=True))
        rclpy.spin_until_future_complete(node,future,timeout_sec=10)
        if not future.done() or not future.result().success:
            raise RuntimeError('Exploration startup failed')
        started=time.monotonic()
        while time.monotonic()-started<args.duration and not report['unsafe_abort']:
            rclpy.spin_once(node,timeout_sec=.05)
            if report['states'] and report['states'][-1].split(':')[0] in (
                    'STOPPED','BLOCKED','FRONTIERS_EXHAUSTED','LIMIT_REACHED'):
                break
        report['duration']=time.monotonic()-started
    finally:
        report['terminal_before_disable'] = report['states'][-1] if report['states'] else None
        report['disable_confirmed'] = False
        if client.service_is_ready():
            future=client.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(node,future,timeout_sec=5)
            report['disable_confirmed'] = bool(future.done() and future.result() is not None and future.result().success)
        stopped = position.copy() if position is not None else None
        drift=0.
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.05)
            if stopped is not None and position is not None:
                drift=max(drift,float(np.linalg.norm(position-stopped)))
        report['hold_drift']=drift
        report['reused_observations']=report['events'].count('OBSERVATION_REUSED')
        report['reached_viewpoints']=report['events'].count('GOAL_REACHED')
        report['final_position']=position.tolist() if position is not None else None
        report['completed']=True
        terminal = (report['terminal_before_disable'] or '').split(':')[0]
        report['passed']=bool(not report['unsafe_abort'] and report['reached_viewpoints']>0 and
                              terminal in ('SCANNING', 'EXPLORING', 'FRONTIERS_EXHAUSTED', 'LIMIT_REACHED') and
                              report['disable_confirmed'] and
                              report.get('final_observed',0)>report.get('initial_observed',0) and
                              report['minimum_clearance'] is not None and report['minimum_clearance']>0 and
                              drift<.05 and (not args.require_reuse or report['reused_observations']>0))
        save()
        print(json.dumps({k:report[k] for k in ('passed','reached_viewpoints','reused_observations',
                         'minimum_clearance','hold_drift')},indent=2),flush=True)
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if report['passed'] else 1)


if __name__ == '__main__':
    main()
