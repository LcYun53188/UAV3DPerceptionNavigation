#!/usr/bin/env python3
"""Collect raw OAK-D VIO evidence in an isolated ROS domain, without flight nodes.

Static drift is meaningful only when the operator keeps the camera stationary.
Receipt age includes the driver's device-to-host clock offset; it is not a
physical sensor latency calibration.
"""

import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[2])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import argparse
from collections import Counter
import gzip
import json
import math
import os
from pathlib import Path
import signal
import struct
import subprocess
import time
import zlib

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu
from tf2_msgs.msg import TFMessage
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus

ROOT = Path(__file__).resolve().parents[3]


def save_mono_png(path, frame):
    def chunk(tag, data):
        return struct.pack('>I',len(data))+tag+data+struct.pack('>I',zlib.crc32(tag+data)&0xffffffff)
    h,w=frame.shape
    raw=b''.join(b'\0'+row.tobytes() for row in frame)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',w,h,8,0,0,0,0))
        +chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    return None if not len(a) else dict(min=float(a.min()), median=float(np.median(a)),
        p95=float(np.percentile(a, 95)), p99=float(np.percentile(a, 99)), max=float(a.max()))


def summarize(rows, start, end, expected_imu_hz=250):
    streams = {}
    for name in sorted({r['stream'] for r in rows}):
        all_r = [r for r in rows if r['stream'] == name]
        rr = [r for r in all_r if start <= r['elapsed'] <= end]
        t = np.array([r['stamp_ns'] for r in rr], dtype=np.int64)
        arrivals = np.array([r['elapsed'] for r in rr])
        streams[name] = dict(count=len(rr), hz=len(rr)/(end-start),
            first_elapsed=all_r[0]['elapsed'], last_elapsed=all_r[-1]['elapsed'],
            max_silence_sec=float(np.diff(np.r_[start, arrivals, end]).max()),
            nonmonotonic=int(np.sum(np.diff(t) <= 0)),
            stamp_interval_ms=stats(np.diff(t)/1e6),
            receipt_age_ms=stats([r['age_ms'] for r in rr]))
        if name == 'status':
            streams[name]['vo_states'] = dict(Counter(str(r['vo_state']) for r in rr))
            streams[name]['callback_ms'] = stats([r['callback_sec']*1000 for r in rr])
        if name == 'imu' and rr:
            accel=np.array([r['accel'] for r in rr]); gyro=np.array([r['gyro'] for r in rr])
            streams[name].update(accel_mean=accel.mean(axis=0).tolist(),
                accel_std=accel.std(axis=0).tolist(), gyro_mean=gyro.mean(axis=0).tolist(),
                gyro_std=gyro.std(axis=0).tolist())
        if name == 'odometry' and rr:
            p = np.array([r['position'] for r in rr]); q = np.array([r['quaternion'] for r in rr])
            steps = np.linalg.norm(np.diff(p, axis=0), axis=1)
            norms = np.linalg.norm(q, axis=1)
            q = q / norms[:, None]
            rot = np.degrees(2*np.arccos(np.clip(np.abs(np.sum(q[1:]*q[:-1], axis=1)), 0, 1)))
            excursion = np.degrees(2*np.arccos(np.clip(np.abs(q@q[0]),0,1)))
            cov=np.array([r['covariance_diag'] for r in rr])
            streams[name].update(net_displacement_m=float(np.linalg.norm(p[-1]-p[0])),
                max_displacement_from_first_m=float(np.linalg.norm(p-p[0],axis=1).max()),
                axis_span_m=np.ptp(p,axis=0).tolist(), step_m=stats(steps),
                angular_step_deg=stats(rot), quaternion_norm=stats(norms),
                max_attitude_change_from_first_deg=float(excursion.max()),
                covariance_diag_max=np.max(cov,axis=0).tolist(),
                invalid_values=int(np.sum(~np.isfinite(np.c_[p,q,cov]))))
    required = {'left','right','imu','status','odometry'}
    checks = {'all_streams': required <= streams.keys()}
    for name in required:
        s=streams.get(name, {})
        checks[name+'_timing'] = (s.get('hz',0) >= (.9*expected_imu_hz if name=='imu' else 20)
            and s.get('max_silence_sec', math.inf) <= .2 and s.get('nonmonotonic',1)==0
            and (s.get('stamp_interval_ms') or {}).get('max',math.inf) <= 200)
    status=streams.get('status',{})
    checks['tracking_success'] = status.get('vo_states',{}) == {'1': status.get('count',0)}
    odom=streams.get('odometry',{})
    checks['finite_pose'] = odom.get('invalid_values',1)==0
    checks['no_position_jump'] = (odom.get('step_m') or {}).get('max',math.inf) <= .1
    checks['no_rotation_jump'] = (odom.get('angular_step_deg') or {}).get('max',math.inf) <= 5
    checks['static_displacement_within_10cm'] = odom.get('max_displacement_from_first_m',math.inf) <= .1
    checks['static_attitude_change_within_1deg'] = odom.get('max_attitude_change_from_first_deg',math.inf) <= 1
    return dict(streams=streams, checks=checks, passed=all(checks.values()),
        scope='Observed raw VIO continuity; static displacement assumes a stationary camera. No motion accuracy or flight acceptance.')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--duration',type=float,default=180)
    parser.add_argument('--warmup',type=float,default=20)
    parser.add_argument('--expected-imu-hz',type=float,default=200)
    parser.add_argument('--launch-arg',action='append',default=[])
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '97':
        parser.error('Run in isolated ROS_DOMAIN_ID=97')
    args.out.mkdir(parents=True,exist_ok=False)
    rclpy.init(); node=rclpy.create_node('oakd_hardware_observer')
    discovery_deadline=time.monotonic()+2
    while time.monotonic()<discovery_deadline:rclpy.spin_once(node,timeout_sec=.1)
    occupied=[n for n,_ in node.get_node_names_and_namespaces()
        if n!='oakd_hardware_observer' and not n.startswith('_ros2cli_daemon')]
    if occupied:
        raise RuntimeError('ROS domain is occupied')
    rows=[]; info={}; saved=set(); origin=time.monotonic(); subscriptions=[]
    def receive(name,msg):
        now=time.monotonic(); stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        row=dict(stream=name,elapsed=now-origin,stamp_ns=stamp,
            age_ms=(time.time_ns()-stamp)/1e6,frame_id=msg.header.frame_id)
        if name=='odometry':
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            row.update(position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],
                covariance_diag=[msg.pose.covariance[i] for i in (0,7,14,21,28,35)])
        elif name=='status':
            row.update(vo_state=msg.vo_state,callback_sec=msg.node_callback_execution_time,
                track_sec=msg.track_execution_time)
        elif name=='imu':
            a=msg.linear_acceleration;g=msg.angular_velocity
            row.update(accel=[a.x,a.y,a.z],gyro=[g.x,g.y,g.z])
        elif name in ('left','right'):
            row.update(width=msg.width,height=msg.height,encoding=msg.encoding)
            if name not in saved and now-origin > args.warmup:
                frame=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width]
                save_mono_png(args.out/(name+'.png'),frame);saved.add(name)
        rows.append(row)
    for name,topic,typ in [('left','/oakd/left/image_raw',Image),('right','/oakd/right/image_raw',Image),
        ('imu','/oakd/imu/raw',Imu),('odometry','/visual_slam/tracking/odometry',Odometry),
        ('status','/visual_slam/status',VisualSlamStatus)]:
        subscriptions.append(node.create_subscription(typ,topic,lambda m,n=name:receive(n,m),qos_profile_sensor_data))
    def camera_info(name,msg):
        info[name]=dict(frame_id=msg.header.frame_id,width=msg.width,height=msg.height,
            k=list(msg.k),p=list(msg.p),r=list(msg.r),d=list(msg.d),distortion_model=msg.distortion_model)
    for side in ('left','right'):
        subscriptions.append(node.create_subscription(CameraInfo,f'/oakd/{side}/camera_info',
            lambda m,s=side:camera_info(s,m),qos_profile_sensor_data))
    transforms={}
    def tf(msg):
        for t in msg.transforms:
            p=t.transform.translation;q=t.transform.rotation
            transforms[t.child_frame_id]=dict(parent=t.header.frame_id,translation=[p.x,p.y,p.z],rotation=[q.x,q.y,q.z,q.w])
    subscriptions.append(node.create_subscription(TFMessage,'/tf_static',tf,
        QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    command=['ros2','launch','uav_bringup','oakd_vio.launch.py',
        'launch_odom_guard:=false','launch_robot_localization:=false',
        'oakd_enable_depth_publish:=false','oakd_enable_pointcloud_publish:=false',*args.launch_arg]
    process=None; snapshots=False; result={}; last_progress=0; param_jobs=[]
    try:
        with (args.out/'launch.log').open('w') as log:
            process=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            print('Hardware capture started:', args.out,flush=True)
            while time.monotonic()-origin < args.duration+args.warmup and process.poll() is None:
                rclpy.spin_once(node,timeout_sec=.05)
                elapsed=time.monotonic()-origin
                if not snapshots and elapsed >= args.warmup:
                    for target in ('/oakd_unified','/visual_slam_node'):
                        destination=(args.out/(target.strip('/')+'-params.yaml')).open('w')
                        param_jobs.append((subprocess.Popen(['ros2','param','dump',target],
                            stdout=destination,stderr=subprocess.STDOUT),destination))
                    snapshots=True
                    print('Measurement window started; keep camera stationary.',flush=True)
                if elapsed-last_progress > 30:
                    print('Elapsed',round(elapsed),'samples',dict(Counter(r['stream'] for r in rows)),flush=True)
                    last_progress=elapsed
            end=time.monotonic()-origin
            result=summarize(rows,args.warmup,args.warmup+args.duration,args.expected_imu_hz)
            fmu=[topic for topic,_ in node.get_topic_names_and_types() if topic.startswith('/fmu/in/')
                and node.get_publishers_info_by_topic(topic)]
            result.update(command=command,domain=97,duration_sec=args.duration,warmup_sec=args.warmup,
                actual_elapsed_sec=end,camera_info=info,static_tf=transforms,fmu_input_publishers=fmu,
                thresholds=dict(image_and_pose_min_hz=20,imu_min_hz=.9*args.expected_imu_hz,max_gap_sec=.2,
                    max_position_step_m=.1,max_rotation_step_deg=5,max_static_displacement_m=.1,
                    max_static_attitude_change_deg=1),
                nodes=node.get_node_names_and_namespaces())
            result['checks']['capture_completed']=end>=args.warmup+args.duration
            result['checks']['no_fmu_publishers']=not fmu
            result['publisher_counts']={topic:len(node.get_publishers_info_by_topic(topic))
                for topic in ('/oakd/left/image_raw','/oakd/right/image_raw','/oakd/imu/raw',
                              '/visual_slam/tracking/odometry','/visual_slam/status')}
            result['checks']['single_publisher_per_stream']=all(v==1 for v in result['publisher_counts'].values())
            result['stereo_stamp_pairing']=dict(
                left_only=len({r['stamp_ns'] for r in rows if r['stream']=='left'}
                    -{r['stamp_ns'] for r in rows if r['stream']=='right'}),
                right_only=len({r['stamp_ns'] for r in rows if r['stream']=='right'}
                    -{r['stamp_ns'] for r in rows if r['stream']=='left'}),
                scope='Published timestamps; the driver averages physical left/right timestamps.')
    except BaseException as exc:
        result.update(error=f'{type(exc).__name__}: {exc}',checks={'capture_completed':False})
        raise
    finally:
        for job,destination in param_jobs:
            try:job.wait(timeout=5)
            except subprocess.TimeoutExpired:job.kill();job.wait()
            destination.close()
        if process is not None:
            if process.poll() is None:process.send_signal(signal.SIGINT)
            try:process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
            # A crashed launcher must not leave a camera owner or duplicate SDK.
            try:os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:pass
            result['launch_exit_code']=process.returncode
        result['passed']=all(result.get('checks',{'incomplete':False}).values())
        result['cleanup_confirmed']=process is None or process.poll() is not None
        with gzip.open(args.out/'samples.jsonl.gz','wt') as f:
            for row in rows:f.write(json.dumps(row,allow_nan=False)+'\n')
        (args.out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        node.destroy_node();rclpy.shutdown()
    print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main())
