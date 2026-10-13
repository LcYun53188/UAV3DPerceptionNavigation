#!/usr/bin/env python3
"""Read-only hardware stereo/trajectory UI and operator-marked motion capture.

Run on domain 97. Buttons mark data phases; they never control hardware motion.
Closing this window stops only the camera/VIO/RViz processes owned by this tool.
"""
import argparse
from collections import Counter, deque
import gzip
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import tkinter as tk

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry, Path as RosPath
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from isaac_ros_visual_slam_interfaces.msg import VisualSlamStatus
from audit_oakd_vio_hardware import summarize, save_mono_png, stats
from std_msgs.msg import Float64MultiArray, String


def motion_summary(rows, start, end, baseline, final, operator_returned,
                   return_position_tolerance_m=.2, return_orientation_tolerance_deg=15):
    if not (np.isfinite(return_position_tolerance_m) and return_position_tolerance_m > 0
            and np.isfinite(return_orientation_tolerance_deg) and return_orientation_tolerance_deg > 0):
        raise ValueError('Return tolerances must be finite and positive')
    result = summarize(rows, start, end, expected_imu_hz=200)
    result['return_tolerances']=dict(position_m=return_position_tolerance_m,orientation_deg=return_orientation_tolerance_deg)
    for key in list(result['checks']):
        if key.startswith('static_'): del result['checks'][key]
    poses = [r for r in rows if r['stream']=='odometry' and start<=r['elapsed']<=end]
    initial = [r for r in poses if baseline[0]<=r['elapsed']<=baseline[1]]
    terminal = [r for r in poses if final[0]<=r['elapsed']<=final[1]] if final else []
    if poses:
        p=np.array([r['position'] for r in poses])
        result['path_length_m']=float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())
    odom=result['streams'].get('odometry',{})
    result['exercise'] = dict(translation_observed=odom.get('max_displacement_from_first_m',0)>=.2,
        rotation_observed=odom.get('max_attitude_change_from_first_deg',0)>=10)
    result['checks']['translation_exercised']=result['exercise']['translation_observed']
    result['checks']['rotation_exercised']=result['exercise']['rotation_observed']
    result['operator_confirmed_return']=operator_returned
    gravity_windows=[]
    for window in (baseline,final):
        rr=[r for r in rows if r['stream']=='imu' and window and window[0]<=r['elapsed']<=window[1]]
        if rr:
            a=np.array([r['accel'] for r in rr]);gravity_windows.append(dict(mean=a.mean(axis=0).tolist(),std=a.std(axis=0).tolist(),samples=len(a)))
    if len(gravity_windows)==2:
        a,b=[np.array(w['mean']) for w in gravity_windows]
        angle=float(np.degrees(np.arccos(np.clip(a@b/np.linalg.norm(a)/np.linalg.norm(b),-1,1))))
        result['gravity_reference']=dict(initial=gravity_windows[0],final=gravity_windows[1],direction_change_deg=angle,
            scope='Stationary accelerometer tilt cross-check. Gravity cannot validate yaw or position.')
        result['checks']['return_tilt_reference_consistent']=angle<=5
    if initial and terminal and operator_returned:
        def mean_pose(rr):
            p=np.mean([r['position'] for r in rr],axis=0)
            q=np.array([r['quaternion'] for r in rr]);q=q/np.linalg.norm(q,axis=1)[:,None]
            q[q@q[0]<0]*=-1;qm=q.mean(axis=0);qm/=np.linalg.norm(qm)
            return p,qm
        p0,q0=mean_pose(initial);p1,q1=mean_pose(terminal)
        distance=float(np.linalg.norm(p1-p0))
        angle=float(np.degrees(2*np.arccos(np.clip(abs(q0@q1),0,1))))
        result['return_error']=dict(position_m=distance,orientation_deg=angle,
            initial_samples=len(initial),final_samples=len(terminal),initial_mean_m=p0.tolist(),final_mean_m=p1.tolist(),
            scope='Operator placement repeatability and VIO drift combined; no external ground truth.')
        result['checks']['return_position_within_tolerance']=distance<=return_position_tolerance_m
        result['checks']['return_orientation_within_tolerance']=angle<=return_orientation_tolerance_deg
    else:
        result['checks']['return_observed']=False
    result['passed']=all(result['checks'].values())
    result['scope']='Slow handheld motion continuity and operator-confirmed return; no absolute accuracy or flight acceptance.'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--return-position-tolerance-m',type=float,default=.2)
    parser.add_argument('--return-orientation-tolerance-deg',type=float,default=15)
    parser.add_argument('--navigation',action='store_true',help='Compare navigation twist and pose-only EKF; raw VIO remains visible.')
    parser.add_argument('--compare-vo',action='store_true',help='Run stereo VO on the same images to isolate IMU fusion faults.')
    parser.add_argument('--launch-arg',action='append',default=[])
    args=parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID')!='97':parser.error('Use isolated ROS_DOMAIN_ID=97')
    args.out.mkdir(parents=True,exist_ok=False)
    rclpy.init(args=[]);node=rclpy.create_node('oakd_motion_readonly_ui')
    until=time.monotonic()+2
    while time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.1)
    occupied=[n for n,_ in node.get_node_names_and_namespaces() if n!='oakd_motion_readonly_ui' and not n.startswith('_ros2cli_daemon')]
    if occupied:raise RuntimeError(f'Hardware domain already occupied: {occupied}')
    origin=time.monotonic();rows=[];latest={};images={};first_pose=None;phase='initializing';baseline=None;final=None;result=None
    lock=threading.Lock();stopping=threading.Event();subscriptions=[];trajectory=[]
    capture=gzip.open(args.out/'samples.jsonl.gz','wt');last_flush=time.monotonic()
    diagnostic_capture=gzip.open(args.out/'sensor-diagnostics.jsonl.gz','wt');diagnostics=[]
    recent_accel=deque(maxlen=200);gravity_reference=None;guard_text=''
    path_pub=node.create_publisher(RosPath,'/oakd_motion/observed_path',10)
    path=RosPath();path.header.frame_id='odom'
    def receive(name,msg):
        nonlocal first_pose,last_flush
        elapsed=time.monotonic()-origin;stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        row=dict(stream=name,elapsed=elapsed,stamp_ns=stamp,age_ms=(time.time_ns()-stamp)/1e6,frame_id=msg.header.frame_id)
        if name in ('odometry','navigation','guarded','filtered','vo_reference'):
            p=msg.pose.pose.position;q=msg.pose.pose.orientation
            row.update(position=[p.x,p.y,p.z],quaternion=[q.x,q.y,q.z,q.w],covariance_diag=[msg.pose.covariance[i] for i in (0,7,14,21,28,35)])
            if name=='odometry' and first_pose is None:first_pose=elapsed
            if name=='odometry' and (len(path.poses)==0 or stamp-(path.poses[-1].header.stamp.sec*10**9+path.poses[-1].header.stamp.nanosec)>=100_000_000):
                pose=PoseStamped();pose.header=msg.header;pose.pose=msg.pose.pose;path.header=msg.header
                path.poses.append(pose);path.poses=path.poses[-10000:];path_pub.publish(path)
                trajectory.append(row['position']);trajectory[:]=trajectory[-10000:]
            v=msg.twist.twist.linear;w=msg.twist.twist.angular
            row.update(linear_velocity=[float(x) if np.isfinite(x) else None for x in (v.x,v.y,v.z)],
                       angular_velocity=[float(x) if np.isfinite(x) else None for x in (w.x,w.y,w.z)])
        elif name=='status':row.update(vo_state=msg.vo_state,callback_sec=msg.node_callback_execution_time)
        elif name=='imu':
            a=msg.linear_acceleration;g=msg.angular_velocity;row.update(accel=[a.x,a.y,a.z],gyro=[g.x,g.y,g.z]);recent_accel.append(row['accel'])
        elif name=='gravity':
            row['gravity']=[[p.x,p.y,p.z] for p in msg.points]
        else:
            row.update(width=msg.width,height=msg.height,encoding=msg.encoding)
            images[name]=msg
        with lock:
            latest[name]=row
            if result is None:
                rows.append(row);capture.write(json.dumps(row,allow_nan=False)+'\n')
                if time.monotonic()-last_flush>2:capture.flush();last_flush=time.monotonic()
    for name,topic,typ in [('left','/oakd/left/image_raw',Image),('right','/oakd/right/image_raw',Image),('imu','/oakd/imu/raw',Imu),('odometry','/visual_slam/tracking/odometry',Odometry),('status','/visual_slam/status',VisualSlamStatus)]:
        subscriptions.append(node.create_subscription(typ,topic,lambda m,n=name:receive(n,m),qos_profile_sensor_data))
    if args.navigation:
        for name,topic in [('navigation','/navigation/odometry'),('guarded','/visual_slam/guarded_odometry'),('filtered','/odometry/filtered')]:
            subscriptions.append(node.create_subscription(Odometry,topic,lambda m,n=name:receive(n,m),qos_profile_sensor_data))
    if args.compare_vo:
        subscriptions.append(node.create_subscription(Odometry,'/vo_reference/visual_slam/tracking/odometry',lambda m:receive('vo_reference',m),qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(Marker,'/visual_slam/vis/gravity',lambda m:receive('gravity',m),10))
    def diagnostic(name,msg):
        row=dict(stream=name,elapsed=time.monotonic()-origin,values=list(msg.data))
        with lock:
            if result is None:diagnostics.append(row);diagnostic_capture.write(json.dumps(row)+'\n')
    for name,topic in [('imu_timing','/oakd/imu/timing'),('exposure','/oakd/stereo/exposure')]:
        subscriptions.append(node.create_subscription(Float64MultiArray,topic,lambda m,n=name:diagnostic(n,m),qos_profile_sensor_data))
    def guard_status(msg):
        nonlocal guard_text
        guard_text=msg.data
        with lock:
            if result is None:
                row=dict(stream='guard_status',elapsed=time.monotonic()-origin,text=msg.data)
                diagnostics.append(row);diagnostic_capture.write(json.dumps(row)+'\n')
    if args.navigation:subscriptions.append(node.create_subscription(String,'/visual_slam/odom_guard/status',guard_status,10))
    def spin():
        while not stopping.is_set():rclpy.spin_once(node,timeout_sec=.05)
    thread=threading.Thread(target=spin);thread.start()
    processes=[];logs=[]
    def launch(command,filename):
        log=(args.out/filename).open('w');logs.append(log)
        p=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,start_new_session=True);processes.append(p);return p
    command=['ros2','launch','uav_bringup','oakd_vio.launch.py','launch_odom_guard:=false','launch_robot_localization:=false','oakd_enable_depth_publish:=false','oakd_enable_pointcloud_publish:=false']
    if args.navigation:
        command=[c for c in command if c not in ('launch_odom_guard:=false','launch_robot_localization:=false')]
        command+=['launch_odom_guard:=true','launch_robot_localization:=true','launch_navigation_odometry:=true',
                  'odom_guard_publish_rejected_as_hold:=false','odom_guard_max_hold_sec_before_reseed:=0.0']
    command+=args.launch_arg
    camera=launch(command,'launch.log')
    # Reuse the RViz layout, hide unavailable guarded/filtered streams, show observed raw path.
    import yaml
    if args.compare_vo:
        from ament_index_python.packages import get_package_share_directory
        params=yaml.safe_load((Path(get_package_share_directory('uav_bringup'))/'config/isaac_visual_slam_oakd_vio_only.yaml').read_text())['visual_slam']['ros__parameters']
        params.update(tracking_mode=0,publish_map_to_odom_tf=False,publish_odom_to_base_tf=False)
        reference_launch=args.out/'vo_reference.launch.py'
        reference_launch.write_text('from launch import LaunchDescription\nfrom launch_ros.actions import ComposableNodeContainer\nfrom launch_ros.descriptions import ComposableNode\ndef generate_launch_description():\n    return LaunchDescription([ComposableNodeContainer(name="vo_reference_container", namespace="vo_reference", package="rclcpp_components", executable="component_container", output="screen", composable_node_descriptions=[ComposableNode(name="visual_slam_node", namespace="vo_reference", package="isaac_ros_visual_slam", plugin="nvidia::isaac_ros::visual_slam::VisualSlamNode", parameters=['+repr(params)+'], remappings=[("visual_slam/image_0","/oakd/left/image_raw"),("visual_slam/image_1","/oakd/right/image_raw"),("visual_slam/camera_info_0","/oakd/left/camera_info"),("visual_slam/camera_info_1","/oakd/right/camera_info"),("tf","/tf"),("tf_static","/tf_static")])])])\n')
        launch(['ros2','launch',str(reference_launch.resolve())],'vo-reference.log')
    config=yaml.safe_load((Path(__file__).resolve().parents[1]/'src/uav_bringup/rviz/visual_slam_check.rviz').read_text())
    group=config['Visualization Manager']['Displays'][2]
    for display in group['Displays']:
        if not args.navigation and display['Name'] in ('Guarded Odometry','Filtered Odometry','Guarded Path'):display['Enabled']=False;display['Value']=False
        if display['Name']=='VO Path':
            display['Enabled']=True;display['Value']=True;display['Topic']['Value']='/oakd_motion/observed_path'
    config['Visualization Manager']['Views']['Current']['Target Frame']='odom'
    config_path=args.out/'motion.rviz';config_path.write_text(yaml.safe_dump(config,sort_keys=False))
    root=tk.Tk();root.title('OAK-D Pro W | Hardware VIO motion test | Read only');root.geometry('980x780+900+80')
    panels={s:tk.Label(root,text=s.upper()) for s in ('left','right')}
    for i,panel in enumerate(panels.values()):panel.grid(row=0,column=i)
    label=tk.Label(root,justify='left',font=('monospace',11),wraplength=960);label.grid(row=1,column=0,columnspan=2,sticky='w')
    canvas=tk.Canvas(root,width=960,height=270,bg='#20242c');canvas.grid(row=2,column=0,columnspan=2)
    buttons=tk.Frame(root);buttons.grid(row=3,column=0,columnspan=2)
    def set_phase(value):
        nonlocal phase
        phase=value;print('PHASE',phase,round(time.monotonic()-origin,2),flush=True)
    def start_motion():
        if phase=='ready':set_phase('moving')
    def returned():
        nonlocal final
        if phase=='moving':
            t=time.monotonic()-origin;final=(t,t+10);set_phase('final_rest')
    start_button=tk.Button(buttons,text='Start motion / 开始移动',command=start_motion,state='disabled');start_button.pack(side='left',padx=10)
    return_button=tk.Button(buttons,text='Returned to original pose / 已放回原位',command=returned,state='disabled');return_button.pack(side='left',padx=10)
    def finish():
        nonlocal result
        end=time.monotonic()-origin
        with lock:
            result=motion_summary(list(rows),baseline[0],end,baseline,final,True,
                args.return_position_tolerance_m,args.return_orientation_tolerance_deg)
            capture.close();diagnostic_capture.close()
        measurement_diagnostics=[r for r in diagnostics if baseline[0]<=r['elapsed']<=end]
        timing=[r['values'] for r in measurement_diagnostics if r['stream']=='imu_timing']
        exposure=[r['values'] for r in measurement_diagnostics if r['stream']=='exposure']
        result['sensor_diagnostics']=dict(imu_accel_minus_gyro_ms=stats([(v[0]-v[1])*1000 for v in timing]),
            left_exposure_us=stats([v[0] for v in exposure]),right_exposure_us=stats([v[1] for v in exposure]),
            left_iso=stats([v[2] for v in exposure]),right_iso=stats([v[3] for v in exposure]),
            physical_stereo_offset_ms=stats([v[4] for v in exposure]),timing_samples=len(timing),exposure_samples=len(exposure))
        if args.navigation:
            result['navigation_comparison']={}
            for name in ('navigation','guarded','filtered'):
                ss=result['streams'].get(name,{})
                result['checks'][name+'_available']=ss.get('count',0)>0 and ss.get('nonmonotonic',1)==0 and ss.get('max_silence_sec',float('inf'))<=.25
                result['navigation_comparison'][name]=dict(count=ss.get('count',0),hz=ss.get('hz',0),guard_status=guard_text if name=='guarded' else None)
            faults=[r for r in measurement_diagnostics if r['stream']=='guard_status' and r['text'].startswith(('reject','latched','reseed'))]
            result['guard_faults']=dict(count=len(faults),first=faults[0] if faults else None)
            result['checks']['navigation_guard_no_faults']=not faults
        topics=['/oakd/left/image_raw','/oakd/right/image_raw','/oakd/imu/raw','/visual_slam/tracking/odometry','/visual_slam/status']
        result.update(domain=97,measurement_start=baseline[0],measurement_end=end,baseline_window=baseline,final_window=final,command=command)
        result['publisher_counts']={t:len(node.get_publishers_info_by_topic(t)) for t in topics}
        fmu=[t for t,_ in node.get_topic_names_and_types() if t.startswith('/fmu/in/') and node.get_publishers_info_by_topic(t)]
        result['fmu_input_publishers']=fmu
        result['checks']['no_fmu_publishers']=not fmu
        result['checks']['single_publisher_per_stream']=all(v==1 for v in result['publisher_counts'].values())
        result['passed']=all(result['checks'].values())
        (args.out/'result.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        for side,msg in images.items():
            frame=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width];save_mono_png(args.out/(side+'.png'),frame)
        set_phase('completed');start_button.configure(state='disabled');return_button.configure(state='disabled')
        print('RESULT', 'PASS' if result['passed'] else 'FAIL',flush=True)
    rviz_started=False;last_state=0;last_image_evidence=-1.;image_evidence_count=0
    def tick():
        nonlocal baseline,rviz_started,last_state,gravity_reference,last_image_evidence,image_evidence_count
        now=time.monotonic()-origin
        if (args.out/'close-ui').exists():root.destroy();return
        if camera.poll() is not None:
            print('Hardware launcher exited:',camera.returncode,flush=True);root.destroy();return
        if first_pose is not None and not rviz_started:
            launch(['rviz2','-d',str(config_path)],'rviz.log');rviz_started=True
        if phase=='initializing' and first_pose is not None and now>=first_pose+20:
            baseline=(now,now+10);set_phase('baseline')
        if phase=='baseline' and now>=baseline[1]:
            reference_rows=[r['accel'] for r in rows if r['stream']=='imu' and baseline[0]<=r['elapsed']<=baseline[1]]
            if reference_rows:gravity_reference=np.mean(reference_rows,axis=0)
            for side,msg in images.items():
                frame=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width];save_mono_png(args.out/('initial-'+side+'.png'),frame)
            set_phase('ready');start_button.configure(state='normal')
        if phase=='ready' and (args.out/'start-motion').exists():start_motion()
        if phase=='moving' and (args.out/'returned').exists():returned()
        if phase=='moving':return_button.configure(state='normal')
        if phase=='final_rest' and now>=final[1]:finish()
        if args.compare_vo and phase=='moving' and now-last_image_evidence>=1 and image_evidence_count<180:
            evidence=args.out/'motion-images';evidence.mkdir(exist_ok=True)
            for side,msg in list(images.items()):
                frame=np.frombuffer(msg.data,dtype=np.uint8).reshape(msg.height,msg.step)[:,:msg.width]
                save_mono_png(evidence/(f'{now:.3f}-'+side+'.png'),frame)
            last_image_evidence=now;image_evidence_count+=1
        for side,msg in list(images.items()):
            if msg.encoding=='mono8':
                data=bytes(msg.data);data=b''.join(data[i*msg.step:i*msg.step+msg.width] for i in range(msg.height))
                photo=tk.PhotoImage(data=f'P5\n{msg.width} {msg.height}\n255\n'.encode()+data,format='PPM').subsample(2)
                panels[side].configure(image=photo);panels[side].photo=photo
        pose=latest.get('odometry',{});status=latest.get('status',{})
        hints={'initializing':'Keep camera still: initializing (20 s after first pose).','baseline':'Keep still: recording initial pose (10 s).','ready':'Ready. Press Start motion; translate slowly >20 cm and rotate >10 degrees.','moving':'Move slowly, then place at original position AND orientation and press Returned.','final_rest':'Keep still: recording returned pose (10 s).','completed':'Capture complete. Live UI stays open; close this window to stop this hardware session.'}
        outcome=('PASS' if result['passed'] else 'FAIL') if result else 'PENDING'
        tilt='';nav=''
        if gravity_reference is not None and recent_accel:
            a=np.mean(list(recent_accel),axis=0);b=gravity_reference
            angle=np.degrees(np.arccos(np.clip(a@b/np.linalg.norm(a)/np.linalg.norm(b),-1,1)))
            tilt=f'Initial tilt difference: {angle:.1f} deg (does not check yaw/position)\n'
        if args.navigation:
            v=latest.get('navigation',{}).get('linear_velocity',[0,0,0]);nav=f'Navigation body speed (m/s): {np.round(v,3)}\nGuard: {guard_text}\n'
        label.configure(text=f"Phase: {phase} | Result: {outcome} | Elapsed: {now:.1f}s\nVO state: {status.get('vo_state')} | Pose age: {now-pose.get('elapsed',0):.3f}s\nPosition (m): {np.round(pose.get('position',[0,0,0]),4)}\nQuaternion xyzw: {np.round(pose.get('quaternion',[0,0,0,1]),4)}\n{tilt}{nav}{hints[phase]}\n{args.out}")
        canvas.delete('all');points=np.array(trajectory[-10000:])
        if len(points)>1:
            for offset,axes,title in [(0,(0,1),'XY'),(480,(0,2),'XZ')]:
                selected=points[:,axes];low=selected.min(axis=0);high=selected.max(axis=0)
                span=max(float((high-low).max()),.5);center=(high+low)/2
                projected=(selected-center)*220/span;projected[:,1]*=-1;projected+=np.array([offset+240,145])
                canvas.create_text(offset+50,20,text=f'{title} / span {span:.2f} m',fill='white',anchor='w')
                canvas.create_line(*projected.flatten().tolist(),fill='#39bfff',width=2)
                x,y=projected[-1];canvas.create_oval(x-4,y-4,x+4,y+4,fill='#ffb347')
        if now-last_state>=1:
            (args.out/'session.json').write_text(json.dumps(dict(phase=phase,elapsed=now,first_pose=first_pose,baseline=baseline,final=final,latest=latest,counts=dict(Counter(r['stream'] for r in rows)),pid=os.getpid(),child_pids=[p.pid for p in processes]),indent=2)+'\n');last_state=now
        root.after(200,tick)
    try:
        tick();root.mainloop()
    finally:
        stopping.set();thread.join(timeout=3);capture.close();diagnostic_capture.close()
        for p in processes:
            if p.poll() is None:p.send_signal(signal.SIGINT)
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
            try:os.killpg(p.pid,signal.SIGKILL)
            except ProcessLookupError:pass
        for log in logs:log.close()
        (args.out/'closed.json').write_text(json.dumps(dict(elapsed=time.monotonic()-origin,capture_completed=result is not None,exit_codes=[p.returncode for p in processes]))+'\n')
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
