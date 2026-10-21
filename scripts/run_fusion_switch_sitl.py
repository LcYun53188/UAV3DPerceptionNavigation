#!/usr/bin/env python3
"""Disarmed synthetic EV + rendered flow/range + actual PX4 selection audit.

No camera VIO accuracy or motor flight claim. A fixed EV fixture is independent
of PX4 estimates and Gazebo truth. No flight-command/setpoint publishers exist.
"""
import argparse
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from sim_validation import ROOT,read,file_hash,write_json
from prepare_px4_sim import ensure_source,check_external
from build_px4_vio import BUILD,verify
from prepare_mtf01_assets import assets
from run_px4_sitl_smoke import free_port,stop
from vio_render_device import environment


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ui',action='store_true')
    parser.add_argument('--duration',type=float,default=90.)
    args=parser.parse_args()
    if not 45<=args.duration<=180:parser.error('duration must be 45..180 s')
    lock=read(ROOT/'simulation/px4/versions.lock.yaml')
    for name in ('sitl','agent','px4_msgs'):ensure_source(lock[name])
    check_external(lock);build_receipt=verify(lock)
    mutex_path=ROOT/'.cache/simulation/px4-smoke.lock'
    mutex_path.parent.mkdir(parents=True,exist_ok=True)
    with mutex_path.open('a') as mutex:
        fcntl.flock(mutex,fcntl.LOCK_EX|fcntl.LOCK_NB)
        for port in (8898,18577,14587,14550):free_port(port)
        if Path('/tmp/px4_lock-7').exists():raise RuntimeError('PX4 instance 7 occupied')
        return execute(args,lock,build_receipt)


def execute(args,lock,build_receipt):
    ident=str(uuid.uuid4());out=ROOT/'.cache/simulation/fusion-switch'/ident
    (out/'rootfs').mkdir(parents=True)
    px4=ROOT/lock['sitl']['path'];models=px4/'Tools/simulation/gz/models'
    profile=assets(out/'assets',px4/'Tools/simulation/gz/worlds/default.sdf')
    fusion=read(ROOT/'simulation/px4/mtf01/joint_fusion.json')
    env=dict(os.environ,ROS_DOMAIN_ID='78',ROS_LOCALHOST_ONLY='1',GZ_PARTITION='uav_fusion_switch_'+ident,
        GZ_IP='127.0.0.1',GZ_DISTRO='harmonic',HEADLESS='1',PX4_SIM_MODEL='gz_'+profile['model'],
        PX4_SYS_AUTOSTART='4001',PX4_GZ_STANDALONE='1',PX4_GZ_WORLD='default',PX4_UXRCE_DDS_PORT='8898',
        PX4_GZ_MODELS=str(out/'assets'),PX4_GZ_WORLDS=str(out/'assets'),
        GZ_SIM_RESOURCE_PATH=str(out/'assets')+':'+str(models),
        GZ_SIM_SYSTEM_PLUGIN_PATH=str(BUILD/'src/modules/simulation/gz_plugins'),
        GZ_SIM_SERVER_CONFIG_PATH=str(ROOT/'simulation/px4/server_control.config'))
    env.update({'PX4_PARAM_'+name:str(value) for name,value in fusion['parameters'].items()})
    from simulation.policy import no_gnss_environment
    localization_policy = no_gnss_environment(env, out/'assets/default.sdf')
    os.environ.update({name:env[name] for name in ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','GZ_PARTITION')})
    sources=[ROOT/name for name in ('scripts/run_fusion_switch_sitl.py','scripts/localization_selection_session.py',
        'scripts/px4_vision_audit.py','scripts/mtf01_flow_bridge.py','scripts/mtf01_range_bridge.py',
        'src/uav_mission/uav_mission/localization_selector.py','src/uav_mission/uav_mission/sitl_fusion_parameters.py',
        'src/uav_mission/uav_mission/vio_gate.py','simulation/px4/mtf01/joint_fusion.json')]
    from simulation.backends import source_hashes
    sources.append(ROOT/'simulation/px4/server_control.config')
    manifest=dict(launcher_sha256=source_hashes(),localization_policy=localization_policy,run_id=ident,domain=78,partition=env['GZ_PARTITION'],calibration_id='a'*64,
        synthetic_vio=True,scope='ground synthetic EV + actual rendered MTF/PX4 EKF; no flight or camera VIO qualification',
        vio_build=build_receipt,px4_parameter_overrides={k:v for k,v in env.items() if k.startswith('PX4_PARAM_')},
        input_sha256={str(p.relative_to(ROOT)):file_hash(p) for p in sources},processes={})
    write_json(out/'manifest.json',manifest)
    processes,logs=[],[]
    def launch(name,command,launch_env=env):
        log=(out/(name+'.log')).open('w');logs.append(log)
        p=subprocess.Popen(list(map(str,command)),cwd=ROOT,env=launch_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append(p);manifest['processes'][name]=dict(pid=p.pid,command=list(map(str,command)))
        write_json(out/'manifest.json',manifest)
        return p
    result=dict(passed=False,scope=manifest['scope'],run_id=ident)
    node=session=None
    try:
        launch('agent',[ROOT/'.deps/microxrce-install/bin/MicroXRCEAgent','udp4','-p','8898'])
        launch('gazebo',['gz','sim','-r','-s','--headless-rendering',out/'assets/default.sdf'],environment(env,'nvidia'))
        if args.ui:launch('gazebo_gui',['gz','sim','-g'],environment(env,'nvidia'))
        launch('sensor_bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock','/mtf01/flow/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mtf01/range/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'])
        launch('range_adapter',[sys.executable,ROOT/'scripts/mtf01_range_bridge.py','--ros-args','-p','use_sim_time:=true'])
        # Start flow only after EV yaw/position are aligned, preserving startup.
        launch('px4',[BUILD/'bin/px4','-d','-i','7','-w',out/'rootfs',BUILD/'etc'])
        config=out/'qgc-config/QGroundControl';config.mkdir(parents=True)
        (config/'QGroundControl.ini').write_text('[AutoConnect]\nautoConnectUDP=true\nautoConnectPixhawk=false\nautoConnectSiKRadio=false\nautoConnectRTKGPS=false\nautoConnectLibrePilot=false\n')
        from usb_joystick import qgc_environment
        launch('qgc',[lock['artifacts']['qgc']['path'],'--allow-multiple','--log-output'],
            qgc_environment(dict(env,XDG_CONFIG_HOME=str(out/'qgc-config'),XDG_CACHE_HOME=str(out/'qgc-cache'),
                QT_QPA_PLATFORM='xcb' if args.ui else 'offscreen')))
        import rclpy
        from rosgraph_msgs.msg import Clock
        from px4_vision_audit import VisionFusionAudit
        from localization_selection_session import SelectionSession
        from uav_nav_interfaces.srv import SelectLocalization
        from unique_identifier_msgs.msg import UUID
        rclpy.init(args=['--ros-args','-p','use_sim_time:=true'])
        node=rclpy.create_node('synthetic_ev_switch_fixture')
        clock=None
        def receive_clock(message):
            nonlocal clock
            clock=message.clock
        node.create_subscription(Clock,'/clock',receive_clock,10)
        audit=VisionFusionAudit(node,'aligned_pose_v1')
        session=SelectionSession(out)
        from rclpy.executors import SingleThreadedExecutor
        executor=SingleThreadedExecutor();executor.add_node(node);executor.add_node(session)
        client=node.create_client(SelectLocalization,'/uav/localization/select')
        until=time.monotonic()+args.duration
        flow_started=False;requests=[];confirmed=[];pending=None
        target_index=0;targets=['FLOW_HORIZONTAL','VIO_FLOW','FLOW_HORIZONTAL','VIO_FLOW']
        stable_since=None
        while time.monotonic()<until:
            executor.spin_once(timeout_sec=.005)
            audit.tick(clock)
            if any(p.poll() is not None for p in processes):raise RuntimeError('Owned simulation process exited')
            if session.samples.get('vehicle') and session.samples['vehicle'].arming_state!=1:
                raise RuntimeError('Unexpected arm; ground audit stopped')
            if (not flow_started and audit.input_count>50 and audit.last.get('flags') and
                    audit.last['flags'].cs_ev_yaw and audit.last['flags'].cs_ev_pos):
                launch('flow_adapter',['/usr/bin/python3',ROOT/'scripts/mtf01_flow_bridge.py','--ros-args','-p','use_sim_time:=true'])
                flow_started=True
            s=session.selector
            if s.phase=='FAULT':raise RuntimeError('Selector fault: '+s.fault)
            if pending is not None and pending.done():
                response=pending.result();pending=None
                requests.append(dict(profile=targets[target_index],accepted=response.accepted,reason=response.reason))
                if not response.accepted:raise RuntimeError('Switch admission rejected: '+response.reason)
            if s.ready and s.phase=='IDLE':
                if stable_since is None:stable_since=time.monotonic()
                if target_index<len(targets) and s.active_profile==targets[target_index] and s.generation==target_index+1:
                    local=session.samples['local'];flags=session.samples['flags']
                    confirmed.append(dict(profile=s.active_profile,generation=s.generation,ground=session.ground_safe(),
                        ev_pos=flags.cs_ev_pos,ev_hgt=flags.cs_ev_hgt,ev_yaw=flags.cs_ev_yaw,flow=flags.cs_opt_flow,
                        range=flags.cs_rng_hgt,resets=[local.xy_reset_counter,local.z_reset_counter,local.heading_reset_counter]))
                    target_index+=1;stable_since=time.monotonic()
                if (target_index<len(targets) and time.monotonic()-stable_since>=1. and pending is None):
                    pending=client.call_async(SelectLocalization.Request(coordinator_instance=ident,
                        request_id=UUID(uuid=list(uuid.uuid4().bytes)),profile=targets[target_index]))
                    stable_since=None
                elif target_index==len(targets) and time.monotonic()-stable_since>=2.:break
            else:stable_since=None
        controls={name:len(node.get_publishers_info_by_topic('/px4_7/fmu/in/'+name))
                  for name in ('vehicle_command','trajectory_setpoint','offboard_control_mode')}
        result.update(requests=requests,confirmed=confirmed,control_publishers=controls,
            source='fixed independent synthetic EV; rendered flow and lidar; native actual EKF aid telemetry',
            source_fault=session.selector.fault,final_profile=session.selector.active_profile,
            final_generation=session.selector.generation,arming_state=getattr(session.samples.get('vehicle'),'arming_state',None),
            landed=getattr(session.samples.get('land'),'landed',None),final_reason=session.selector.reason)
        result['passed']=(len(confirmed)==4 and all(item['ground'] and item['ev_hgt'] and item['ev_yaw'] and
            item['flow'] and item['range'] and item['ev_pos']==(item['profile']=='VIO_FLOW') for item in confirmed) and
            not any(controls.values()) and result['arming_state']==1 and result['landed'] and not result['source_fault'])
        executor.shutdown()
    except Exception as error:result['error']=str(error)
    finally:
        if session is not None:session.close();session.destroy_node()
        if node is not None:node.destroy_node();rclpy.try_shutdown()
        for p in reversed(processes):stop(p)
        for log in logs:log.close()
        write_json(out/'result.json',result)
        print('Fusion selection evidence:',out,flush=True)
        print(json.dumps(result,indent=2),flush=True)
    return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
