"""Disarmed joint observation only: existing VIO contract plus independent MTF aids.

This module cannot arm, publish control setpoints, or authorize a mission. The
production VioGate continues to reject optical-flow/range aiding by default.
"""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import json
import math
from pathlib import Path
import subprocess
import threading
import time
from std_msgs.msg import String
from px4_msgs.msg import SensorOpticalFlow,DistanceSensor
from rclpy.qos import qos_profile_sensor_data
from uav_mission.vio_gate import VioGate
from px4_vio_real_fusion import RealPoseFusionAudit
from mtf01_audit import fields,assessment

NATIVE_TOPICS=('sensor_optical_flow','vehicle_optical_flow','distance_sensor',
    'estimator_aid_src_optical_flow','estimator_aid_src_rng_hgt','estimator_status_flags',
    'vehicle_local_position','vehicle_status')
INPUT_TOPICS={'flow':'/px4_7/fmu/in/sensor_optical_flow','range':'/px4_7/fmu/in/distance_sensor'}


def parameter_receipts(build,env,names):
    return {name:subprocess.check_output([str(build/'bin/px4-param'),'--instance','7','show',name],
        env=env,text=True,timeout=3) for name in names}


class VioWithMtfObserverGate(VioGate):
    """Explicit observer policy; never selected by a FlightServer profile."""
    forbidden_pose_aids=tuple(n for n in VioGate.forbidden_pose_aids if n not in ('cs_opt_flow','cs_rng_hgt'))


from uav_mission.mtf_inputs import mtf_source_health


def observation_state(vio_ready, mtf_ready):
    if vio_ready and mtf_ready:return 'JOINT_OBSERVATIONS_READY'
    if vio_ready:return 'VIO_OBSERVATIONS_ONLY'
    if mtf_ready:return 'MTF_OBSERVATIONS_ONLY'
    return 'UNAVAILABLE'


class NativeAidRecorder:
    """CLI reads run in a worker, away from 250 Hz ROS/VIO callbacks."""
    def __init__(self,build,out,env,*,extra_topics=()):
        self.build,self.out,self.env=build,out,env
        self.topics=NATIVE_TOPICS+tuple(extra_topics)
        self.records=[];self.errors=[]
        self.phase='healthy'
        self.finish=threading.Event()
        self.thread=threading.Thread(target=self.run,name='joint-native-aids',daemon=False)
        self.thread.start()

    def run(self):
        while not self.finish.is_set():
            phase=self.phase
            receipt=dict(phase=phase,mono=time.monotonic(),topics={})
            try:
                for topic in self.topics:
                    if self.finish.is_set():break
                    command=[str(self.build/'bin/px4-listener'),'--instance','7',topic,'-n','1']
                    run=subprocess.run(command,env=self.env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=3)
                    with (self.out/('joint-'+topic+'.log')).open('a') as log:log.write(run.stdout+'\n')
                    receipt['topics'][topic]=fields(run.stdout)
                if len(receipt['topics'])==len(self.topics) and phase==self.phase:
                    self.records.append(receipt)
            except Exception as exc:self.errors.append(str(exc))
            self.finish.wait(1.)

    def stop(self):
        self.finish.set();self.thread.join(timeout=5)
        if self.thread.is_alive():raise RuntimeError('Joint telemetry worker did not stop')


class JointPoseFusionAudit(RealPoseFusionAudit):
    gate_class=VioWithMtfObserverGate

    def __init__(self,*args,native,profile,**kwargs):
        super().__init__(*args,**kwargs)
        self.native,self.mtf_profile=native,profile
        self.mtf_messages={};self.mtf_received={};self.mtf_counts={'flow':0,'range':0}
        self.joint_history=[];self.last_joint_record=0.
        self.joint_ready_since=None;self.longest_joint_ready_s=0.
        self.mtf_stop=None;self.healthy_snapshot=None
        self.mtf_writer_violations=[]
        self.status_output=self.node.create_publisher(String,'/uav/vio/joint_observation_status',10)
        for key,kind in (('flow',SensorOpticalFlow),('range',DistanceSensor)):
            def receiver(key):
                def callback(m):
                    self.mtf_messages[key]=m;self.mtf_received[key]=time.monotonic();self.mtf_counts[key]+=1
                return callback
            self.node.create_subscription(kind,INPUT_TOPICS[key],receiver(key),qos_profile_sensor_data)

    def mtf_health(self):
        writers={k:len(self.node.get_publishers_info_by_topic(t)) for k,t in INPUT_TOPICS.items()}
        if any(n>1 for n in writers.values()) and not self.mtf_writer_violations:
            self.mtf_writer_violations.append(dict(mono=time.monotonic(),writers=writers))
        return mtf_source_health(self.mtf_messages,self.mtf_received,writers,time.monotonic(),self.ros())

    def tick(self):
        super().tick()
        now=time.monotonic();mtf_ready,reason=self.mtf_health()
        vio_ready=self.gate.reason=='READY' and not self.stream.fault
        state=observation_state(vio_ready,mtf_ready)
        if vio_ready and mtf_ready:
            if self.joint_ready_since is None:self.joint_ready_since=now
            self.longest_joint_ready_s=max(self.longest_joint_ready_s,now-self.joint_ready_since)
        else:self.joint_ready_since=None
        if now-self.last_joint_record>=.1:
            record=dict(mono=now,ros=self.ros(),state=state,vio_reason=self.gate.reason,
                mtf_reason=reason,phase=self.native.phase,navigation_authorized=False,
                control_authorized=False,scope='disarmed observer only')
            self.joint_history.append(record);self.last_joint_record=now
            self.status_output.publish(String(data=json.dumps(record,separators=(',',':'))))

    def freeze_healthy(self):
        self.healthy_snapshot=dict(vio_ready=self.gate.reason=='READY',mtf_ready=self.mtf_health()[0],
            count=len(self.outputs),longest_joint_ready_s=self.longest_joint_ready_s,
            writers={k:len(self.node.get_publishers_info_by_topic(t)) for k,t in INPUT_TOPICS.items()})

    def mark_mtf_stop(self,fault):
        self.mtf_stop=dict(mono=time.monotonic(),fault=fault,ev_count=len(self.outputs))
        self.native.phase='mtf_loss'

    def joint_result(self):
        healthy=[r['topics'] for r in self.native.records if r['phase']=='healthy']
        sensor=assessment(healthy,self.mtf_profile,with_ev=True)
        stopped=set() if not self.mtf_stop else set(('flow','range') if self.mtf_stop['fault']=='all' else (self.mtf_stop['fault'],))
        writers={k:len(self.node.get_publishers_info_by_topic(t)) for k,t in INPUT_TOPICS.items()}
        checks=dict(healthy_window=bool(self.healthy_snapshot and self.healthy_snapshot['vio_ready'] and self.healthy_snapshot['mtf_ready']),
            simultaneous_window=self.longest_joint_ready_s>=5.,actual_mtf_fusion=sensor['passed'],
            sole_mtf_writers=bool(self.healthy_snapshot and self.healthy_snapshot['writers']=={'flow':1,'range':1} and not self.mtf_writer_violations),
            source_writers_after_loss=all(n==(0 if k in stopped else 1) for k,n in writers.items()),
            native_recorder_ok=not self.native.errors)
        stages={}
        if self.mtf_stop:
            fault=self.mtf_stop['fault'];boundary=self.mtf_stop['mono']
            observations=[r for r in self.joint_history if r['phase']=='mtf_loss' and r['mono']>=boundary+.5]
            first=next((r for r in self.joint_history if r['phase']=='mtf_loss' and r['state']!='JOINT_OBSERVATIONS_READY'),None)
            native=[r['topics'] for r in self.native.records if r['phase']=='mtf_loss']
            tail=native[-1] if native else {}
            flags=tail.get('estimator_status_flags',{})
            stopped=['flow','range'] if fault=='all' else [fault]
            checks['mtf_loss_prompt']=bool(first and first['mono']-boundary<=.5)
            checks['vio_survives_mtf_loss']=bool(observations and all(r['state']=='VIO_OBSERVATIONS_ONLY' for r in observations) and
                len(self.outputs)>self.mtf_stop['ev_count']+100)
            for key in stopped:
                flag='cs_opt_flow' if key=='flow' else 'cs_rng_hgt'
                checks[key+'_fusion_retired']=flags.get(flag)==False
            stages['mtf_loss']=dict(fault=fault,first_degraded=first,observations=len(observations),last_flags=flags)
        after=[r for r in self.joint_history if self.stopped_at is not None and r['mono']>=self.stopped_at+.5]
        if not self.mtf_stop:
            checks['mtf_survives_vio_loss']=bool(after and all(r['state']=='MTF_OBSERVATIONS_ONLY' for r in after))
            survivor=[r['topics'] for r in self.native.records if r['phase']=='vio_loss']
            checks['mtf_still_actually_fuses']=assessment(survivor,self.mtf_profile,with_ev=True)['passed']
        else:
            checks['no_complete_observation_set']=bool(after and all(r['state']=='UNAVAILABLE' for r in after))
        return dict(passed=all(checks.values()),checks=checks,actual_mtf=sensor,
            healthy_snapshot=self.healthy_snapshot,longest_joint_ready_s=self.longest_joint_ready_s,
            stages=stages,source_counts=self.mtf_counts,native_errors=self.native.errors,
            writer_violations=self.mtf_writer_violations,
            scope='same-aircraft VIO/flow/range disarmed observation; no mission admission or airborne fallback')

    def evidence(self):
        return dict(super().evidence(),**{'joint-native-receipts.json':self.native.records,
            'joint-observation-history.json':self.joint_history})
