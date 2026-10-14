"""Single-parameter writes to a proven local PX4 SITL instance, never hardware.

Native CLI work runs outside ROS callbacks. Only EV_CTRL 10/11 is writable;
the continuous height/yaw, flow/range and single-EKF configuration is frozen.
"""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import time


FROZEN = {'EKF2_GPS_CTRL':0,'EKF2_OF_CTRL':1,'EKF2_RNG_CTRL':2,
          'EKF2_HGT_REF':3,'EKF2_MAG_TYPE':5,'SENS_IMU_MODE':0,
          'EKF2_MULTI_IMU':1,'EKF2_MULTI_MAG':0}


class SitlFusionParameters:
    def __init__(self, workspace, evidence, guard):
        self.workspace, self.evidence = Path(workspace).resolve(), Path(evidence).resolve()
        self.build=self.workspace/'.deps/px4-vio-sdk16-build'
        self.guard, self.closed=guard,False
        self.deadline=None
        if not self.evidence.is_relative_to(self.workspace/'.cache/simulation'):
            raise ValueError('SITL_EVIDENCE_OUTSIDE_WORKSPACE')
        if os.environ.get('ROS_DOMAIN_ID')!='78':raise ValueError('SITL_DOMAIN_REQUIRED')
        receipt=json.loads((self.build/'vio-build.json').read_text())
        name=str((self.build/'bin/px4').relative_to(self.workspace))
        if receipt['artifacts'].get(name)!=hashlib.sha256((self.build/'bin/px4').read_bytes()).hexdigest():
            raise ValueError('SITL_BUILD_IDENTITY_INVALID')
        self.owned()
        self.pool=ThreadPoolExecutor(max_workers=1,thread_name_prefix='sitl-fusion-parameter')

    def owned(self):
        if self.closed:raise ValueError('PARAMETER_BACKEND_CLOSED')
        manifest=json.loads((self.evidence/'manifest.json').read_text())
        if manifest.get('domain')!=78 or manifest.get('partition')!=os.environ.get('GZ_PARTITION'):
            raise ValueError('SITL_SESSION_MISMATCH')
        pid=manifest['processes']['px4']['pid']
        command=Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
        if (command[0]!=str(self.build/'bin/px4').encode() or b'-i' not in command or b'-w' not in command or
                command[command.index(b'-i')+1]!=b'7' or
                command[command.index(b'-w')+1]!=str(self.evidence/'rootfs').encode() or
                Path(f'/proc/{pid}/exe').resolve()!=(self.build/'bin/px4').resolve()):
            raise ValueError('SITL_INSTANCE_NOT_OWNED')
        return manifest

    def cli(self, *arguments):
        self.owned()
        if not self.guard():raise ValueError('GROUND_AUTHORITY_LOST')
        remaining=4. if self.deadline is None else self.deadline-time.monotonic()
        if remaining<=0:raise ValueError('PARAMETER_TRANSACTION_TIMEOUT')
        run=subprocess.run([str(self.build/'bin/px4-param'),'--instance','7',*arguments],
            cwd=self.workspace,env=os.environ.copy(),text=True,stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,timeout=min(1.,remaining),check=True)
        return run.stdout

    def read(self, name):
        text=self.cli('show',name)
        match=re.search(r'\b'+re.escape(name)+r'\s*\[[^\]]+\]\s*:\s*([-+\d.eE]+)',text)
        if not match:raise ValueError('PARAMETER_RECEIPT_MISSING:'+name)
        value=float(match.group(1))
        if not value.is_integer():raise ValueError('PARAMETER_RECEIPT_NOT_INTEGER:'+name)
        return int(value)

    def apply(self, target):
        if target not in (10,11):raise ValueError('PARAMETER_TARGET_NOT_ALLOWED')
        self.deadline=time.monotonic()+4.
        before={name:self.read(name) for name in (*FROZEN,'EKF2_EV_CTRL')}
        if any(before[name]!=value for name,value in FROZEN.items()) or before['EKF2_EV_CTRL'] not in (10,11):
            raise ValueError('FROZEN_FUSION_CONFIGURATION_CHANGED')
        # One integer write only. No EKF restart, height/yaw/magnetometer edits,
        # sensor process restart, MAVLink or flight-command publisher.
        self.cli('set','EKF2_EV_CTRL',str(target))
        after={name:self.read(name) for name in before}
        if after['EKF2_EV_CTRL']!=target or any(after[name]!=value for name,value in FROZEN.items()):
            raise ValueError('FUSION_PARAMETER_READBACK_INVALID')
        return dict(before=before,after=after)

    def submit(self, target):
        return self.pool.submit(self.apply,target)

    def close(self):
        self.closed=True
        self.pool.shutdown(wait=False,cancel_futures=True)
