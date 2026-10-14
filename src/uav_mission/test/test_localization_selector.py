from concurrent.futures import Future
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest
from px4_msgs.msg import EstimatorAidSource1d,EstimatorAidSource2d
from uav_mission.localization_selector import LocalizationSelector
from uav_mission.px4_flight import FlightServer
from uav_mission.sitl_fusion_parameters import SitlFusionParameters,FROZEN
from uav_nav_interfaces.srv import SelectLocalization
from unique_identifier_msgs.msg import UUID
from test_vio_gate import CAL,pose_update


class Parameters:
    def __init__(self):self.calls=[];self.futures=[];self.fail_next=False;self.pending=False
    def submit(self,target):
        self.calls.append(target)
        future=Future();self.futures.append(future)
        if self.fail_next:
            future.set_exception(ValueError('readback failure'));self.fail_next=False
        elif not self.pending:
            future.set_result(dict(before=dict(FROZEN,EKF2_EV_CTRL=11 if target==10 else 10),
                                   after=dict(FROZEN,EKF2_EV_CTRL=target)))
        return future


def update(selector,now,*,flow=False,reset=0):
    gate=selector.gates['VIO_FLOW']
    pose_update(gate,now)
    messages=deepcopy(gate.samples)
    messages['flags'].cs_opt_flow=messages['flags'].cs_rng_hgt=True
    messages['flags'].cs_ev_pos=not flow
    messages['local'].xy_reset_counter=reset
    messages['local'].dist_bottom_valid=True;messages['local'].dist_bottom=1.5
    for name,kind in [('optical_flow',EstimatorAidSource2d),('rng_hgt',EstimatorAidSource1d)]:
        messages[name]=kind(timestamp=int(now*1e6),timestamp_sample=int(now*1e6),time_last_fuse=int(now*1e6),fused=True)
    for name,message in messages.items():selector.receive(name,message,now)


def tick(selector,now,*,flow=False,reset=0,ground=True,endpoints='same'):
    update(selector,now,flow=flow,reset=reset)
    return selector.tick(now,now,{n:1 for n in selector.gates['VIO_FLOW'].required},ground,endpoints)


def ready():
    backend=Parameters();s=LocalizationSelector(CAL,backend)
    for t in (10.,11.,12.):tick(s,t)
    assert s.ready and s.navigation_allowed
    return s,backend


def test_parameter_ack_is_not_fusion_confirmation_and_round_trip_creates_new_epoch():
    s,b=ready()
    assert s.request('FLOW_HORIZONTAL',bytes([1]*16),12.01,True)==(True,'SWITCH_ADMITTED')
    tick(s,12.1)  # Actual flags still show EV position despite parameter ACK.
    assert s.phase=='VERIFYING' and s.active_profile=='VIO_FLOW' and not s.ready
    for t in (12.2,13.2,14.3):tick(s,t,flow=True)
    assert s.ready and s.active_profile=='FLOW_HORIZONTAL' and s.generation==1
    assert not s.navigation_allowed
    assert s.samples['flags'].cs_ev_hgt and s.samples['flags'].cs_ev_yaw
    assert s.request('VIO_FLOW',bytes([2]*16),14.31,True)[0]
    tick(s,14.4,flow=True)
    # PX4 restart of EV XY produces a ground-only position reset.
    for t in (14.5,15.5,16.6):tick(s,t,reset=1)
    assert s.ready and s.active_profile=='VIO_FLOW' and s.generation==2 and not s.fault
    assert s.navigation_allowed and b.calls==[10,11]


@pytest.mark.parametrize('target',['VIO_FLOW','FLOW_HORIZONTAL','FLOW_LOCAL','invalid'])
def test_airborne_or_active_task_cannot_start_a_switch(target):
    s,b=ready()
    assert not s.request(target,bytes([1]*16),12.1,False)[0]
    assert b.calls==[] and s.generation==0


def test_startup_resets_restart_warmup_but_bound_idle_resets_latch():
    s=LocalizationSelector(CAL,Parameters())
    tick(s,10.);tick(s,11.,reset=1)
    assert not s.fault and not s.ready
    tick(s,12.,reset=1);tick(s,13.1,reset=1)
    assert s.ready
    tick(s,13.2,reset=2)
    assert s.fault=='LOCALIZATION_RESET_OUTSIDE_SWITCH' and not s.ready


def test_request_identity_cache_cannot_rewrite_or_replay_switch():
    s,b=ready();key=bytes([1]*16)
    assert s.request('FLOW_HORIZONTAL',key,12.1,True)[0]
    assert s.request('FLOW_HORIZONTAL',key,12.2,True)[0]
    assert s.request('VIO_FLOW',key,12.2,True)==(False,'REQUEST_ID_CONFLICT')
    assert not s.request('FLOW_HORIZONTAL',bytes(16),12.2,True)[0]
    assert b.calls==[10]


@pytest.mark.parametrize('failure',['parameter','timeout'])
def test_bounded_rollback_requires_readback_and_actual_old_fusion(failure):
    s,b=ready();b.fail_next=failure=='parameter'
    assert s.request('FLOW_HORIZONTAL',bytes([1]*16),12.1,True)[0]
    for i in range(100):
        t=12.2+i*.1
        tick(s,t)  # EV position remains active: FLOW target cannot confirm.
        if s.phase=='VERIFYING_ROLLBACK':break
    assert s.phase=='VERIFYING_ROLLBACK' and b.calls==[10,11] and not s.ready
    start=t
    for i in range(1,25):tick(s,start+i*.1)
    assert s.phase=='ROLLED_BACK' and s.ready and s.active_profile=='VIO_FLOW' and s.generation==0


def test_rollback_timeout_never_claims_available_old_profile():
    s,b=ready();b.pending=True
    assert s.request('FLOW_HORIZONTAL',bytes([1]*16),12.1,True)[0]
    for i in range(170):tick(s,12.2+i*.1)
    assert s.phase=='FAULT' and s.fault=='ROLLBACK_FUSION_TIMEOUT' and not s.ready


@pytest.mark.parametrize('fault',['source','selector','ground','clock','endpoint'])
def test_fault_or_authority_loss_during_transition_does_not_auto_clear(fault):
    s,b=ready()
    assert s.request('FLOW_HORIZONTAL',bytes([1]*16),12.1,True)[0]
    if fault=='source':
        source=deepcopy(s.samples['source']);source.valid=False;s.receive('source',source,12.11)
    elif fault=='selector':
        selector=deepcopy(s.samples['selector']);selector.primary_instance=1;s.receive('selector',selector,12.11)
    update(s,12.2)
    s.tick(12.2,11. if fault=='clock' else 12.2,{n:1 for n in s.gates['VIO_FLOW'].required},
        fault!='ground','different' if fault=='endpoint' else 'same')
    assert s.phase=='FAULT' and not s.ready
    tick(s,12.3)
    assert s.phase=='FAULT' and not s.request('VIO_FLOW',bytes([2]*16),12.4,True)[0]


@pytest.mark.parametrize('fault',['range','height','yaw','flow','source_stale'])
def test_flow_horizontal_still_requires_height_yaw_range_and_actual_fusion(fault):
    s,_=ready()
    assert s.request('FLOW_HORIZONTAL',bytes([1]*16),12.1,True)[0]
    for t in (12.2,13.2,14.3):tick(s,t,flow=True)
    assert s.ready
    if fault=='range':s.samples['local'].dist_bottom_valid=False
    elif fault in ('height','yaw'):setattr(s.samples['flags'],'cs_ev_hgt' if fault=='height' else 'cs_ev_yaw',False)
    elif fault=='flow':s.samples['optical_flow'].time_last_fuse=1
    else:s.samples['source'].sample_stamp.sec=10
    assert not s.tick(14.3,14.3,{n:1 for n in s.gates['VIO_FLOW'].required},True,'same')


def test_gateway_scope_rejects_old_coordinator_and_non_nil_root():
    s,_=ready()
    f=SimpleNamespace(lock=threading.RLock(),localization_selector=s,instance='current')
    for instance,root in [('old',[0]*16),('current',[1]*16)]:
        req=SelectLocalization.Request(coordinator_instance=instance,mission_uuid=UUID(uuid=root),
            request_id=UUID(uuid=[1]*16),profile='FLOW_HORIZONTAL')
        response=FlightServer.select_localization(f,req,SelectLocalization.Response())
        assert not response.accepted
        assert response.reason=='STALE_IDENTITY_OR_NON_NIL_ROOT'


def parameter_backend(tmp_path,monkeypatch,values=None,guard=lambda:True):
    values=dict(FROZEN,EKF2_EV_CTRL=11) if values is None else values
    build=tmp_path/'.deps/px4-vio-sdk16-build';(build/'bin').mkdir(parents=True)
    (build/'bin/px4').write_bytes(b'test-owned-binary')
    artifact='.deps/px4-vio-sdk16-build/bin/px4'
    (build/'vio-build.json').write_text(json.dumps({'artifacts':{artifact:hashlib.sha256(b'test-owned-binary').hexdigest()}}))
    evidence=tmp_path/'.cache/simulation/test';evidence.mkdir(parents=True)
    monkeypatch.setenv('ROS_DOMAIN_ID','78')
    monkeypatch.setattr(SitlFusionParameters,'owned',lambda self:None)
    calls=[]
    def run(command,**_):
        calls.append(command)
        verb,name=command[3:5]
        if verb=='set':values[name]=int(command[5])
        return SimpleNamespace(stdout=f'{name} [used]: {values[name]}\n')
    monkeypatch.setattr('uav_mission.sitl_fusion_parameters.subprocess.run',run)
    return SitlFusionParameters(tmp_path,evidence,guard),calls


def test_native_backend_writes_only_ev_horizontal_mask_and_verifies_frozen_parameters(tmp_path,monkeypatch):
    b,calls=parameter_backend(tmp_path,monkeypatch)
    try:
        receipt=b.submit(10).result(timeout=2)
        assert receipt['before']['EKF2_EV_CTRL']==11 and receipt['after']['EKF2_EV_CTRL']==10
        writes=[c[3:] for c in calls if c[3]=='set']
        assert writes==[['set','EKF2_EV_CTRL','10']]
        assert all(receipt['after'][n]==v for n,v in FROZEN.items())
    finally:b.close()


@pytest.mark.parametrize('failure',['target','configuration','authority'])
def test_native_backend_rejects_unreviewed_changes_before_any_write(tmp_path,monkeypatch,failure):
    values=dict(FROZEN,EKF2_EV_CTRL=11)
    if failure=='configuration':values['EKF2_GPS_CTRL']=1
    b,calls=parameter_backend(tmp_path,monkeypatch,values,guard=lambda:failure!='authority')
    try:
        with pytest.raises(ValueError):b.submit(0 if failure=='target' else 10).result(timeout=2)
        assert not any(c[3]=='set' for c in calls)
    finally:b.close()


def test_native_backend_never_binds_an_unowned_pid_or_other_domain(tmp_path,monkeypatch):
    b,_=parameter_backend(tmp_path,monkeypatch)
    monkeypatch.undo()
    try:
        with pytest.raises((OSError,ValueError)):SitlFusionParameters.owned(b)
    finally:b.close()
