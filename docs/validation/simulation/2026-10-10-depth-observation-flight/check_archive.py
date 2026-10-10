#!/usr/bin/env python3
"""Validate expected observation abort, physical motion and confirmed ground cleanup."""
import gzip
import hashlib
import json
import math
from pathlib import Path
root=Path(__file__).resolve().parent
for line in (root/'SHA256SUMS').read_text().splitlines():
    digest,name=line.split('  ',1)
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
for line in (root/'RAW_LOG_SHA256SUMS').read_text().splitlines():
    digest,name=line.split('  ',1)
    assert hashlib.sha256(gzip.decompress((root/name).read_bytes())).hexdigest()==digest,name
for directory in ('first-flight','measured-yaw-flight'):
    folder=root/directory
    load=lambda name:json.loads((folder/name).read_text())
    r=load('flight-observation.json');m=load('manifest.json')
    assert r['passed'] and r['observation_cleanup_verified'] and not r['observation_task_succeeded']
    assert r['action_status']==6 and r['result']==dict(code='ABORTED',reason='OBSERVATION_INSUFFICIENT',cleanup_confirmed=True,mock=False)
    assert r['final_land'] and r['final_arming']==1 and r['max_truth_displacement_m']>=1.5
    assert r['bt_exit_code']==1 and r['bt_dispatch_count']==1
    assert r['bt_step_accepts']==[0,1] and r['bt_step_completes']==[0]
    assert 'OBSERVE' in r['phases'] and 'NAVIGATE' not in r['phases'] and 'PLAN_REQUEST' not in r['phases']
    assert max(v['angle_rad'] for v in r['observations'])>=2*math.pi
    assert not any(v['clear'] for v in r['observations'])
    assert m['depth_reference_profile']['camera_pitch_deg']==5 and not m['require_vio']
    receipt=load('depth-assets.json')
    assert receipt['localization_session']==m['run_id'] and receipt['partition']==m['partition']
    for name,digest in receipt['files'].items():assert hashlib.sha256((folder/name).read_bytes()).hexdigest()==digest
    commands=load('flight-commands.json')
    assert any(c['command']==400 for c in commands) and any(c['command']==21 for c in commands)
    truth=load('flight-truth.json')
    points=[t['position'] for t in truth if t['phase']=='OBSERVE']
    assert len(points)>100 and max(math.dist(p,points[0]) for p in points)<.3
final=json.loads((root/'measured-yaw-flight/flight-observation.json').read_text())
headings=[v['measured_heading_rad'] for v in final['observations']]
sweep=sum(math.atan2(math.sin(b-a),math.cos(b-a)) for a,b in zip(headings,headings[1:]))
assert abs(sweep-final['observation_measured_yaw_sweep_rad'])<1e-9 and sweep>=5.8
assert final['planning_context_final']['localization_session']==json.loads((root/'measured-yaw-flight/manifest.json').read_text())['run_id']
assert final['planning_context_final']['observed_voxels']>100000
print('PASS: actual takeoff/yaw observation and expected abort with landed disarmed cleanup; no navigation success')
