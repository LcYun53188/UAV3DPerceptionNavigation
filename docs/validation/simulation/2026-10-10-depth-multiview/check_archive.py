#!/usr/bin/env python3
"""Check physical surveys, map gaps, terminal semantics and immutable evidence."""
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
load=lambda folder,name:json.loads((root/folder/name).read_text())
for folder in ('tracking-limit-flight','slow-tracking-limit-flight','final-flight'):
    r=load(folder,'flight-observation.json');m=load(folder,'manifest.json')
    assert r['final_land'] and r['final_arming']==1 and r['bt_dispatch_count']==1
    assert m['depth_reference_profile']['camera_pitch_deg']==5 and not m['require_vio']
    receipt=load(folder,'depth-assets.json')
    assert receipt['localization_session']==m['run_id'] and receipt['partition']==m['partition']
    for name,digest in receipt['files'].items():
        assert hashlib.sha256((root/folder/name).read_bytes()).hexdigest()==digest
    assert hashlib.sha256((root/folder/'flight-profile.json').read_bytes()).hexdigest()==m['flight_profile_sha256']
    assert hashlib.sha256((root/folder/'mission-recipe.json').read_bytes()).hexdigest()==m['flight_recipe_sha256']
    if folder!='final-flight':
        assert not r['passed'] and r['result']['reason']=='OBSERVATION_DRIFT'
        assert not r['result']['cleanup_confirmed'] and r['fallback_landed_disarmed']
        assert 'PLAN_REQUEST' not in r['phases']
r=load('final-flight','flight-observation.json');config=load('final-flight','flight-profile.json')
assert r['passed']
observations=r['observations'];truth=load('final-flight','flight-truth.json')
trace=load('final-flight','flight-trace.json')
assert max(math.dist(t['position_enu'],t['reference']) for t in trace if t['phase']=='OBSERVE')<=.3
for index,offset in enumerate(config['observation_offsets_enu']):
    scan=[v for v in observations if v['survey_index']==index and v['survey_state']=='SCAN']
    assert len(scan)>=16
    points=[t['position'] for t in truth if scan[0]['mono']<=t['mono']<=scan[-1]['mono']]
    expected=[r['home_truth'][i]+offset[i] for i in range(3)]
    assert points and max(math.dist(p,expected) for p in points)<=.3
assert any(v['survey_state']=='DONE' for v in observations)
assert max(v['angle_rad'] for v in observations)>=len(config['observation_offsets_enu'])*math.tau
assert r['observation_measured_yaw_sweep_rad']>=len(config['observation_offsets_enu'])*math.tau-1.
last=observations[-1];volume=last['volume']
assert volume and volume['outside_cells']==0
assert sum(v['unknown'] for v in volume['layers'])==volume['unknown']
assert r['planning_context_final']['localization_session']==load('final-flight','manifest.json')['run_id']
if r['observation_task_succeeded']:
    assert r['result']['code']=='SUCCEEDED' and 'PLAN_REQUEST' in r['phases']
    assert r['bt_step_completes']==list(range(len(r['steps'])))
    assert all(w['error_m']<=.3 for w in r['waypoint_truth'])
else:
    assert r['action_status']==6 and r['result']['reason']=='OBSERVATION_INSUFFICIENT'
    assert r['observation_cleanup_verified'] and r['result']['cleanup_confirmed']
    assert 'PLAN_REQUEST' not in r['phases'] and volume['unknown']>0
print('PASS: archive, actual multiview survey, terminal semantics and height-resolved map diagnostics')
