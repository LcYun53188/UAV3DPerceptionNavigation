#!/usr/bin/env python3
"""Check archive bytes, real source binding and separate synthetic curve evidence."""
import gzip
import hashlib
import json
from pathlib import Path
root=Path(__file__).resolve().parent
for line in (root/'SHA256SUMS').read_text().splitlines():
    digest,name=line.split('  ',1)
    assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
for line in (root/'RAW_LOG_SHA256SUMS').read_text().splitlines():
    digest,name=line.split('  ',1)
    assert hashlib.sha256(gzip.decompress((root/name).read_bytes())).hexdigest()==digest,name
for name in ('first-pass','final-pass'):
    directory=root/name
    load=lambda name:json.loads((directory/name).read_text())
    manifest=load('manifest.json');result=load('observation.json')
    planning=load('depth-planning.json');mapping=load('depth-map.json')
    assert result['passed'] and result['depth_mapping_passed'] and result['depth_planning_passed']
    assert result['arming_states']==[1] and result['landed']
    assert planning['passed'] and planning['valid_contexts']>=5 and planning['reason']=='READY'
    assert planning['localization_session']==manifest['run_id']
    assert (planning['map_id'],planning['map_epoch'])==(mapping['map_id'],mapping['epoch'])
    assert planning['map_version']<=mapping['version']
    assert planning['alignment_generation']==1 and planning['alignment_id']=='depth-map-is-ekf-odom-v1'
    assert len(planning['reset_counters'])==6 and planning['reset_counters'][-1]==mapping['odometry_reset_counter']
    assert all(value==1 for value in planning['writers'].values())
    assert planning['planner_goal_readers']==1
    assert planning['flight_envelope_start_clear'] is False and planning['goal_dispatched'] is False
    assert manifest['depth_reference_profile']['camera_pitch_deg']==5
    for path,digest in manifest['depth_reference_assets_sha256'].items():
        assert hashlib.sha256((directory/path).read_bytes()).hexdigest()==digest
failed=json.loads((root/'startup-clock-fail/depth-planning.json').read_text())
assert not failed['passed'] and failed['reason']=='CLOCK_FAULT' and failed['valid_contexts']==0
curve=json.loads((root/'synthetic-ego-regression/result.json').read_text())
assert curve['passed'] and curve['execution_admission']['passed']
assert curve['normal']['detour_y_m']>1. and curve['flight_control_topics']==[]
assert curve['cleanup_exit_codes']==[0,0,0]
print('PASS: archived real-map planning sources and separate synthetic EGO admission; no flight qualification')
