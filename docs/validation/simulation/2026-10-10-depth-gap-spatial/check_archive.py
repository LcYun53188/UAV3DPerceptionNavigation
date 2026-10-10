#!/usr/bin/env python3
"""Verify immutable evidence and expected failures, without asserting task success."""
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parent

def load(folder, name):
    path = ROOT / folder / name
    raw = path.read_bytes() if path.exists() else gzip.decompress(path.with_suffix(path.suffix+'.gz').read_bytes())
    return json.loads(raw)

for filename, raw in [('SHA256SUMS', False), ('RAW_LOG_SHA256SUMS', True)]:
    for line in (ROOT / filename).read_text().splitlines():
        digest, name = line.split('  ', 1)
        content = (ROOT / name).read_bytes()
        assert hashlib.sha256(gzip.decompress(content) if raw else content).hexdigest() == digest, name
old = load('', 'old-camera-mount.json')
assert not old['passed'] and abs(old['translation_error_m']-.24)<1e-6
assert hashlib.sha256(gzip.decompress((ROOT/'old-camera-mount-scene.txt.gz').read_bytes())).hexdigest()==old['scene_sha256']
expected = {'diagnostic-flight': (4930, 2), 'fixed-mount-flight': (4043, 0),
            'enclosed-flight': (185, 0), 'complementary-flight': (155, 0), 'covered-flight': (1, 0)}
for folder in [p.name for p in ROOT.iterdir() if p.is_dir() and (p/'manifest.json').exists()]:
    manifest = load(folder, 'manifest.json')
    assert manifest['depth_reference_profile']['camera_pitch_deg'] == 5 and not manifest['require_vio']
    for filename, key in [('flight-profile.json', 'flight_profile_sha256'), ('mission-recipe.json', 'flight_recipe_sha256')]:
        assert hashlib.sha256((ROOT/folder/filename).read_bytes()).hexdigest() == manifest[key], (folder, filename)
    for filename, digest in load(folder, 'depth-assets.json')['files'].items():
        assert hashlib.sha256((ROOT/folder/filename).read_bytes()).hexdigest() == digest, (folder, filename)
    if folder != 'diagnostic-flight':
        mount = load(folder, 'camera-mount.json')
        assert mount['passed'] and mount['translation_error_m'] < 1e-6 and mount['rotation_error_rad'] < 1e-6
        assert hashlib.sha256(gzip.decompress((ROOT/folder/'camera-mount-scene.txt.gz').read_bytes())).hexdigest()==mount['scene_sha256']
    for path in (ROOT/folder).glob('observation-grid-*.json'):
        metadata = json.loads(path.read_text())
        grid_path = path.parent/metadata['grid_file']
        assert hashlib.sha256(grid_path.read_bytes()).hexdigest() == metadata['grid_sha256']
        assert metadata['localization_session'] == manifest['run_id']
        with np.load(grid_path, allow_pickle=False) as grid:
            extent = np.array([metadata['radius_m']+metadata['braking_margin_m']]*2+[metadata['radius_m']])
            lo = np.floor((np.asarray(metadata['point_map'])-extent-grid['origin'])/float(grid['resolution'])).astype(int)
            hi = np.floor((np.asarray(metadata['point_map'])+extent-grid['origin'])/float(grid['resolution'])).astype(int)
            assert np.all(lo>=0) and np.all(hi<grid['distance'].shape)
            slices = tuple(slice(a,b+1) for a,b in zip(lo,hi))
            d,o = grid['distance'][slices], grid['observed'][slices].astype(bool)
            unknown,occupied = int(np.count_nonzero(~o)), int(np.count_nonzero(o & np.isfinite(d) & (d<=0)))
            assert unknown == metadata['diagnostics']['unknown'] and occupied == metadata['diagnostics']['occupied']
            if path.name == 'observation-grid-returned.json' and folder in expected:
                assert (unknown,occupied) == expected[folder]
    if folder in expected:
        result = load(folder, 'flight-observation.json')
        assert result['final_land'] and result['final_arming']==1 and result['observation_cleanup_verified']
    if folder == 'ego-planner-no-path-flight':
        result = load(folder, 'flight-observation.json')
        assert not result['passed'] and result['fallback_landed_disarmed']
        assert load(folder, 'flight-events.json')[-1]['reason'] == 'EGO_EXECUTION:PLANNER_TIMEOUT'
        assert not load(folder, 'accepted-plans.json')
        metadata = load(folder, 'observation-grid-returned.json')
        with np.load(ROOT/folder/metadata['grid_file'], allow_pickle=False) as grid:
            radius = .85
            lo = np.floor((np.asarray(metadata['point_map'])-radius-grid['origin'])/float(grid['resolution'])).astype(int)
            hi = np.floor((np.asarray(metadata['point_map'])+radius-grid['origin'])/float(grid['resolution'])).astype(int)
            observed = grid['observed'][tuple(slice(a,b+1) for a,b in zip(lo,hi))]
            assert int(np.count_nonzero(~observed)) == 3
    if folder == 'seed-reserve-observation-insufficient':
        result=load(folder, 'flight-observation.json')
        assert result['result']['code']=='ABORTED' and result['result']['reason']=='OBSERVATION_INSUFFICIENT'
        assert result['observation_cleanup_verified'] and result['result']['cleanup_confirmed']
        assert not load(folder, 'accepted-plans.json')
        metadata=load(folder, 'observation-grid-returned.json')
        assert abs(metadata['radius_m']-.85)<1e-6 and metadata['diagnostics']['unknown']==1
    if folder == 'depth-startup-rejected':
        observation = load(folder, 'observation.json')
        assert not observation['passed'] and observation['arming_states']==[1]
        assert not load(folder, 'depth-camera.json')['passed']
        assert not (ROOT/folder/'flight-commands.json').exists()
    if folder in ('ego-complete-flight', 'ego-final-sweep-flight'):
        result = load(folder, 'flight-observation.json')
        assert result['passed'] and result['result']['code']=='SUCCEEDED'
        assert result['result']['cleanup_confirmed'] and result['final_land'] and result['final_arming']==1
        assert result['navigation_backend']=='EGO' and not result['vio_required']
        assert result['bt_step_accepts']==list(range(7)) and result['bt_step_completes']==list(range(7))
        assert result['bt_dispatch_count']==1 and result['bt_exit_code']==0
        plans = load(folder, 'accepted-plans.json')
        assert len(plans)==2 and [p['authorization']['step_index'] for p in plans]==[2,4]
        assert len({tuple(p['authorization']['mission_uuid']) for p in plans})==1
        assert len({tuple(p['authorization']['child_uuid']) for p in plans})==2
        for plan in plans:
            assert plan['authorization']['owner']=='TASK'
            assert plan['authorization']['coordinator_instance']==manifest['run_id']
            assert plan['bound']['context']['localization_session']==manifest['run_id']
            assert plan['bound']['trajectory']['map_id']==plan['bound']['context']['map_id']
        events,truth = load(folder, 'flight-events.json'), load(folder, 'flight-truth.json')
        assert events[-1]['phase']=='COMPLETE' and not any(e['phase']=='FAULT' for e in events)
        for event,next_event in zip(events,events[1:]):
            if event['phase'] not in ('NAVIGATE','RETURN'): continue
            points = [t for t in truth if event['mono']<=t['mono']<=next_event['mono']]
            assert points and next_event['phase']=='AWAIT_STEP'
            expected = np.asarray(event['target_enu'])-result['home_enu']+np.asarray(result['home_truth'])
            error = float(np.linalg.norm(np.asarray(points[-1]['position'])-expected))
            assert error<.2, (folder,event['phase'],error)
        trace = load(folder, 'flight-trace.json')
        assert trace[-1]['owner']=='NONE' and trace[-1]['armed']==1
        active = [t for t in trace if t['phase'] in ('NAVIGATE','RETURN')]
        assert active and all(np.linalg.norm(np.asarray(t['position_enu'])-t['reference'])<=.3 for t in active)
    if folder == 'ego-return-tracking-flight':
        assert not (ROOT/folder/'flight-truth.json').exists() and not (ROOT/folder/'flight-truth.json.gz').exists()
    if folder == 'ego-map-stale-flight':
        result = load(folder, 'flight-observation.json')
        assert not result['passed'] and result['fallback_landed_disarmed']
        events = load(folder, 'flight-events.json')
        assert events[-1]['reason'] == 'EGO_EXECUTION:STALE_MAP_SOURCE'
        plans = load(folder, 'accepted-plans.json')
        assert len(plans)==1
        assert plans[0]['authorization']['coordinator_instance']==manifest['run_id']
        assert plans[0]['bound']['context']['localization_session']==manifest['run_id']
audit=load('', 'process-group-audit.json')
assert audit['all_exited'] and audit['groups'] and not any(g['present'] for g in audit['groups'])
print('Archive hashes, physical mounts, voxel counts, source identities and expected failures verified.')
