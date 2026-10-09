"""Qualification cannot be inferred from raw PASS or scene appearance."""
import json
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
from assess_vio_warehouse import assess, digest_file
from prepare_vio_sensor_assets import assets


def case(tmp_path):
    world=tmp_path/'upstream.sdf';world.write_text('<sdf version="1.9"><world name="default"/></sdf>')
    run=tmp_path/'run';profile,_=assets(run/'assets',world,scene='warehouse')
    manifest=dict(run_id='test',profile=profile,fuse_pose=True,motion=False,duration_s=120,
        input_sha256={'original/run/'+name:digest_file(run/name) for name in
            ('assets/default.sdf','assets/'+profile['model']+'/model.sdf','assets/warehouse-layout.json')})
    result=dict(passed=True,arming_states=[1],cleanup_confirmed=True,
        real_pose_fusion=dict(longest_ready_s=115))
    for name,value in [('manifest',manifest),('result',result)]:
        (run/(name+'.json')).write_text(json.dumps(value))
    return run,manifest,result


def test_source_pass_still_does_not_authorize_flight(tmp_path):
    run,_,_=case(tmp_path);r=assess(run)
    assert r['passed'] and not r['flight_authorized']
    assert 'WH-F01' in r['pending_cases'] and 'WH-N01' in r['pending_cases']


def test_short_raw_pass_is_not_warehouse_qualification(tmp_path):
    run,m,r=case(tmp_path);m['duration_s']=60;r['real_pose_fusion']['longest_ready_s']=55
    (run/'manifest.json').write_text(json.dumps(m));(run/'result.json').write_text(json.dumps(r))
    report=assess(run);assert not report['passed']
    assert not report['checks']['full_duration'] and not report['checks']['sustained_ready']


def test_asset_or_texture_drift_cannot_qualify(tmp_path):
    run,_,_=case(tmp_path)
    with (run/'assets/default.sdf').open('a') as f:f.write('\n')
    report=assess(run);assert not report['passed'] and not report['checks']['generated_assets_match']
    texture=next((run/'assets').rglob('*.png'));texture.write_bytes(b'changed')
    report=assess(run);assert not report['checks']['texture_integrity']


def test_missing_motion_metrics_are_failed_checks_not_type_errors(tmp_path):
    run,m,r=case(tmp_path);m['fuse_pose']=False;m['motion']=True
    r['motion']=dict(position_rmse_m=None,position_max_m=None,orientation_max_deg=None)
    (run/'manifest.json').write_text(json.dumps(m));(run/'result.json').write_text(json.dumps(r))
    report=assess(run);assert not report['passed']
    assert not report['checks']['bounded_rmse'] and not report['checks']['bounded_max']
