#!/usr/bin/env python3
"""Read-only warehouse scene/source qualification; never authorizes flight."""
import argparse
import gzip
import hashlib
import json
from pathlib import Path
from vio_warehouse_scene import box_clearance

ROOT=Path(__file__).resolve().parents[1]


def read(folder,name):
    p=folder/name
    return json.loads(p.read_text()) if p.is_file() else json.loads(gzip.decompress(Path(str(p)+'.gz').read_bytes()))


def digest_file(p):
    raw=p.read_bytes() if p.is_file() else gzip.decompress(Path(str(p)+'.gz').read_bytes())
    return hashlib.sha256(raw).hexdigest()


def assess(folder):
    m=read(folder,'manifest.json');r=read(folder,'result.json')
    receipt=read(folder,'assets/warehouse-layout.json')
    cfg=json.loads((ROOT/'simulation/scenes/warehouse_acceptance.json').read_text())
    geometry=box_clearance(receipt['layout'])
    world=folder/'assets/default.sdf'
    model=folder/'assets'/m['profile']['model']/'model.sdf'
    asset_suffixes=('assets/default.sdf','assets/'+m['profile']['model']+'/model.sdf','assets/warehouse-layout.json')
    assets_match=all(len(matches:=[(name,digest) for name,digest in m['input_sha256'].items() if name.endswith('/'+suffix)])==1
                     and digest_file(folder/suffix)==matches[0][1] for suffix in asset_suffixes)
    common=dict(warehouse_scene=m['profile'].get('scene')=='warehouse',
        generated_assets_match=assets_match,
        clearance_matches=receipt['clearance']==geometry,
        layout_current=receipt['layout_sha256']==digest_file(ROOT/'simulation/scenes/vio_warehouse.json'),
        texture_integrity=all(digest_file(folder/'assets'/name)==digest for name,digest in receipt.get('texture_sha256',{}).items()),
        disarmed=r.get('arming_states')==[1],cleanup=r.get('cleanup_confirmed') is True)
    if m.get('fuse_pose'):
        case='WH-V01';fusion=r.get('real_pose_fusion',{})
        checks=dict(common,raw_passed=r.get('passed') is True and not m.get('diagnostic_visual_only',False) and not m.get('sdk_debug_dump',False),
            full_duration=m['duration_s']>=cfg['required_fusion_duration_s'],
            sustained_ready=fusion.get('longest_ready_s',0)>=cfg['required_continuous_ready_s'])
    elif m.get('motion'):
        case='WH-V02';motion=r.get('motion',{})
        checks=dict(common,raw_passed=r.get('passed') is True and not m.get('diagnostic_visual_only',False) and not m.get('sdk_debug_dump',False),
            full_duration=m['duration_s']>=120,
            bounded_rmse=isinstance(motion.get('position_rmse_m'),(int,float)) and 0<=motion['position_rmse_m']<=cfg['position_rmse_max_m'],
            bounded_max=isinstance(motion.get('position_max_m'),(int,float)) and 0<=motion['position_max_m']<=cfg['position_error_max_m'],
            bounded_orientation=isinstance(motion.get('orientation_max_deg'),(int,float)) and 0<=motion['orientation_max_deg']<=cfg['orientation_error_max_deg'])
    else:raise ValueError('Unsupported warehouse case')
    return dict(schema=1,run_id=m['run_id'],case=case,passed=all(checks.values()),checks=checks,
        raw_passed=r.get('passed'),flight_authorized=False,
        pending_cases=[c['id'] for c in cfg['cases'] if c['id'].startswith(('WH-F','WH-N'))],
        scope='source prerequisite only; flight/hover/navigation not evaluated',
        artifacts=dict(world_sha256=digest_file(world),model_sha256=digest_file(model)))


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();report=assess(a.directory);a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
