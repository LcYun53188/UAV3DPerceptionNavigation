#!/usr/bin/env python3
"""Same-pixel SDK16 VIO/VO diagnostic; one initial yaw/position anchor only."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from analyze_vio_physical_replay import read_rows
from analyze_vio_sdk16_marginal import B
from diagnose_vio_sdk16_errors import match, summarize, relative_errors, stats


def diagnose(folder,recording):
    names=['stock-vio','default-vio','default-vo'];series={n:read_rows(folder/(n+'.jsonl')) for n in names}
    starts=[next(r['stamp'] for r in series[n] if r['valid'] and r.get('gravity_cv') is not None) for n in names[:2]]
    start=max(starts);end=min(s[-1]['stamp'] for s in series.values())
    truth=json.loads((recording/'truth.json').read_text());ts=np.array([r['stamp'] for r in truth]);xyz=np.array([r['position'] for r in truth])
    out={}
    for name,s in series.items():
        eligible=[r for r in s if start<=r['stamp']<=end and r['valid']]
        p=[dict(stamp=r['stamp'],position=(B@r['position_cv']).tolist(),
            quaternion=(Rotation.from_matrix(B)*Rotation.from_quat(r['quaternion_cv'])*Rotation.from_matrix(B.T)).as_quat().tolist()) for r in eligible]
        t0=p[0]['stamp'];ref=Rotation.from_quat([r['quaternion'] for r in truth]);r0=Slerp(ts,ref)(t0).as_matrix();est=Rotation.from_quat(p[0]['quaternion']).as_matrix()
        yaw=np.arctan2(r0[1,0],r0[0,0])-np.arctan2(est[1,0],est[0,0]);a=Rotation.from_euler('z',yaw)
        origin=np.array([np.interp(t0,ts,xyz[:,k]) for k in range(3)]);offset=origin-a.apply(p[0]['position'])
        poses=[dict(r,position=(a.apply(r['position'])+offset).tolist(),quaternion=(a*Rotation.from_quat(r['quaternion'])).as_quat().tolist()) for r in p]
        rows=match(truth,poses,dict(sample_stamp=t0,position_enu=origin.tolist()))
        out[name]=dict(summary=summarize(rows,.1),relative_error_1s=relative_errors(rows,1.),
            native_track_ms=stats([r['track_ms'] for r in s]),invalid_frames=sum(not r['valid'] for r in s),
            initialized_first_stamp_s=starts[names.index(name)] if name!='default-vo' else None,
            initialization_delay_s=starts[names.index(name)]-s[0]['stamp'] if name!='default-vo' else None)
    a,b=series[names[0]],series[names[1]];pairs=[(x,y) for x,y in zip(a,b) if x['valid'] and y['valid']]
    invariant=dict(timestamp_identical=all(x['timestamp_ns']==y['timestamp_ns'] for x,y in zip(a,b)),
        validity_identical=all(x['valid']==y['valid'] for x,y in zip(a,b)),
        position_max_difference_m=max(np.linalg.norm(np.array(x['position_cv'])-y['position_cv']) for x,y in pairs),
        quaternion_max_difference=max(np.linalg.norm(np.array(x['quaternion_cv'])-y['quaternion_cv']) for x,y in pairs))
    return dict(schema=1,flight_authorized=False,common_interval_s=[start,end],
        scope='offline consumed pixels/IMU; corrected physical rig; synchronous SBA; one common-time yaw/position anchor, no roll/pitch/scale/time-offset fit; moving anchor, not ground/flight qualification',
        frame_count={n:len(s) for n,s in series.items()},pose_invariance=invariant,comparisons=out,
        performance_comparison_qualified=False,
        performance_caveat='Three sequential replays; cache/order and synchronous SBA affect timings; no whole-system speedup claim',
        input_sha256={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in [recording/'truth.json',folder/'receipt.json',*[folder/(n+'.jsonl') for n in names]]},
        diagnostic_tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('folder',type=Path);p.add_argument('--recording',type=Path,required=True);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();result=diagnose(a.folder,a.recording);a.out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps(result,indent=2))
