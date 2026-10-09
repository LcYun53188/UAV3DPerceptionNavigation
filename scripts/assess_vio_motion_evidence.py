#!/usr/bin/env python3
"""Replay archived reference-motion error independently, without ROS or live graph."""
import argparse
import gzip
import json
from pathlib import Path
import numpy as np


def rotation(q):
    x,y,z,w = np.asarray(q)/np.linalg.norm(q)
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
        [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
        [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def replay(root):
    def read(name):
        path = root/name
        return json.loads(path.read_text() if path.exists() else gzip.decompress((root/(name+'.gz')).read_bytes()))
    truth,raw,norm,status,result = [read(n+'.json') for n in
        ('truth','sdk-poses','normalized-poses','normalized-status','result')]
    initial = norm[0]['stamp'] if norm else raw[0]['stamp']+2.
    raw = [p for p in raw if p['stamp']>=initial]
    ts = np.array([r['stamp'] for r in truth]); positions = np.array([r['position'] for r in truth])
    raw = [p for p in raw if ts[0]<=p['stamp']<=ts[-1]]
    def sample(t):
        i = int(np.clip(np.searchsorted(ts,t,side='right')-1,0,len(ts)-2))
        f = (t-ts[i])/(ts[i+1]-ts[i]);q0,q1 = np.array(truth[i]['quaternion']),np.array(truth[i+1]['quaternion'])
        if np.dot(q0,q1)<0: q1 = -q1
        return positions[i]*(1-f)+positions[i+1]*f,rotation(q0*(1-f)+q1*f)
    xyz,r = sample(raw[0]['stamp']);align = r@rotation(raw[0]['quaternion']).T
    offset = xyz-align@np.array(raw[0]['position'])
    errors = [np.linalg.norm(align@np.array(p['position'])+offset-sample(p['stamp'])[0]) for p in raw]
    rmse,maximum = float(np.sqrt(np.mean(np.square(errors)))),float(max(errors))
    valid = [s for s in status if s['valid']]
    fault = next((s for s in status if valid and s['mono']>valid[0]['mono'] and not s['valid']),None)
    retirement = None
    if fault:
        late = [s for s in status if s['mono']>fault['mono']+.5]
        retirement = dict(reason=fault['reason'],stamp=fault['stamp'],
            latched=bool(late) and all(not s['valid'] and s['reason']==fault['reason'] for s in late),
            no_late_output=not any(p['mono']>fault['mono']+.5 for p in norm))
        if norm:
            rejected = next((p for p in raw if p['stamp']>norm[-1]['stamp']),None)
            if rejected:
                R=rotation(rejected['quaternion']);A=np.zeros((6,6));A[:3,:3]=A[3:,3:]=R
                retirement['next_sdk_stamp']=rejected['stamp']
                retirement['next_world_variances']=np.diag(A@np.array(rejected['covariance']).reshape(6,6)@A.T).tolist()
    reported = result.get('motion_raw_sdk_diagnostic',{})
    agrees = bool(reported) and abs(reported.get('position_rmse_m',-1)-rmse)<1e-9 and abs(reported.get('position_max_m',-1)-maximum)<1e-9
    return dict(scope='raw SDK error replay only, not confidence or flight acceptance',
        position_rmse_m=rmse,position_max_m=maximum,samples=len(raw),
        error_limits_met=rmse<=.15 and maximum<=.30,reported_metrics_agree=agrees,
        alignment_stamp=raw[0]['stamp'],retirement=retirement,
        original_overall_passed=result['passed'],cleanup_confirmed=result['cleanup_confirmed'],
        fmu_inputs_absent=not any(result['control_publishers'].values()),
        normalized_last_stamp=norm[-1]['stamp'] if norm else None,raw_last_stamp=raw[-1]['stamp'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory',type=Path)
    print(json.dumps(replay(parser.parse_args().directory),indent=2))
