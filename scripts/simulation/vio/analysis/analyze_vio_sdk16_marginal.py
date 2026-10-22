#!/usr/bin/env python3
"""Validate repaired SDK exports and same-trajectory covariance counterfactuals."""

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
from scipy.spatial.transform import Rotation
from vio_covariance_consistency import evaluate, skew

B=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
PERMUTE=np.eye(6)[[3,4,5,0,1,2]]
B6=np.zeros((6,6));B6[:3,:3]=B6[3:,3:]=B

def euler_to_world(c,r):
    cp=np.hypot(r[0,0],r[1,0]);cy=r[0,0]/cp;sy=r[1,0]/cp;sp=-r[2,0]
    e=np.array([[cy*cp,-sy,0.],[sy*cp,cy,0.],[-sp,0.,1.]])
    j=np.zeros((6,6));j[:3,3:]=B;j[3:,:3]=B@e
    return j@((c+c.T)*.5)@j.T

def analyze(information,poses,truth):
    data=np.loadtxt(information,delimiter=',',ndmin=2)
    if data.shape[1]!=276:raise ValueError('Unreviewed native information diagnostic layout')
    lookup={round(row[0]):row for row in data};selected=[];legacy=[];conditional=[]
    maximum_relative=0.;ratios=[];invalid=0
    for p in poses:
        row=lookup.get(round(p['stamp']*1e9))
        if row is None:continue
        h=row[3:228].reshape(15,15);r=row[228:237].reshape(3,3);t=row[237:240]
        if not np.isfinite(h).all() or np.any(np.diag(h)<=0):invalid+=1;continue
        scale=1/np.sqrt(np.diag(h));hs=scale[:,None]*((h+h.T)*.5)*scale[None,:]
        if np.linalg.eigvalsh(hs)[0]<=0:invalid+=1;continue
        c=(scale[:,None]*np.linalg.inv(hs)*scale[None,:])[:6,:6]
        cc=np.linalg.inv((h[:6,:6]+h[:6,:6].T)*.5)
        j=np.zeros((6,6));j[:3,:3]=j[3:,3:]=r;j[3:,:3]=-r@skew(t)
        world=B6@PERMUTE@j@c@j.T@PERMUTE.T@B6.T
        actual=np.asarray(p['covariance']).reshape(6,6)
        maximum_relative=max(maximum_relative,float(np.linalg.norm(actual-world)/np.linalg.norm(world)))
        ratios.append(np.trace(c)/np.trace(cc))
        rr=B.T@Rotation.from_quat(p['quaternion']).as_matrix()@B
        old=euler_to_world(row[240:276].reshape(6,6),rr)
        fixed_conditional=B6@PERMUTE@j@cc@j.T@PERMUTE.T@B6.T
        selected.append(p);legacy.append(dict(p,covariance=old.ravel().tolist()))
        conditional.append(dict(p,covariance=fixed_conditional.ravel().tolist()))
    outputs={}
    for name,series in [('legacy_api',legacy),('conditional_with_correct_frames',conditional),('marginal_with_correct_frames',selected)]:
        out=evaluate(truth,series);outputs[name]={k:v for k,v in out.items() if k!='rows'}
    return dict(schema=1,flight_authorized=False,
        scope='same estimator trajectory; conditional vs marginal and frame export diagnostics',
        native_diagnostic_sha256=hashlib.sha256(information.read_bytes()).hexdigest(),
        diagnostic_rows=len(data),matched_exports=len(selected),invalid_information_exports=invalid,
        export_relative_error_max=maximum_relative,export_matches_full_state=maximum_relative<5e-4,
        trace_marginal_over_conditional_quantiles=np.quantile(ratios,[0.,.5,.95,1.]).tolist(),
        remaining_conditional_states=['fixed landmark coordinates','SBA-frozen IMU biases','gravity'],comparisons=outputs)

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path);parser.add_argument('--out',required=True,type=Path)
    args=parser.parse_args();read=lambda n:json.loads((args.folder/n).read_text())
    out=analyze(args.folder/'sdk16-information.csv',read('normalized-poses.json'),read('truth.json'))
    args.out.parent.mkdir(parents=True,exist_ok=True);args.out.write_text(json.dumps(out,indent=2)+'\n')
    print(json.dumps(out,indent=2));return 0 if out['export_matches_full_state'] else 1

if __name__=='__main__':raise SystemExit(main())
