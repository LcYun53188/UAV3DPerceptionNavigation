#!/usr/bin/env python3
"""Separate SDK conditional export correctness from global covariance validity."""

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
from analyze_vio_sdk16_marginal import B6, PERMUTE
from vio_covariance_consistency import skew


def positive_inverse(h):
    if not np.isfinite(h).all() or np.any(np.diag(h)<=0):return None
    scale=1/np.sqrt(np.diag(h));a=scale[:,None]*((h+h.T)*.5)*scale[None,:]
    if np.linalg.eigvalsh(a)[0]<=0:return None
    return scale[:,None]*np.linalg.inv(a)*scale[None,:]


def diagnose(run):
    path=run/'sdk16-information.csv';data=np.loadtxt(path,delimiter=',',ndmin=2)
    if data.shape[1]!=276:raise ValueError('Native diagnostic layout differs')
    poses=json.loads((run/'sdk-poses.json').read_text());lookup={round(p['stamp']*1e9):p for p in poses}
    state=json.loads((run/'sdk16-inertial-state.json').read_text())
    initialized={round(r[0]) for r in state if len(r)==14 and r[1]==1.}
    counts=dict(logged=len(data),initialized=0,pose_information_invalid=0,full_information_invalid=0,
        full_invalid_successful_pnp=0,matched_conditional_exports=0,invalid_source_covariance=0)
    failures=[];relative=[];ratios=[];sigma=[]
    for row in data:
        stamp=round(row[0])
        if stamp not in initialized:continue
        counts['initialized']+=1
        h=row[3:228].reshape(15,15);r=row[228:237].reshape(3,3);t=row[237:240]
        cc=positive_inverse(h[:6,:6]);full=positive_inverse(h)
        if full is None:
            counts['full_information_invalid']+=1
            counts['full_invalid_successful_pnp']+=int(row[1]==1)
        if cc is None:
            counts['pose_information_invalid']+=1
            failures.append(dict(stamp_s=stamp/1e9,successful_pnp=bool(row[1]),integrated=bool(row[2])))
            continue
        j=np.zeros((6,6));j[:3,:3]=j[3:,3:]=r;j[3:,:3]=-r@skew(t)
        transform=B6@PERMUTE@j
        expected=transform@cc@transform.T
        if full is not None:ratios.append(np.trace(full[:6,:6])/np.trace(cc))
        p=lookup.get(stamp)
        if p is None:continue
        actual=np.asarray(p['covariance']).reshape(6,6)
        if not np.isfinite(actual).all() or np.linalg.eigvalsh((actual+actual.T)*.5)[0]<=0:
            counts['invalid_source_covariance']+=1;continue
        relative.append(float(np.linalg.norm(actual-expected)/np.linalg.norm(expected)))
        sigma.append(np.sqrt(np.diag(actual)))
        counts['matched_conditional_exports']+=1
    return dict(schema=1,run_id=run.name,counts=counts,flight_authorized=False,
        covariance_kind='conditional pose; nuisance states and fixed landmarks not marginalized',
        optimizer_changed=False,export_relative_error_max=max(relative) if relative else None,
        export_relative_error_quantiles=np.quantile(relative,[.5,.95,.99,1]).tolist() if relative else None,
        conditional_export_correct=bool(relative and max(relative)<5e-4 and not counts['invalid_source_covariance']),
        marginal_over_conditional_trace_quantiles=np.quantile(ratios,[0,.5,.95,1]).tolist() if ratios else None,
        native_sigma_m_rad_median=np.median(sigma,axis=0).tolist() if sigma else None,
        invalid_pose_information=failures,global_covariance_qualified=False,
        input_sha256={n:hashlib.sha256((run/n).read_bytes()).hexdigest() for n in
            ['sdk16-information.csv','sdk-poses.json','sdk16-inertial-state.json']},
        diagnostic_tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();result=diagnose(a.run);a.out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n');print(json.dumps(result,indent=2))
