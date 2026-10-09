#!/usr/bin/env python3
"""Read-only per-axis error and covariance chronology; single initial alignment."""
import argparse
import json
from pathlib import Path
import numpy as np
from assess_vio_warehouse import read
from assess_vio_motion_evidence import rotation


def diagnose(folder):
    truth=read(folder,'truth.json');raw=read(folder,'sdk-poses.json')
    norm=read(folder,'normalized-poses.json');reported=read(folder,'result.json')['motion_raw_sdk_diagnostic']
    first=norm[0]['stamp'] if norm else raw[0]['stamp']+2
    ts=np.asarray([x['stamp'] for x in truth]);xyz=np.asarray([x['position'] for x in truth])
    raw=[x for x in raw if max(first,ts[0])<=x['stamp']<=ts[-1]]
    def reference(t):
        i=int(np.clip(np.searchsorted(ts,t,side='right')-1,0,len(ts)-2))
        f=(t-ts[i])/(ts[i+1]-ts[i]);a=np.asarray(truth[i]['quaternion']);b=np.asarray(truth[i+1]['quaternion'])
        if a@b<0:b=-b
        return xyz[i]*(1-f)+xyz[i+1]*f,rotation(a*(1-f)+b*f)
    position,orientation=reference(raw[0]['stamp'])
    align=orientation@rotation(raw[0]['quaternion']).T
    translation=position-align@np.asarray(raw[0]['position'])
    norm_stamps={round(n['stamp']*1e9) for n in norm}
    rows=[]
    for p in raw:
        actual,_=reference(p['stamp']);error=align@np.asarray(p['position'])+translation-actual
        # SDK right-tangent covariance expressed in world for diagnosis only.
        r=align@rotation(p['quaternion']);a=np.zeros((6,6));a[:3,:3]=a[3:,3:]=r
        cov=a@np.asarray(p['covariance']).reshape(6,6)@a.T
        rows.append(dict(stamp=p['stamp'],error_enu_m=error.tolist(),error_m=float(np.linalg.norm(error)),
            world_variances=np.diag(cov).tolist(),normalized_present=round(p['stamp']*1e9) in norm_stamps))
    errors=np.asarray([r['error_enu_m'] for r in rows]);rmse=float(np.sqrt(np.mean(np.sum(errors**2,axis=1))))
    bins=[]
    for start in np.arange(rows[0]['stamp'],rows[-1]['stamp'],10):
        selected=[r for r in rows if start<=r['stamp']<start+10]
        if not selected:continue
        bins.append(dict(start_ros=float(start),count=len(selected),
            rmse_m=float(np.sqrt(np.mean([r['error_m']**2 for r in selected]))),
            mean_error_enu_m=np.mean([r['error_enu_m'] for r in selected],axis=0).tolist(),
            max_world_variance=float(max(max(r['world_variances']) for r in selected))))
    return dict(scope='SDK error/covariance diagnosis; not source confidence or flight acceptance',
        alignment_stamp=rows[0]['stamp'],position_rmse_m=rmse,
        per_axis_rmse_m=np.sqrt(np.mean(errors**2,axis=0)).tolist(),
        metrics_match=abs(rmse-reported['position_rmse_m'])<1e-9,
        windows=bins,samples=rows)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();r=diagnose(a.directory);a.output.write_text(json.dumps(r,indent=2)+'\n')
    print(json.dumps({k:v for k,v in r.items() if k not in ('samples','windows')},indent=2))
