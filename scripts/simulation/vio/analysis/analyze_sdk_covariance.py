#!/usr/bin/env python3
"""Offline covariance evidence; no change to source quality or flight admission."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import argparse
import gzip
import json
from pathlib import Path
import numpy as np
from assess_vio_motion_evidence import rotation


def read(folder, name):
    path=folder/name
    return json.loads(path.read_text() if path.exists() else
                      gzip.decompress(Path(str(path)+'.gz').read_bytes()))


def covariance_statistics(poses, start):
    selected=[p for p in poses if p['stamp']>=start]
    if not selected:raise ValueError('No covariance samples in observation interval')
    body=np.asarray([p['covariance'] for p in selected]).reshape(-1,6,6)
    symmetric=(body+body.transpose(0,2,1))/2
    rotations=np.asarray([rotation(p['quaternion']) for p in selected])
    transform=np.zeros_like(body)
    transform[:,:3,:3]=transform[:,3:,3:]=rotations
    world=transform@symmetric@transform.transpose(0,2,1)
    diagonals=np.diagonal(world,axis1=1,axis2=2)
    eigenvalues=np.linalg.eigvalsh(symmetric)
    return dict(covariance_semantics='computed under existing right-tangent contract; SDK implementation semantics need re-review',
        samples=len(selected),start_stamp=selected[0]['stamp'],end_stamp=selected[-1]['stamp'],
        max_antisymmetry=float(np.max(np.abs(body-body.transpose(0,2,1)))),
        min_eigenvalue=float(eigenvalues.min()),
        non_psd_samples=int(np.count_nonzero(eigenvalues[:,0]<-1e-8)),
        rotation_eigenvalue_error=float(np.max(np.abs(eigenvalues-np.linalg.eigvalsh(world)))),
        world_variance_quantiles=np.quantile(diagonals,[.05,.5,.95],axis=0).tolist(),
        body_height_over_limit_fraction=float(np.mean(symmetric[:,2,2]>.25)),
        world_height_over_limit_fraction=float(np.mean(diagonals[:,2]>.25)),
        source_covariance_valid_fraction=float(np.mean(np.all((diagonals>0)&(diagonals<=.25),axis=1))),
        sdk_identity_covariance_samples=int(np.count_nonzero(np.max(np.abs(body-np.eye(6)),axis=(1,2))<1e-7)))


def motion_analysis(folder):
    raw=read(folder,'sdk-poses.json');truth=read(folder,'truth.json');result=read(folder,'result.json')
    audit=result['motion_raw_sdk_diagnostic'];start=audit['alignment_stamp']
    r=np.asarray(audit['alignment_rotation']);offset=np.asarray(audit['alignment_translation'])
    ts=np.asarray([p['stamp'] for p in truth]);xyz=np.asarray([p['position'] for p in truth])
    selected=[p for p in raw if max(start,ts[0])<=p['stamp']<=ts[-1]]
    errors=[];variances=[]
    for p in selected:
        target=np.asarray([np.interp(p['stamp'],ts,xyz[:,k]) for k in range(3)])
        errors.append(r@p['position']+offset-target)
        basis=r@rotation(p['quaternion']);cov=np.asarray(p['covariance']).reshape(6,6)[:3,:3]
        variances.append(np.diag(basis@((cov+cov.T)/2)@basis.T))
    errors=np.asarray(errors);variances=np.asarray(variances)
    rmse=float(np.sqrt(np.mean(np.sum(errors**2,axis=1))))
    assert abs(rmse-audit['position_rmse_m'])<1e-9,'Error replay disagrees with runtime'
    timing_available=any((folder/n).exists() for n in
                         ('normalizer-timing.json','normalizer-timing.json.gz'))
    timing=read(folder,'normalizer-timing.json')['records'] if timing_available else []
    drops=[t for t in timing if t['stage']=='quality_drop']
    retire=next((t for t in timing if t['stage']=='retire'),None)
    return dict(scope='error after fixed initial alignment; not calibrated statistical NEES',
        covariance=covariance_statistics(raw,start),position_rmse_m=rmse,
        axis_rmse_m=np.sqrt(np.mean(errors**2,axis=0)).tolist(),
        rms_reported_position_sigma_m=np.sqrt(np.mean(variances,axis=0)).tolist(),
        normalizer_timing_available=timing_available,
        first_quality_drop=drops[0] if drops else None,
        quality_drop_count=len(drops) if timing_available else None,retirement=retire)


def native_poses(rows):
    b=np.array([[0.,0.,1.],[-1.,0.,0.],[0.,-1.,0.]])
    transform=np.zeros((6,6));transform[:3,:3]=transform[3:,3:]=b
    poses=[]
    for row in rows:
        if not row['valid']:continue
        # ROS wrapper permutes rotation/translation and changes the CV basis.
        indices=[3,4,5,0,1,2]
        native=np.asarray(row['covariance_rot_pos_right']).reshape(6,6)
        cov=transform@native[np.ix_(indices,indices)]@transform.T
        # Similarity rotates the quaternion vector; its scalar remains unchanged.
        q=row['quaternion_cv'];converted=np.r_[b@q[:3],q[3]]
        poses.append(dict(stamp=row['stamp'],position=(b@row['position_cv']).tolist(),
            quaternion=converted.tolist(),covariance=cov.ravel().tolist()))
    return poses


def native_motion_accuracy(poses, folder, start):
    truth=read(folder,'truth.json')
    ts=np.asarray([p['stamp'] for p in truth]);xyz=np.asarray([p['position'] for p in truth])
    poses=[p for p in poses if max(start,ts[0])<=p['stamp']<=ts[-1]]
    if not poses:raise ValueError('Native replay does not overlap reference interval')
    def reference(t):
        i=int(np.clip(np.searchsorted(ts,t,side='right')-1,0,len(ts)-2))
        f=(t-ts[i])/(ts[i+1]-ts[i])
        q0=np.asarray(truth[i]['quaternion']);q1=np.asarray(truth[i+1]['quaternion'])
        if q0@q1<0:q1=-q1
        return np.asarray([np.interp(t,ts,xyz[:,k]) for k in range(3)]),rotation(q0*(1-f)+q1*f)
    position,orientation=reference(poses[0]['stamp'])
    align=orientation@rotation(poses[0]['quaternion']).T
    offset=position-align@poses[0]['position']
    errors=np.asarray([align@p['position']+offset-reference(p['stamp'])[0] for p in poses])
    return dict(scope='one initial truth alignment; native replay diagnostic only',
        alignment_stamp=poses[0]['stamp'],samples=len(poses),
        rmse_m=float(np.sqrt(np.mean(np.sum(errors**2,axis=1)))),
        max_m=float(np.max(np.linalg.norm(errors,axis=1))))


def native_analysis(path, reference=None):
    rows=[json.loads(line) for line in path.read_text().splitlines() if line.strip()]
    poses=native_poses(rows);start=max(17.,poses[0]['stamp']+2.)
    selected=[r for r in rows if r['valid'] and r['stamp']>=start]
    world=[]
    for p in [p for p in poses if p['stamp']>=start]:
        r=rotation(p['quaternion']);c=np.asarray(p['covariance']).reshape(6,6)[:3,:3]
        world.append(np.trace(r@c@r.T))
    world=np.asarray(world);resets=np.flatnonzero(world[1:]<world[:-1]*.75)+1
    return dict(scope='same recorded SDK-consumed input; diagnostic exports enabled; no ROS timing qualification',
        sdk_version=rows[0]['sdk_version'],frames=len(rows),valid=sum(r['valid'] for r in rows),
        gravity_available_samples=sum(r.get('gravity_cv') is not None for r in rows),
        keyframes=sum(r.get('keyframe',False) for r in rows),
        observations_quantiles=np.quantile([r['observations'] for r in selected],[0,.5,1]).tolist(),
        landmarks_quantiles=np.quantile([r['landmarks'] for r in selected],[0,.5,1]).tolist(),
        large_covariance_drop_samples=len(resets),
        drops_at_keyframe=sum(bool(selected[i]['keyframe']) for i in resets),
        drop_times=[selected[i]['stamp'] for i in resets],covariance=covariance_statistics(poses,start),
        motion_accuracy=native_motion_accuracy(poses,reference,start) if reference else None)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion',action='append',default=[],help='name=run_directory')
    parser.add_argument('--native',action='append',default=[],help='name=replay_jsonl')
    parser.add_argument('--native-reference',type=Path,help='original motion run for replay accuracy')
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    result=dict(schema=1,flight_authorized=False,
        motions={name:motion_analysis(Path(path)) for name,path in (x.split('=',1) for x in args.motion)},
        native={name:native_analysis(Path(path),args.native_reference) for name,path in (x.split('=',1) for x in args.native)})
    args.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
