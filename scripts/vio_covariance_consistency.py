#!/usr/bin/env python3
"""Truth-observer-only six-dimensional consistency rejection test.

This checks a fixed gauge, not absolute position/yaw observability. The SDK
does not expose anchor/current correlation: use its PSD upper bound, never
an independence assumption or an empirically fitted variance multiplier.
Correlated frames are not independent Monte Carlo trials. Passing the coverage
screen is necessary, and does not authorize flight or prove calibration.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp
from scipy.stats import chi2


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def anchor_bound(a, displacement, current, initial):
    jc = np.zeros((6, 6)); jc[:3, :3] = jc[3:, 3:] = a
    ji = np.zeros((6, 6)); ji[:3, :3] = ji[3:, 3:] = -a
    ji[:3, 3:] = skew(displacement) @ a
    return 2 * (jc @ current @ jc.T + ji @ initial @ ji.T)


def evaluate(truth, poses, *, aligned=False, binding=None):
    ts = np.asarray([r['stamp'] for r in truth])
    if len(ts) < 2 or np.any(np.diff(ts) <= 0):
        raise ValueError('Truth must be strictly ordered')
    ps = np.asarray([r['position'] for r in truth])
    rotations = Rotation.from_quat([r['quaternion'] for r in truth])
    interpolate = Slerp(ts, rotations)
    paired = [p for p in poses if ts[0] <= p['stamp'] <= ts[-1]]
    if not paired: raise ValueError('No bracketed poses')
    pt = np.asarray([p['stamp'] for p in paired])
    if np.any(np.diff(pt) <= 0): raise ValueError('Poses must be strictly ordered')
    reference_p = np.column_stack([np.interp(pt, ts, ps[:, k]) for k in range(3)])
    reference_r = interpolate(pt).as_matrix()
    p0 = np.asarray(paired[0]['position'])
    c0 = np.asarray(paired[0]['covariance']).reshape(6, 6)
    if aligned:
        if not binding: raise ValueError('Launch alignment binding required')
        # Fixed configured launch yaw, and independently observed physical launch
        # position at the actual ground-binding timestamp. No trajectory fit.
        t0 = binding['sample_stamp']
        if not ts[0] <= t0 <= ts[-1]: raise ValueError('Anchor not bracketed')
        origin = np.array([np.interp(t0, ts, ps[:, k]) for k in range(3)])
        reference_p += np.asarray(binding['position_enu']) - origin
        a = np.eye(3); offset = np.zeros(3)
    else:
        a = reference_r[0] @ Rotation.from_quat(paired[0]['quaternion']).as_matrix().T
        offset = reference_p[0] - a @ p0
    rows = []; minimum = float('inf')
    for i, p in enumerate(paired):
        position = a @ np.asarray(p['position']) + offset
        r = a @ Rotation.from_quat(p['quaternion']).as_matrix()
        error = np.r_[position-reference_p[i], Rotation.from_matrix(r @ reference_r[i].T).as_rotvec()]
        c = np.asarray(p['covariance']).reshape(6, 6)
        if not aligned: c = anchor_bound(a, a @ (np.asarray(p['position'])-p0), c, c0)
        c = (c+c.T)/2
        eig = np.linalg.eigvalsh(c); minimum = min(minimum, float(eig[0]))
        if not np.isfinite(c).all() or eig[0] <= 0: raise ValueError('Non-positive covariance')
        nees = float(error @ np.linalg.solve(c, error))
        rows.append(dict(stamp=p['stamp'],error=error.tolist(),nees=nees,
                         position_sigma=np.sqrt(np.diag(c)[:3]).tolist()))
    # Exclude the first forced-zero gauge anchor from the rejection screen.
    selected = [r for r in rows if r['stamp'] >= rows[0]['stamp']+2.]
    if not selected: raise ValueError('Insufficient post-anchor window')
    values = np.asarray([r['nees'] for r in selected])
    limit95, limit99 = float(chi2.ppf(.95, 6)), float(chi2.ppf(.99, 6))
    # Conservative screening floors, fixed before new online data collection.
    # No ANEES confidence interval: temporal independence is not established.
    coverage95 = float(np.mean(values <= limit95)); coverage99 = float(np.mean(values <= limit99))
    checks = dict(minimum_window=bool(pt[-1]-pt[0]>=120.),minimum_samples=len(selected)>=1000,
                  sample_continuity=float(np.max(np.diff(pt)))<=.081,
                  positive_covariance=bool(minimum>0),coverage95_screen=coverage95>=.90,
                  coverage99_screen=coverage99>=.98)
    errors = np.asarray([r['error'] for r in selected])
    return dict(schema=1,scope='fixed-gauge global pose covariance rejection screen; truth observer only',
        absolute_global_observability=False,flight_authorized=False,
        rejection_screen_passed=all(checks.values()),checks=checks,aligned_launch=aligned,
        unknown_temporal_correlation_bound=not aligned or binding.get('covariance_policy')=='unknown_temporal_correlation_bound_v1',
        independent_trials=False,statistical_calibration_proven=False,
        paired_samples=len(rows),interval_s=[float(pt[0]),float(pt[-1])],minimum_eigenvalue=minimum,
        nees_median=float(np.median(values)),nees_p95=float(np.quantile(values,.95)),nees_max=float(values.max()),
        chi2_6d_threshold95=limit95,chi2_6d_threshold99=limit99,
        coverage95=coverage95,coverage99=coverage99,
        position_rmse_m=float(np.sqrt(np.mean(np.sum(errors[:,:3]**2,axis=1)))),
        orientation_rmse_deg=float(np.degrees(np.sqrt(np.mean(np.sum(errors[:,3:]**2,axis=1))))),rows=rows)


def analyze(folder):
    read = lambda n: json.loads((folder/n).read_text())
    if (folder/'fusion-outputs.json').is_file():
        records = read('fusion-outputs.json'); poses=[]
        for row in records:
            m=row['aligned'];s=m['header']['stamp'];p=m['pose']['pose']
            poses.append(dict(stamp=s['sec']+s['nanosec']/1e9,
                position=[p['position'][k] for k in ('x','y','z')],
                quaternion=[p['orientation'][k] for k in ('x','y','z','w')],covariance=m['pose']['covariance']))
        result=evaluate(read('truth.json'),poses,aligned=True,binding=read('result.json')['real_pose_fusion']['alignment'])
        names=['truth.json','fusion-outputs.json','result.json']
    else:
        result=evaluate(read('truth.json'),read('normalized-poses.json'))
        names=['truth.json','normalized-poses.json']
    result['run_id']=folder.name
    result['input_sha256']={n:hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in names}
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path);parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args();result=analyze(args.folder)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
    return 0 if result['rejection_screen_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
