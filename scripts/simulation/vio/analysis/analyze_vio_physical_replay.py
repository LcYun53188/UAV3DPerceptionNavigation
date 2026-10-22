#!/usr/bin/env python3
"""Recompute physical-reference replay diagnostics, never flight acceptance."""

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

from analyze_vio_sdk16_marginal import B, analyze, euler_to_world
from vio_covariance_consistency import evaluate


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def position_accuracy(poses, truth):
    # Match the consistency tool's bracketing and two-second anchor exclusion,
    # without interpreting the original SDK covariance as a valid uncertainty.
    times = np.asarray([r['stamp'] for r in truth])
    xyz = np.asarray([r['position'] for r in truth])
    poses = [p for p in poses if times[0] <= p['stamp'] <= times[-1]]
    stamps = np.asarray([p['stamp'] for p in poses])
    reference = np.column_stack([np.interp(stamps, times, xyz[:, k]) for k in range(3)])
    attitude = Slerp(times, Rotation.from_quat([r['quaternion'] for r in truth]))(stamps[0]).as_matrix()
    a = attitude@Rotation.from_quat(poses[0]['quaternion']).as_matrix().T
    offset = reference[0]-a@np.asarray(poses[0]['position'])
    error = np.asarray([a@np.asarray(p['position'])+offset for p in poses])-reference
    error = error[stamps >= stamps[0]+2.]
    return dict(scope='one initial full rigid alignment; post-anchor two-second exclusion',
                alignment_stamp=float(stamps[0]), samples=len(error),
                rmse_m=float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
                max_m=float(np.max(np.linalg.norm(error, axis=1))))


def run(folder, recording):
    truth = json.loads((recording/'truth.json').read_text())
    stock = read_rows(folder/'sdk16.jsonl')
    selected = [r for r in stock if r['valid'] and r['gravity_cv'] is not None]
    stock_poses = [dict(stamp=r['stamp'], position=(B@r['position_cv']).tolist(),
        quaternion=np.r_[B@r['quaternion_cv'][:3], r['quaternion_cv'][3]].tolist()) for r in selected]
    position = position_accuracy(stock_poses, truth)
    position.update(covariance_qualified=False,
        reason='Original SDK covariance uses incorrect error parameterization; only position assessed here.')
    marginal = read_rows(folder/'marginal.jsonl')
    poses = []
    invalid_stamps = []
    for row in marginal:
        if not row['valid'] or row['gravity_cv'] is None:
            continue
        c = np.asarray(row['covariance_rot_pos_right'], dtype=float).reshape(6, 6)
        if not np.isfinite(c).all() or np.linalg.eigvalsh((c+c.T)/2)[0] <= 0:
            # Counterfactuals are evaluated only where a covariance exists.
            # Explicitly retain failure stamps; exclusion cannot qualify a source.
            invalid_stamps.append(row['stamp'])
            continue
        r = Rotation.from_quat(row['quaternion_cv']).as_matrix()
        poses.append(dict(stamp=row['stamp'], position=(B@row['position_cv']).tolist(),
            quaternion=Rotation.from_matrix(B@r@B.T).as_quat().tolist(),
            covariance=euler_to_world(c, r).ravel().tolist()))
    comparison = analyze(folder/'information.csv', poses, truth)
    comparison['invalid_source_covariance_stamps_s'] = invalid_stamps
    comparison['source_validity_passed'] = not invalid_stamps
    openvins = evaluate(truth, [r for r in read_rows(folder/'openvins.jsonl') if r['initialized']])
    openvins = {k: v for k, v in openvins.items() if k != 'rows'}
    inputs = [folder/(n+'.jsonl') for n in ('sdk16', 'marginal', 'openvins')]
    inputs += [folder/'information.csv', recording/'truth.json']
    return dict(schema=1, flight_authorized=False,
        scope='under 120s; initial full rigid alignment; offline diagnostics, not online qualification',
        stock_position=position, marginal=comparison, openvins=openvins,
        input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in inputs})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder', type=Path)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    result = run(args.folder, args.recording)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(dict(stock_position=result['stock_position'],
        marginal_coverage95=result['marginal']['comparisons']['marginal_with_correct_frames']['coverage95'],
        openvins_position_rmse_m=result['openvins']['position_rmse_m']), indent=2))


if __name__ == '__main__':
    main()
