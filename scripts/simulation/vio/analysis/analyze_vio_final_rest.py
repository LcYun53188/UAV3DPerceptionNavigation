#!/usr/bin/env python3
"""Separate pose marginal diagnostics in an already frozen final-rest window."""

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


def analyze(run, covariance_path):
    covariance = json.loads(covariance_path.read_text())
    if covariance['run_id'] != run.name:
        raise ValueError('Covariance analysis belongs to another run')
    for name, digest in covariance['input_sha256'].items():
        if hashlib.sha256((run/name).read_bytes()).hexdigest() != digest:
            raise ValueError('Covariance input changed: '+name)
    outputs = json.loads((run/'fusion-outputs.json').read_text())
    trigger = json.loads((run/'fixture-trigger.json').read_text())['ros']
    rows = covariance['rows']
    if len(outputs) != len(rows):
        raise ValueError('Output pairing mismatch')
    selected = []
    for row, output in zip(rows, outputs):
        message = output['aligned']
        stamp = message['header']['stamp']
        t = stamp['sec']+stamp['nanosec']/1e9
        if abs(t-row['stamp']) > 1e-7:
            raise ValueError('Original timestamp pairing mismatch')
        if t < trigger+105.:
            continue
        c = np.asarray(message['pose']['covariance']).reshape(6, 6)
        error = np.asarray(row['error'])
        selected.append(dict(stamp=t, error=error.tolist(), sigma=np.sqrt(c.diagonal()).tolist(),
            nees_pose=row['nees'],
            nees_position_marginal=float(error[:3]@np.linalg.solve(c[:3, :3], error[:3])),
            nees_orientation_marginal=float(error[3:]@np.linalg.solve(c[3:, 3:], error[3:]))))
    if not selected:
        raise ValueError('No final-rest samples')
    native = json.loads((run/'sdk16-inertial-state.json').read_text())
    native = [r for r in native if len(r) == 14 and r[1] == 1. and
              selected[0]['stamp'] <= r[0]/1e9 <= selected[-1]['stamp']]
    if not native:
        raise ValueError('No paired native inertial states')
    last = selected[-1]
    return dict(schema=1, run_id=run.name, flight_authorized=False,
        scope='diagnostic only; position and orientation marginal NEES do not add to full pose NEES',
        interval_s=[selected[0]['stamp'], selected[-1]['stamp']], samples=len(selected),
        final_error_m_rad=last['error'], final_standard_deviation_m_rad=last['sigma'],
        final_orientation_error_deg=np.degrees(last['error'][3:]).tolist(),
        final_orientation_standard_deviation_deg=np.degrees(last['sigma'][3:]).tolist(),
        median_position_marginal_nees=float(np.median([r['nees_position_marginal'] for r in selected])),
        median_orientation_marginal_nees=float(np.median([r['nees_orientation_marginal'] for r in selected])),
        mean_native_accelerometer_bias_m_s2=np.mean([r[11:14] for r in native], axis=0).tolist(),
        mean_native_gyro_bias_rad_s=np.mean([r[8:11] for r in native], axis=0).tolist(),
        model_bias_authority='declared simulation IMU: Gaussian white noise, no constant or time-varying bias',
        interpretation='Tilt/bias coupling is a remaining hypothesis; these diagnostics do not prove its unique cause.',
        input_sha256={str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
            [covariance_path, run/'fusion-outputs.json', run/'fixture-trigger.json', run/'sdk16-inertial-state.json']})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--covariance', required=True, type=Path)
    parser.add_argument('--out', required=True, type=Path)
    args = parser.parse_args()
    result = analyze(args.run, args.covariance)
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
