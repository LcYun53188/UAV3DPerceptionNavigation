#!/usr/bin/env python3
"""Read-only stationary startup comparison; never grants flight qualification."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def summarize(folder):
    folder = Path(folder)
    manifest = json.loads((folder/'manifest.json').read_text())
    result = json.loads((folder/'result.json').read_text())
    raw = np.load(folder/'sdk-poses-raw.npz')
    t, p = raw['stamp'], raw['position']
    if not len(t) or not np.isfinite(t).all() or not np.isfinite(p).all():
        raise ValueError('Missing or nonfinite original VIO poses')
    states = json.loads((folder/'sdk16-inertial-state.json').read_text())
    initialized = [row for row in states if len(row) == 14 and row[1] == 1.]
    start = manifest['processes']['gazebo']['started_system_ns']/1e9
    admissions = {}
    for name in ('map-admission', 'flight-admission'):
        path = folder/(name+'.json')
        if path.exists():
            record = json.loads(path.read_text())
            admissions[name] = dict(
                # PAX extraction can round historical filesystem mtimes by
                # a fraction of a microsecond; report at microsecond precision.
                wall_s=round(record.get('system_ns', path.stat().st_mtime_ns)/1e9-start, 6),
                ros_s=record.get('ros'),
                time_basis='recorded_system_ns' if 'system_ns' in record else 'original_file_mtime',
                stability=record.get('stability'))
    bins = []
    for begin in np.arange(np.floor(t[0]/10)*10, t[-1], 10.):
        positions = p[(t >= begin) & (t < begin+10.)]
        if len(positions):
            bins.append(dict(begin_s=float(begin), samples=len(positions),
                             position_span_m=float(np.linalg.norm(np.ptp(positions, axis=0)))))
    truth = []
    truth_path = folder/'course-truth.jsonl'
    if truth_path.exists():
        truth = [json.loads(line) for line in truth_path.read_text().splitlines()]
    truth = [r for r in truth if r['source_sim_s'] >= t[0]]
    truth_span = float(np.linalg.norm(np.ptp([r['position'] for r in truth], axis=0))) if truth else None
    inputs = ['manifest.json', 'result.json', 'sdk-poses-raw.npz', 'sdk16-inertial-state.json']
    inputs += [name+'.json' for name in admissions]
    return dict(schema=1, run_id=folder.name, course=manifest.get('obstacle_course'),
        scope='Ground startup only; raw VIO relative drift is not absolute localization or flight accuracy',
        flight_authorized=False, automatic_flight=result.get('automatic_flight'),
        operator_session_passed=result.get('operator_session_passed'),
        final_grounded=result.get('operator_final_grounded'), cleanup_confirmed=result.get('cleanup_confirmed'),
        pose_samples=len(t), source_interval_s=[float(t[0]),float(t[-1])],
        initialized_ros_s=initialized[0][0]/1e9 if initialized else None,
        max_displacement_from_first_pose_m=float(np.max(np.linalg.norm(p-p[0],axis=1))),
        full_position_span_m=float(np.linalg.norm(np.ptp(p,axis=0))),
        final_accelerometer_bias_m_s2=initialized[-1][11:14] if initialized else None,
        truth_position_span_after_first_vio_m=truth_span, admissions=admissions, ten_second_bins=bins,
        input_sha256={name:hashlib.sha256((folder/name).read_bytes()).hexdigest() for name in inputs})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('runs', type=Path, nargs='+')
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps([summarize(p) for p in args.runs],indent=2,allow_nan=False)+'\n')


if __name__ == '__main__':
    main()
