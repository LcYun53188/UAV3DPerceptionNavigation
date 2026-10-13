#!/usr/bin/env python3
"""Native candidate diagnostics, independent of ROS and flight admission.

Position covariance uses the source-reviewed SDK world-frame interpretation.
The SDK 15 modified binary still needs source matching; its values are only
conditional comparisons. No Euler-angle covariance is relabelled as a tangent.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from analyze_sdk_covariance import native_poses, native_motion_accuracy


CV_TO_FLU = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def position_statistics(rows):
    if not rows:
        return None
    covariance = np.array([r['covariance_rot_pos_right'] for r in rows]).reshape(-1, 6, 6)
    if not np.isfinite(covariance).all():
        raise ValueError('Non-finite SDK covariance')
    symmetric = (covariance + covariance.transpose(0, 2, 1)) / 2
    # SDK 16 source computes world Euler/position covariance. Position basis
    # changes with the constant CV/FLU transform, never the current pose.
    position = CV_TO_FLU @ symmetric[:, 3:, 3:] @ CV_TO_FLU.T
    diagonal = np.diagonal(position, axis1=1, axis2=2)
    return dict(samples=len(rows),
                ros_world_position_variance_quantiles=np.quantile(diagonal, [.05, .5, .95], axis=0).tolist(),
                position_trace_median=float(np.median(np.trace(position, axis1=1, axis2=2))),
                over_position_limit_fraction=float(np.mean(np.any(diagonal > .25, axis=1))),
                non_psd_samples=int(np.count_nonzero(np.linalg.eigvalsh(symmetric)[:, 0] < -1e-8)))


def keyframe_window_max(rows, window=25.):
    stamps = [r['stamp'] for r in rows if r.get('keyframe')]
    first, maximum = 0, 0
    for last, stamp in enumerate(stamps):
        while stamp - stamps[first] > window:
            first += 1
        maximum = max(maximum, last - first + 1)
    return maximum


def audit(rows, reference=None, start=17.):
    if not rows:
        raise ValueError('No replay samples')
    stamps = np.array([r['stamp'] for r in rows])
    if not np.isfinite(stamps).all() or not np.all(np.diff(stamps) > 0):
        raise ValueError('Invalid or unordered replay timeline')
    valid = [r for r in rows if r['valid']]
    if not valid:
        raise ValueError('No valid poses')
    selected = [r for r in valid if r['stamp'] >= start]
    if not selected:
        raise ValueError('No observation window')
    gravity = [r for r in valid if r.get('gravity_cv') is not None]
    imu = [r for r in valid if r.get('imu_state') is not None]
    settings = [{k: rows[0].get(k) for k in ('async_sba', 'realtime', 'keyframe_max_seconds')}]
    if any(any(r.get(k) != settings[0][k] for k in settings[0]) for r in rows):
        raise ValueError('Replay configuration changed inside stream')
    result = dict(scope='native input replay only; not statistical calibration, ROS timing or flight acceptance',
                  flight_authorized=False, qualified=False, sdk_version=rows[0]['sdk_version'],
                  covariance_semantics='position statistics assume source-reviewed world covariance; SDK15 modified semantics remain conditional',
                  frames=len(rows), valid=len(valid), settings=settings[0],
                  keyframes=sum(bool(r.get('keyframe')) for r in rows),
                  keyframe_max_in_25s=keyframe_window_max(rows),
                  gravity_samples=len(gravity), first_gravity_stamp=gravity[0]['stamp'] if gravity else None,
                  gravity_norm_quantiles=np.quantile([np.linalg.norm(r['gravity_cv']) for r in gravity], [.05, .5, .95]).tolist() if gravity else None,
                  imu_state_samples=len(imu),
                  first_nonzero_bias_stamp=next((r['stamp'] for r in imu if np.linalg.norm(r['imu_state']['gyro_bias']) + np.linalg.norm(r['imu_state']['acc_bias']) > 1e-10), None),
                  last_imu_state=imu[-1]['imu_state'] if imu else None,
                  track_ms_quantiles=np.quantile([r['track_ms'] for r in rows], [.5, .95, .99, 1.]).tolist(),
                  observation_window=position_statistics(selected),
                  gravity_unavailable=position_statistics([r for r in selected if r.get('gravity_cv') is None]),
                  gravity_available=position_statistics([r for r in selected if r.get('gravity_cv') is not None]),
                  motion_accuracy=native_motion_accuracy(native_poses(rows), reference, start) if reference else None)
    return result


def compare_pose_rows(baseline, probe):
    left = {r['timestamp_ns']: r for r in baseline if r['valid']}
    right = {r['timestamp_ns']: r for r in probe if r['valid']}
    if left.keys() != right.keys():
        raise ValueError('Probe and baseline valid timestamps differ')
    if not left:
        raise ValueError('No common valid poses')
    differences = [np.linalg.norm(np.array(left[t]['position_cv']) - right[t]['position_cv']) for t in left]
    return dict(samples=len(left), max_position_difference_m=float(max(differences)),
                rms_position_difference_m=float(np.sqrt(np.mean(np.square(differences)))))


def read_rows(path):
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--replay', action='append', required=True, help='name=file.jsonl')
    parser.add_argument('--reference', type=Path)
    parser.add_argument('--start', type=float, default=17.)
    parser.add_argument('--pose-probe', help='baseline_name,probe_name')
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    recordings = {name: read_rows(Path(path)) for name, path in (v.split('=', 1) for v in args.replay)}
    result = dict(schema=1, flight_authorized=False,
                  runs={name: audit(rows, args.reference, args.start) for name, rows in recordings.items()})
    if args.pose_probe:
        left, right = args.pose_probe.split(',')
        result['covariance_probe_pose_comparison'] = compare_pose_rows(recordings[left], recordings[right])
    args.out.write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
