#!/usr/bin/env python3
"""Read-only VIO transport, EKF noise and control-lag diagnosis of a completed run."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from pyulog import ULog
from analyze_vio_chain_flight import read
from assess_vio_timing import assess


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not len(a):
        return dict(samples=0)
    return dict(samples=len(a), minimum=float(a.min()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), maximum=float(a.max()),
                rmse=float(np.sqrt(np.mean(a*a))))


def interpolate(times, source_times, points):
    return np.column_stack([np.interp(times, source_times, points[:, i]) for i in range(3)])


def diagnose(folder):
    result, result_sha = read(folder, 'result.json')
    if not result.get('cleanup_confirmed'):
        raise ValueError('Only completed owned runs may be diagnosed')
    trace, trace_sha = read(folder, 'flight-trace.json')
    paths = list(folder.rglob('*.ulg'))
    if len(paths) != 1:
        raise ValueError('Expected exactly one owned native ULog')
    ulog = ULog(str(paths[0]), message_name_filter_list=[
        'trajectory_setpoint', 'vehicle_local_position', 'vehicle_local_position_setpoint',
        'vehicle_visual_odometry', 'estimator_aid_src_ev_pos', 'estimator_aid_src_ev_yaw'])
    data = {d.name: d.data for d in ulog.data_list}
    params = ulog.initial_parameters
    report = dict(schema=1, run_id=folder.name,
        scope='offline original native-clock transport/control diagnosis; lag probe does not correct reported flight errors or estimator timestamps',
        source_sha256={'result':result_sha, 'trace':trace_sha,
            'ulog':hashlib.sha256(paths[0].read_bytes()).hexdigest()},
        parameters={k:params[k] for k in ('MPC_XY_P', 'MPC_Z_P', 'EKF2_EV_DELAY',
            'EKF2_EV_NOISE_MD', 'EKF2_EVP_NOISE', 'EKF2_EVA_NOISE', 'EKF2_EV_CTRL',
            'EKF2_OF_CTRL', 'EKF2_RNG_CTRL', 'EKF2_IMU_CTRL', 'EKF2_DELAY_MAX')}, timing={})
    controlled = [r for r in trace if r['armed'] == 2]
    if not controlled:
        raise ValueError('No armed trajectory to diagnose')
    start, end = controlled[0]['ros'], controlled[-1]['ros']
    report['transport_wall_timing'] = assess(folder,
        after_mono=controlled[0]['mono'], before_mono=controlled[-1]['mono'])
    for filename in ('sensor-timing.json', 'normalizer-timing.json', 'fusion-timing.json'):
        timing, digest = read(folder, filename)
        report['source_sha256'][filename] = digest
        stages = sorted({r['stage'] for r in timing['records']})
        report['timing'][filename] = dict(total=timing['total'], retained=timing['retained'],
            scope='retained records only; age uses ROS simulation time; duration/gap uses wall monotonic time', stages={})
        for stage in stages:
            rows = [r for r in timing['records'] if r['stage'] == stage and start <= r['ros'] <= end]
            if not rows:
                continue
            report['timing'][filename]['stages'][stage] = dict(
                sample_age_ms=stats([r['age_s']*1000 for r in rows if 'age_s' in r]),
                callback_gap_wall_ms=stats(np.diff([r['mono'] for r in rows])*1000))
            if stage == 'imu':
                report['timing'][filename]['stages'][stage]['recording_decimation'] = 10
            for field in ('sdk_track_s', 'sdk_callback_s'):
                if any(field in r for r in rows):
                    report['timing'][filename]['stages'][stage][field+'_ms'] = stats(
                        [r[field]*1000 for r in rows if field in r])
        rows = timing['records']
        # Same original sample, no cross-clock subtraction.
        pairs = {}
        for r in rows:
            if start <= r['ros'] <= end and r.get('sample', 0) > 0:
                pairs.setdefault(r['sample'], {})[r['stage']] = r['mono']
        for first, last in (('pose_rx', 'pose_emit'), ('pose_rx', 'accepted')):
            delays = [(v[last]-v[first])*1000 for v in pairs.values() if first in v and last in v]
            if delays:
                report['timing'][filename][first+'_to_'+last+'_wall_ms'] = stats(delays)

    ev = data['vehicle_visual_odometry']
    ev_mask = (ev['timestamp']/1e6 >= start) & (ev['timestamp']/1e6 <= end)
    floor = dict(position_variance=params['EKF2_EVP_NOISE']**2,
                 orientation_variance=params['EKF2_EVA_NOISE']**2)
    report['ev_noise_floor'] = dict(mode=params['EKF2_EV_NOISE_MD'], ingestion_floors=floor,
        scope='pinned PX4 EKF2 uses max(parameter variance, message variance) per axis when mode=0', axes={})
    for field in floor:
        for axis in range(3):
            values = ev[f'{field}[{axis}]'][ev_mask]
            report['ev_noise_floor']['axes'][f'{field}[{axis}]'] = dict(
                incoming=stats(values), floor_dominated_fraction=float(np.mean(values < floor[field])),
                effective=stats(np.maximum(values, floor[field])))
    for aid in ('estimator_aid_src_ev_pos', 'estimator_aid_src_ev_yaw'):
        if aid in data:
            a = data[aid]; mask = (a['timestamp']/1e6 >= start) & (a['timestamp']/1e6 <= end)
            report['ev_noise_floor'][aid] = {k:stats(v[mask]) for k,v in a.items()
                if k == 'observation_variance' or k.startswith('observation_variance[')}
    report['ev_noise_floor']['pinned_yaw_fusion_lower_bound_variance'] = max(
        floor['orientation_variance'], params['EKF2_EVA_NOISE'], .01**2)
    report['ev_noise_floor']['yaw_units_issue'] = (
        'Pinned ev_yaw_control.cpp compares orientation variance with unsquared '
        'ev_att_noise; parameter YAML unit is rad, native variance is 0.1 rad^2 '
        'at the default 0.1 setting, rather than the 0.01 rad^2 ingestion floor.')

    local = data['vehicle_local_position']
    local_time = local['timestamp']/1e6
    local_enu = np.column_stack((local['y'], local['x'], -local['z']))
    sp = data['trajectory_setpoint']; sp_time = sp['timestamp']/1e6
    sp_mask = (sp_time >= start) & (sp_time <= end)
    report['offboard_feedforward'] = {field:dict(samples=int(sp_mask.sum()),
        all_axes_nan_samples=int(np.count_nonzero(np.all(np.column_stack(
            [np.isnan(sp[f'{field}[{i}]'][sp_mask]) for i in range(3)]), axis=1))))
        for field in ('velocity', 'acceleration')}
    report['offboard_feedforward']['scope'] = 'armed native log includes PX4 native LAND setpoints'
    report['offboard_feedforward']['per_phase'] = {}
    report['control_lag'] = {}
    for phase in ('NAVIGATE', 'RETURN', 'OBSERVE'):
        rows = [r for r in controlled if r['phase'] == phase]
        if not rows:
            continue
        t = np.array([r['ros'] for r in rows]); references = np.array([r['reference'] for r in rows])
        phase_sp = (sp_time >= t[0]) & (sp_time <= t[-1])
        report['offboard_feedforward']['per_phase'][phase] = {field:dict(
            samples=int(phase_sp.sum()), all_axes_nan_samples=int(np.count_nonzero(
                np.all(np.column_stack([np.isnan(sp[f'{field}[{i}]'][phase_sp])
                                        for i in range(3)]), axis=1))))
            for field in ('velocity', 'acceleration')}
        mask = (local_time >= t[0]+2.5) & (local_time <= t[-1]-.5)
        times = local_time[mask]; positions = local_enu[mask]
        if not len(times):
            continue
        probes = [(float(lag), float(np.sqrt(np.mean(np.sum(
            (interpolate(times-lag, t, references)-positions)**2, axis=1)))))
            for lag in np.arange(0, 2.001, .02)]
        best = min(probes, key=lambda row:row[1])
        report['control_lag'][phase] = dict(samples=len(times),
            unshifted_rmse_m=probes[0][1], best_reference_lag_s=best[0],
            diagnostic_shifted_rmse_m=best[1],
            first_order_xy_time_constant_s=1/params['MPC_XY_P'],
            scope='counterfactual temporal probe in phase interior; no trajectory fit; never a qualified tracking metric')
    # Check that the archived ROS trace and ULog actually share the pinned clock.
    t = np.array([r['ros'] for r in controlled])
    trace_position = np.array([r['position_enu'] for r in controlled])
    report['trace_native_clock_consistency_m'] = stats(np.linalg.norm(
        interpolate(t, local_time, local_enu)-trace_position, axis=1))
    report['script_sha256'] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(diagnose(args.run), indent=2)+'\n')
    print(args.output)
