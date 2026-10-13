#!/usr/bin/env python3
"""SDK16 error diagnostics against observer-only truth, with no trajectory fit."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation, Slerp


def stats(values):
    v=np.asarray(values,dtype=float)
    v=v[np.isfinite(v)]
    if not len(v): return dict(samples=0)
    return dict(samples=len(v),rmse=float(np.sqrt(np.mean(v*v))),median=float(np.median(v)),
        p95=float(np.quantile(v,.95)),p99=float(np.quantile(v,.99)),maximum=float(v.max()))


def match(truth,poses,binding):
    ts=np.asarray([r['stamp'] for r in truth]);xyz=np.asarray([r['position'] for r in truth])
    if len(ts)<2 or np.any(np.diff(ts)<=0): raise ValueError('Truth must be strictly increasing')
    paired=[p for p in poses if ts[0]<=p['stamp']<=ts[-1]]
    if not paired: raise ValueError('No bracketed poses')
    pt=np.asarray([p['stamp'] for p in paired])
    if np.any(np.diff(pt)<=0): raise ValueError('Pose timestamps must be strictly increasing')
    anchor=binding['sample_stamp']
    if not ts[0]<=anchor<=ts[-1]: raise ValueError('Anchor not bracketed by independent truth')
    origin=np.array([np.interp(anchor,ts,xyz[:,k]) for k in range(3)])
    reference=np.column_stack([np.interp(pt,ts,xyz[:,k]) for k in range(3)])-origin+binding['position_enu']
    ref_r=Slerp(ts,Rotation.from_quat([r['quaternion'] for r in truth]))(pt)
    estimated=np.asarray([p['position'] for p in paired])
    rot=Rotation.from_quat([p['quaternion'] for p in paired])
    orientation=(rot*ref_r.inv()).as_rotvec()
    # Derive Euler yaw from matrix elements, without Euler gimbal ambiguity.
    a=rot.as_matrix();b=ref_r.as_matrix()
    yaw=np.arctan2(a[:,1,0],a[:,0,0])-np.arctan2(b[:,1,0],b[:,0,0])
    yaw=np.arctan2(np.sin(yaw),np.cos(yaw))
    indices=np.clip(np.searchsorted(ts,pt,side='right'),1,len(ts)-1)
    gaps=ts[indices]-ts[indices-1]
    rows=[dict(stamp=float(t),position_error_m=e.tolist(),orientation_error_deg=np.degrees(o).tolist(),
        yaw_error_deg=float(np.degrees(y)),truth_bracket_gap_s=float(g),reference_position_m=r.tolist())
        for t,e,o,y,g,r in zip(pt,estimated-reference,orientation,yaw,gaps,reference)]
    return rows


def summarize(rows,limit):
    if not rows:return dict(status='NOT_MEASURED',samples=0,maximum_position_within_limit=False)
    e=np.asarray([r['position_error_m'] for r in rows]);o=np.asarray([r['orientation_error_deg'] for r in rows])
    norm=np.linalg.norm(e,axis=1);worst=int(np.argmax(norm))
    return dict(samples=len(rows),interval_s=[rows[0]['stamp'],rows[-1]['stamp']],
        position_m=stats(norm),position_axis_rmse_m=np.sqrt(np.mean(e*e,axis=0)).tolist(),
        position_axis_max_abs_m=np.max(np.abs(e),axis=0).tolist(),orientation_deg=stats(np.linalg.norm(o,axis=1)),
        yaw_deg=stats(np.abs([r['yaw_error_deg'] for r in rows])),
        worst_stamp_s=rows[worst]['stamp'],worst_position_error_m=e[worst].tolist(),
        maximum_position_within_limit=bool(norm.max()<=limit),limit_m=limit,
        truth_bracket_max_gap_s=max(r['truth_bracket_gap_s'] for r in rows))


def relative_errors(rows,horizon):
    if len(rows)<2:return dict(samples=0,horizon_s=horizon)
    ts=np.asarray([r['stamp'] for r in rows]);errors=np.asarray([r['position_error_m'] for r in rows])
    values=[]
    for i,t in enumerate(ts):
        j=int(np.searchsorted(ts,t+horizon))
        if j<len(ts) and abs(ts[j]-t-horizon)<=.041:
            values.append(np.linalg.norm(errors[j]-errors[i]))
    return dict(horizon_s=horizon,position_increment_error_m=stats(values),
        scope='world-frame displacement increment error; no best-fit trajectory alignment')


def timing(folder):
    read=lambda n:json.loads((folder/n).read_text())
    result={}
    for name,stage in [('normalizer-timing.json','pose_rx'),('fusion-timing.json','pose_rx')]:
        if not (folder/name).exists():continue
        data=read(name)
        records=[r for r in data['records'] if r['stage']==stage and r.get('sample',0)>0]
        result[name]=dict(original_sample_age_ms=stats([r['age_s']*1000 for r in records]),
            retained=data['retained'],total=data['total'],
            scope='ROS simulation clock age at callback; not wall-clock camera-to-actuator latency')
    return result


def time_offset_probe(truth,poses,binding,start):
    """Counterfactual only: never alter timestamps or qualification metrics."""
    ts=np.asarray([r['stamp'] for r in truth]);xyz=np.asarray([r['position'] for r in truth])
    origin=np.array([np.interp(binding['sample_stamp'],ts,xyz[:,k]) for k in range(3)])
    selected=[p for p in poses if start+12<=p['stamp']<=start+88 and ts[0]+.12<=p['stamp']<=ts[-1]-.12]
    if not selected:return dict(status='NOT_MEASURED')
    t=np.asarray([p['stamp'] for p in selected]);p=np.asarray([p['position'] for p in selected]);rows=[]
    for delta in np.linspace(-.12,.12,49):
        ref=np.column_stack([np.interp(t+delta,ts,xyz[:,k]) for k in range(3)])-origin+binding['position_enu']
        norm=np.linalg.norm(p-ref,axis=1)
        rows.append(dict(reference_offset_ms=float(delta*1000),rmse_m=float(np.sqrt(np.mean(norm*norm))),maximum_m=float(norm.max())))
    return dict(scope='truth-only diagnostic time-offset fit on moving core; excluded from all acceptance and published timestamps',
        best_rmse=min(rows,key=lambda r:r['rmse_m']),original_time=rows[24],samples=len(selected),grid=rows)


def diagnose(folder,limit=.10):
    read=lambda n:json.loads((folder/n).read_text())
    result=read('result.json');manifest=read('manifest.json');truth=read('truth.json')
    binding=result.get('real_pose_fusion',{}).get('alignment')
    trigger=read('fixture-trigger.json')
    if not binding or not trigger:raise ValueError('Missing actual ground binding or fixture trigger')
    outputs=read('fusion-outputs.json');poses=[]
    for record in outputs:
        m=record['aligned'];p=m['pose']['pose'];s=m['header']['stamp']
        poses.append(dict(stamp=s['sec']+s['nanosec']/1e9,position=[p['position'][k] for k in ('x','y','z')],
            quaternion=[p['orientation'][k] for k in ('x','y','z','w')]))
    rows=match(truth,poses,binding);start=trigger['ros']
    phases=dict(initial_rest=[r for r in rows if r['stamp']<start],
        motion=[r for r in rows if start<=r['stamp']<=start+100],
        final_rest=[r for r in rows if r['stamp']>=start+105])
    phases={name:summarize(values,limit) for name,values in phases.items()}
    phases['all']=summarize(rows,limit)
    ref=np.asarray([r['reference_position_m'] for r in rows])
    traveled=float(np.linalg.norm(np.diff(ref,axis=0),axis=1).sum())
    final=[r for r in rows if r['stamp']>=start+105]
    final_return=stats([np.linalg.norm(r['position_error_m']) for r in final])
    from vio_covariance_consistency import analyze
    try:
        covariance=analyze(folder)
        cov={k:v for k,v in covariance.items() if k!='rows'}
    except ValueError as exc:cov=dict(status='FAIL',reason=str(exc))
    # EKF measurements retain original sample timestamps; no delay fitting.
    ts=np.asarray([r['stamp'] for r in truth]);xyz=np.asarray([r['position'] for r in truth])
    telemetry=read('fusion-telemetry.json');ekf=[]
    for r in telemetry:
        if r['name']!='local':continue
        m=r['message'];t=m['timestamp_sample']/1e6
        if not ts[0]<=t<=ts[-1]:continue
        position=[m['y'],m['x'],-m['z']]
        # EKF yaw is NED yaw; position is the subject of this EKF diagnostic.
        ekf.append(dict(stamp=t,position=position,quaternion=[0,0,0,1],
            state_valid=bool(m['xy_valid'] and m['z_valid'] and not m['dead_reckoning'])))
    ekf_unique={r['stamp']:r for r in ekf};ekf=sorted(ekf_unique.values(),key=lambda r:r['stamp'])
    ekf_rows=match(truth,ekf,binding)
    ekf_phases={name:summarize([r for r in ekf_rows if (start<=r['stamp']<=start+100 if name=='motion' else start+105<=r['stamp']<=rows[-1]['stamp'])],limit)
        for name in ('motion','final_rest')}
    ekf_phases['after_vio_stop']=summarize([r for r in ekf_rows if r['stamp']>rows[-1]['stamp']],limit)
    for value in ekf_phases.values():
        value.pop('orientation_deg',None);value.pop('yaw_deg',None)
    raw=read('sdk-poses.json');raw_aligned=[]
    a=np.asarray(binding['rotation']);d=np.asarray(binding['translation']);ar=Rotation.from_matrix(a)
    for p in raw:
        raw_aligned.append(dict(stamp=p['stamp'],position=(a@p['position']+d).tolist(),
            quaternion=(ar*Rotation.from_quat(p['quaternion'])).as_quat().tolist()))
    raw_rows=match(truth,raw_aligned,binding)
    pt=np.asarray([p['stamp'] for p in poses]);raw_t=np.asarray([p['stamp'] for p in raw])
    state=result['sdk16_inertial_state'];native=read('sdk16-inertial-state.json')
    valid_native=[r for r in native if len(r)==14 and r[1]==1]
    post=[r for r in valid_native if r[0]/1e9>=start+105]
    bias=dict(samples=len(post),accelerometer_bias_m_s2_median=np.median([r[11:14] for r in post],axis=0).tolist() if post else None,
        gyro_bias_rad_s_median=np.median([r[8:11] for r in post],axis=0).tolist() if post else None,
        scope='native estimator state; ideal simulated IMU has no injected bias; real OAK-D noise/calibration not verified')
    source_checks=dict(candidate_diagnostic=result.get('sdk16_candidate_checks_passed') is True,
        actual_dynamic=result.get('same_aircraft_dynamic',{}).get('passed') is True,
        actual_joint_fusion=result.get('joint_observation',{}).get('passed') is True,
        cleanup=result.get('cleanup_confirmed') is True,
        disarmed=result.get('arming_states')==[1],
        complete_final_rest=bool(final and final[-1]['stamp']-final[0]['stamp']>=20),
        truth_dense=all(v.get('truth_bracket_max_gap_s',1)<=.03 for v in phases.values()),
        pose_continuous=bool(len(pt)>1 and np.max(np.diff(pt))<=.081))
    accuracy_checks=dict(vio_all_samples=phases['all']['maximum_position_within_limit'],
        vio_motion=phases['motion']['maximum_position_within_limit'],
        vio_return=phases['final_rest']['maximum_position_within_limit'],
        raw_sdk_all_samples=summarize(raw_rows,limit)['maximum_position_within_limit'],
        ekf_motion=ekf_phases['motion']['maximum_position_within_limit'],
        ekf_return=ekf_phases['final_rest']['maximum_position_within_limit'])
    files=['manifest.json','result.json','truth.json','fusion-outputs.json','fusion-telemetry.json',
        'sdk-poses.json','fixture-trigger.json','sdk16-inertial-state.json','normalizer-timing.json','fusion-timing.json']
    return dict(schema=1,run_id=folder.name,selected_sdk=manifest.get('selected_sdk'),limit_m=limit,
        flight_authorized=False,scope='simulated sensors, disarmed physical fixture; fixed ground position/yaw binding; no full trajectory/roll/pitch/scale/time-offset fit',
        source_checks=source_checks,accuracy_checks=accuracy_checks,
        accuracy_status='PASS' if all(source_checks.values()) and all(accuracy_checks.values()) else 'FAIL',
        phases=phases,raw_sdk=summarize(raw_rows,limit),ekf=ekf_phases,
        ekf_valid_samples_fraction=float(np.mean([r['state_valid'] for r in ekf if start<=r['stamp']<=start+100])),
        returned_position_error_m=final_return,
        actual_traveled_distance_m=traveled,reference_extent_m=np.ptp(ref,axis=0).tolist(),
        relative_error_1s=relative_errors(rows,1.),relative_error_10s=relative_errors(rows,10.),
        achieved_real_time_factor=(poses[-1]['stamp']-poses[0]['stamp'])/(outputs[-1]['mono']-outputs[0]['mono']),
        maximum_frame_to_frame_error_increment_m=float(np.linalg.norm(np.diff(np.array([r['position_error_m'] for r in rows]),axis=0),axis=1).max()),
        max_output_sample_gap_s=float(np.max(np.diff(pt))) if len(pt)>1 else None,
        max_raw_sample_gap_s=float(np.max(np.diff(raw_t))) if len(raw_t)>1 else None,
        initialization_delay_source_s=(state['first_initialized_ns']-state['first_track_ns'])/1e9,
        timing=timing(folder),native_final_rest_bias=bias,covariance=cov,
        diagnostic_time_offset_probe=dict(vio=time_offset_probe(truth,poses,binding,start),ekf=time_offset_probe(truth,ekf,binding,start)),
        source_stop=result.get('real_pose_fusion',{}).get('first_stop_rejection'),
        measurement_rows=rows,ekf_measurement_rows=[r for r in ekf_rows if r['stamp']>=start],
        input_sha256={n:hashlib.sha256((folder/n).read_bytes()).hexdigest() for n in files},
        diagnostic_tool_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--limit-m',type=float,default=.10)
    parser.add_argument('--out',type=Path,required=True)
    args=parser.parse_args()
    if not np.isfinite(args.limit_m) or args.limit_m<=0:parser.error('Positive finite limit required')
    result=diagnose(args.run,args.limit_m)
    args.out.parent.mkdir(parents=True,exist_ok=True)
    args.out.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:result[k] for k in ('run_id','accuracy_status','accuracy_checks','phases','ekf','max_output_sample_gap_s')},indent=2))


if __name__=='__main__':main()
