#!/usr/bin/env python3
"""Fail-closed VIO candidate acceptance receipt; publishes no ROS controls."""

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
from vio_covariance_consistency import analyze


def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()


def moving_covariance_screen(rows,start):
    """Long static intervals must not hide undercoverage during actual motion."""
    values=np.asarray([r['nees'] for r in rows if start+12<=r['stamp']<=start+88])
    coverage95=float(np.mean(values<=12.591587243743977)) if len(values) else None
    coverage99=float(np.mean(values<=16.811893829770927)) if len(values) else None
    return dict(passed=bool(len(values)>=1000 and np.isfinite(values).all() and
        coverage95>=.90 and coverage99>=.98),samples=len(values),
        coverage95=coverage95,coverage99=coverage99,minimum_coverage95=.90,minimum_coverage99=.98,
        scope='additional moving-phase rejection screen with the same coverage floors; no independent-trial claim')


def acceptance(folder,root,*,prior_covariance=None,frozen_archive=None,frozen_index=None):
    read=lambda name:json.loads((folder/name).read_text())
    manifest=read('manifest.json');result=read('result.json');covariance=analyze(folder)
    integrity={name:bool((root/name).is_file() and digest(root/name)==expected)
               for name,expected in manifest['input_sha256'].items()}
    frozen=None
    if frozen_archive is not None:
        from freeze_vio_run import verify
        frozen=verify(frozen_archive,frozen_index or frozen_archive.with_suffix('.index.json'))
        if frozen['run_id']!=folder.name:raise ValueError('Frozen evidence belongs to a different run')
        members=frozen['members']
        for name in ('manifest.json','result.json','truth.json','fusion-outputs.json','fusion-telemetry.json','joint-native-receipts.json'):
            if members.get('runs/'+folder.name+'/'+name)!=digest(folder/name):
                raise ValueError('Live output differs from frozen evidence: '+name)
        integrity={name:members.get('inputs/'+name)==expected for name,expected in manifest['input_sha256'].items()}
    dynamic=result.get('same_aircraft_dynamic',{})
    checks=dict(actual_same_aircraft=manifest.get('same_aircraft_motion') is True and not manifest['motion'],
        source_candidate=result.get('estimator_candidate_checks_passed',result.get('sdk16_candidate_checks_passed')) is True,
        actual_dynamic_fusion=dynamic.get('passed') is True,
        actual_pose_fusion=result.get('real_pose_fusion',{}).get('passed') is True,
        joint_fusion=result.get('joint_observation',{}).get('passed') is True,
        cleanup=result.get('cleanup_confirmed') is True,
        frozen_inputs=bool(integrity) and all(integrity.values()),
        unknown_ev_velocity=result.get('real_pose_fusion',{}).get('checks',{}).get('unknown_velocity') is True,
        no_arming=result.get('arming_states')==[1],
        fixed5=manifest['profile']['camera_pitch_deg']==5,
        position_accuracy=covariance['position_rmse_m']<=.15)
    # Actual optical-flow/range fusion must advance during the moving portion,
    # not merely before it or in the last five stationary CLI snapshots.
    start=dynamic.get('trigger',{}).get('ros',float('inf'))
    native=[]
    for row in read('joint-native-receipts.json'):
        t=row['topics'].get('vehicle_status',{}).get('_observed_now_us',0)/1e6
        if row['phase']=='healthy' and start+12<=t<=start+88:native.append(row['topics'])
    checks['moving_mtf_receipts']=len(native)>=20
    checks['no_moving_gnss_or_ev_velocity']=bool(native and all(
        all(r['estimator_status_flags'].get(k)==False for k in
            ('cs_gnss_pos','cs_gnss_vel','cs_gnss_yaw','cs_gps_hgt','cs_ev_vel')) for r in native))
    outputs={r['ev']['timestamp_sample']:r['ev'] for r in read('fusion-outputs.json')}
    native_ev=[r.get('vehicle_visual_odometry',{}) for r in native]
    native_pairs=[(r,outputs.get(round(r.get('timestamp_sample',-1)))) for r in native_ev]
    topic_receipt=read('truth-topic-isolation.json') if (folder/'truth-topic-isolation.json').is_file() else {}
    topic_info=topic_receipt.get('topics',{})
    checks['gazebo_truth_receipt_identity']=bool(topic_receipt.get('run_id')==manifest['run_id'] and
        topic_receipt.get('partition')==manifest['partition'] and topic_receipt.get('passed',True) is True)
    default_topic='/model/'+manifest['model']+'/odometry_with_covariance'
    checks['gazebo_default_truth_no_publisher']=bool(topic_info.get(default_topic,{}).get('exit_code')==0 and
        'No publishers on topic' in topic_info.get(default_topic,{}).get('stdout',''))
    checks['gazebo_truth_covariance_observer_only']=bool(
        'No subscribers on topic' in topic_info.get('/vio/truth_covariance_observer_only',{}).get('stdout',''))
    checks['native_ev_is_only_actual_vio']=bool(native_pairs and all(
        ev is not None and r.get('quality')==100 and r.get('velocity_frame')==0 and
        all(r.get(k)==[None,None,None] for k in ('velocity','velocity_variance','angular_velocity')) and
        np.allclose(r.get('position',[]),ev['position'],atol=2e-5,rtol=0) and
        np.allclose(r.get('q',[]),ev['q'],atol=2e-5,rtol=0) for r,ev in native_pairs))
    mtf={}
    for name in ('optical_flow','rng_hgt'):
        aid=[r['estimator_aid_src_'+name] for r in native]
        stamps=[r.get('time_last_fuse',0) for r in aid]
        ok=bool(len(aid)>=20 and all(r.get('fused')==1 and r.get('innovation_rejected')==0 for r in aid)
                and all(a<b for a,b in zip(stamps,stamps[1:])))
        checks['moving_'+name+'_fusion']=ok
        nis=[]
        for r in aid:
            innovation=np.atleast_1d(r.get('innovation',[]));variance=np.atleast_1d(r.get('innovation_variance',[]))
            if len(innovation)==len(variance)>0 and np.isfinite(innovation).all() and np.all(variance>0):
                nis.append(float(np.sum(innovation**2/variance)))
        mtf[name]=dict(receipts=len(aid),actually_fused=ok,nis_median=float(np.median(nis)) if nis else None,
                       nis_max=max(nis) if nis else None)
    ev={}
    telemetry=read('fusion-telemetry.json')
    for name in ('ev_pos','ev_hgt','ev_yaw'):
        records=[r['message'] for r in telemetry if r['name']==name and
                 start+12<=r['message']['timestamp_sample']/1e6<=start+88]
        nis=[]
        for r in records:
            v=np.atleast_1d(r['innovation_variance']);e=np.atleast_1d(r['innovation'])
            if np.isfinite(e).all() and np.all(v>0):nis.append(float(np.sum(e*e/v)))
        ev[name]=dict(samples=len(records),nis_median=float(np.median(nis)) if nis else None,
                      nis_max=max(nis) if nis else None,
                      innovation_rejected=sum(bool(r['innovation_rejected']) for r in records))
    truth=read('truth.json');ts=np.asarray([r['stamp'] for r in truth]);xyz=np.asarray([r['position'] for r in truth])
    binding=result['real_pose_fusion']['alignment'];t0=binding['sample_stamp']
    origin=np.asarray([np.interp(t0,ts,xyz[:,k]) for k in range(3)])
    errors=[];valid=[]
    for row in telemetry:
        if row['name']!='local':continue
        m=row['message'];t=m['timestamp_sample']/1e6
        if not (max(ts[0],start+12)<=t<=min(ts[-1],start+88)):continue
        reference=np.asarray([np.interp(t,ts,xyz[:,k]) for k in range(3)])-origin+binding['position_enu']
        position=np.asarray([m['y'],m['x'],-m['z']])
        errors.append(float(np.linalg.norm(position-reference)))
        valid.append(bool(m['xy_valid'] and m['z_valid'] and not m['dead_reckoning']))
    ekf=dict(samples=len(errors),position_rmse_m=float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
        position_max_m=max(errors) if errors else None,initial_launch_reference_only=True,trajectory_fit=False)
    checks['moving_ekf_accuracy']=bool(len(errors)>=1000 and all(valid) and ekf['position_rmse_m']<=.15 and ekf['position_max_m']<=.30)
    current_screen=covariance['rejection_screen_passed']
    moving_screen=moving_covariance_screen(covariance['rows'],start)
    prior_screen=None if prior_covariance is None else prior_covariance['rejection_screen_passed']
    # A temporal trajectory alone cannot certify global covariance calibration.
    # Fail if either fixed-gauge screen rejects; otherwise remain INCONCLUSIVE
    # until independent-run/gauge observability qualification is supplied.
    cov_status='FAIL' if not current_screen or prior_screen is False or not moving_screen['passed'] else 'INCONCLUSIVE'
    receipt=dict(schema=1,sdk=('OpenVINS '+result['estimator_candidate_contract']['source_commit'] if manifest.get('estimator')=='openvins' else result.get('sdk16_candidate_contract',{}).get('sdk_version','16.0.0+0558842')),run_id=folder.name,
        overall='FAIL' if not all(checks.values()) or cov_status=='FAIL' else 'INCONCLUSIVE',
        same_aircraft_dynamic=dict(status='PASS' if all(checks.values()) else 'FAIL',checks=checks,mtf=mtf,ev=ev,ekf_motion=ekf),
        global_covariance=dict(status=cov_status,current_fixed_gauge_screen=current_screen,
            additional_moving_phase_screen=moving_screen,
            prior_fixed_gauge_screen=prior_screen,prior_covariance_supplied=prior_covariance is not None,
            independent_trials_available=False,
            absolute_global_observability=False,statistical_calibration_proven=False),
        flight=dict(status='NOT_RUN',authorized=False,reason='Estimator global covariance qualification not passed',
            arming_states=result.get('arming_states'),controls=result.get('real_pose_fusion',{}).get('control_publishers')),
        analysis_tool_sha256=digest(Path(__file__)),input_integrity=integrity,files={n:digest(folder/n) for n in ('manifest.json','result.json','truth.json','fusion-outputs.json','fusion-telemetry.json','joint-native-receipts.json')},
        frozen_evidence=dict(archive=str(frozen_archive),sha256=frozen['archive_sha256'],members_verified=len(frozen['members'])) if frozen else None,
        native_truth_isolation=topic_receipt)
    return receipt,covariance


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--root',type=Path,default=Path(__file__).resolve().parents[4])
    parser.add_argument('--prior-covariance',type=Path)
    parser.add_argument('--frozen-archive',type=Path)
    parser.add_argument('--frozen-index',type=Path)
    args=parser.parse_args();prior=json.loads(args.prior_covariance.read_text()) if args.prior_covariance else None
    receipt,covariance=acceptance(args.folder,args.root,prior_covariance=prior,frozen_archive=args.frozen_archive,frozen_index=args.frozen_index)
    args.out.mkdir(parents=True,exist_ok=True)
    (args.out/'acceptance.json').write_text(json.dumps(receipt,indent=2)+'\n')
    (args.out/'covariance-consistency.json').write_text(json.dumps(covariance,indent=2)+'\n')
    print(json.dumps({k:v for k,v in receipt.items() if k not in ('input_integrity','files')},indent=2))
    return 0 if receipt['overall']=='PASS' else 1


if __name__=='__main__':raise SystemExit(main())
