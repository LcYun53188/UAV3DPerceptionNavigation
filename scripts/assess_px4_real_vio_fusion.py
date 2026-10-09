#!/usr/bin/env python3
"""Independently inspect saved source/output/EKF chronology, without ROS or VIO code."""
import argparse
import gzip
import hashlib
import json
import math
from pathlib import Path
import numpy as np


def read(folder,name):
    path=folder/name
    if path.is_file():return json.loads(path.read_text())
    return json.loads(gzip.decompress(path.with_suffix(path.suffix+'.gz').read_bytes()))


def ns(t):return t['sec']*1_000_000_000+t['nanosec']


def assess(folder):
    result=read(folder,'result.json');f=result['real_pose_fusion'];manifest=read(folder,'manifest.json')
    outputs=read(folder,'fusion-outputs.json');echoes=read(folder,'fusion-dds-echoes.json')
    history=read(folder,'fusion-telemetry.json');poses=read(folder,'normalized-poses.json')
    raw=read(folder,'sdk-poses.json');statuses=read(folder,'normalized-status.json')
    source={round(p['stamp']*1e9):p for p in poses};sdk={round(p['stamp']*1e9):p for p in raw}
    alignment=f.get('alignment') or {};stop=f.get('stop_request_mono')
    checks=dict(owned_disarmed=manifest['fuse_pose'] and not manifest['motion'] and not manifest['reset_source'],
        completed=result['passed'] and result['cleanup_confirmed'] and not result['remaining_owned_processes'],
        audited_checks=all(f['checks'].values()) and all(result['checks'].values()),
        normalized_checks=result['normalized_pose']['passed'],
        source_identity=bool(alignment) and alignment.get('calibration_id')==manifest['calibration_id']
            and all(s['session']==alignment.get('source_session') and s['reset']==alignment.get('reset_counter')
                    and s['calibration']==alignment.get('calibration_id') for s in statuses),
        sdk_normalization=bool(outputs),original_samples=bool(outputs),coordinate_conversion=bool(outputs),covariance_propagation=bool(outputs),
        echo_samples=bool(outputs),unknown_velocity=bool(echoes),
        actual_three_aids=True,local_health=False,flags=False,stopped=False)
    if not alignment or stop is None:return dict(passed=False,checks=checks)
    record={k:v for k,v in alignment.items() if k!='alignment_id'}
    checks['alignment_hash']=hashlib.sha256(json.dumps(record,sort_keys=True,separators=(',',':')).encode()).hexdigest()==alignment['alignment_id']
    r=np.asarray(alignment['rotation']);offset=np.asarray(alignment['translation'])
    c0=np.asarray(alignment['initial_covariance']);yaw_j=np.asarray(alignment['initial_yaw_jacobian'])
    anchor_stamp=round(alignment['sample_stamp']*1e9);anchor_source=source.get(anchor_stamp)
    checks['anchor_is_original']=anchor_source is not None
    if anchor_source is None:return dict(passed=False,checks=checks)
    checks['anchor_covariance']=np.allclose(c0,np.asarray(anchor_source['covariance']).reshape(6,6),rtol=0,atol=1e-10)
    p0=np.asarray(anchor_source['position']);last_stamp=anchor_stamp;ev_samples={}
    for output in outputs:
        aligned,ev=output['aligned'],output['ev'];t=ns(aligned['header']['stamp']);p=source.get(t);raw_p=sdk.get(t)
        checks['original_samples'] &= (p is not None and raw_p is not None and t>last_stamp
            and ev['timestamp_sample']==round(t/1000) and -.05<=(ev['timestamp']-ev['timestamp_sample'])/1e6<=.2
            and output['mono']<=stop+.5)
        last_stamp=t;ev_samples[ev['timestamp_sample']]=ev
        if p is None:continue
        pos=aligned['pose']['pose']['position'];enu=np.array([pos[n] for n in ('x','y','z')])
        checks['coordinate_conversion'] &= (aligned['header']['frame_id']=='px4_local_enu'
            and np.allclose(enu,r@p['position']+offset,rtol=0,atol=1e-8)
            and np.allclose(ev['position'],[enu[1],enu[0],-enu[2]],rtol=0,atol=1e-6))
        q=aligned['pose']['pose']['orientation'];original_q=np.asarray(p['quaternion'])
        x,y,z,w=original_q/np.linalg.norm(original_q)
        if raw_p is not None:
            basis=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
            transform=np.zeros((6,6));transform[:3,:3]=transform[3:,3:]=basis
            body=np.asarray(raw_p['covariance']).reshape(6,6)
            normalized=transform@((body+body.T)/2)@transform.T
            raw_q=np.asarray(raw_p['quaternion']);raw_q=raw_q/np.linalg.norm(raw_q)
            checks['sdk_normalization'] &= bool(np.allclose(p['position'],raw_p['position'],rtol=0,atol=1e-10)
                and np.allclose(p['quaternion'],raw_q,rtol=0,atol=1e-10)
                and np.allclose(np.asarray(p['covariance']).reshape(6,6),normalized,rtol=1e-8,atol=1e-10))
        delta=math.atan2(r[1,0],r[0,0]);co,si=math.cos(delta/2),math.sin(delta/2)
        expected_q=[co*x-si*y,co*y+si*x,co*z+si*w,co*w-si*z]
        checks['coordinate_conversion'] &= np.allclose([q[n] for n in ('x','y','z','w')],expected_q,rtol=0,atol=1e-8)
        root=math.sqrt(.5)
        checks['coordinate_conversion'] &= np.allclose(ev['q'],[root*(q['w']+q['z']),root*(q['x']+q['y']),
            root*(q['x']-q['y']),root*(q['w']-q['z'])],rtol=0,atol=1e-6)
        j=np.zeros((6,6));j[:3,:3]=j[3:,3:]=r
        d=r@(np.asarray(p['position'])-p0);initial=np.zeros((6,6));initial[:3,:3]=-r
        initial[:3,3:]=-np.outer(np.cross([0,0,1],d),yaw_j);initial[3:,3:]=-np.outer([0,0,1],yaw_j)
        expected=2*(j@np.asarray(p['covariance']).reshape(6,6)@j.T+initial@c0@initial.T)
        observed=np.asarray(aligned['pose']['covariance']).reshape(6,6)
        checks['covariance_propagation'] &= bool(np.allclose(expected,observed,rtol=1e-8,atol=1e-10)
            and np.linalg.eigvalsh(observed).min()>=-1e-8 and np.all(np.diag(observed)>0) and np.all(np.diag(observed)<=.25))
    echo_counts={}
    for e in echoes:
        ev=e['message'];t=ev['timestamp_sample'];echo_counts[t]=echo_counts.get(t,0)+1
        checks['echo_samples'] &= t in ev_samples and ev==ev_samples.get(t)
        checks['unknown_velocity'] &= (ev['velocity_frame']==0 and all(v is None for n in
            ('velocity','velocity_variance','angular_velocity') for v in ev[n]))
    checks['echo_samples'] &= len(echo_counts)==len(ev_samples) and all(v==1 for v in echo_counts.values())
    for n in ('ev_pos','ev_hgt','ev_yaw'):
        samples=[e for e in history if e['name']==n]
        fused=[e for e in samples if e['mono']<stop and e['message']['fused'] and not e['message']['innovation_rejected']]
        drained=[e for e in samples if e['mono']>=stop+1.]
        checks['actual_three_aids'] &= (len(fused)>=50 and len({e['message']['time_last_fuse'] for e in fused})>=50
            and samples[-1]['message']['time_last_fuse']==f['last_fuse_after_drain'][n]==f['last_fuse_at_end'][n]
            # PX4 publishes these aid sources on new observations. After the
            # input stops, absence of further aid samples is expected.
            and all(e['message']['time_last_fuse']==f['last_fuse_at_end'][n] for e in drained))
    checks['no_ev_velocity']=not any(e['message']['fused'] for e in history if e['name']=='ev_vel')
    local_entries=[e for e in history if e['name']=='local' and stop-5<=e['mono']<stop]
    locals_=[e['message'] for e in local_entries]
    span=(locals_[-1]['timestamp']-locals_[0]['timestamp'])/1e6 if len(locals_)>1 else 0.
    checks['local_health']=(len(locals_)>1 and local_entries[-1]['mono']-local_entries[0]['mono']>=4.8
        and span>0 and 90<=(len(locals_)-1)/span<=110) and all(
        all(m[n] for n in ('xy_valid','z_valid','v_xy_valid','v_z_valid','heading_good_for_control'))
        and not m['dead_reckoning'] and all(math.isfinite(m[n]) and 0<m[n]<=.5 for n in ('eph','epv','evh','evv'))
        and 0<m['heading_var']<=.25 for m in locals_)
    flags=f['before_stop']['flags']
    checks['flags']=all(flags['cs_'+n] for n in ('ev_pos','ev_hgt','ev_yaw')) and flags['cs_baro_hgt'] and not any(
        flags['cs_'+n] for n in ('ev_vel','gnss_pos','gnss_vel','gnss_yaw','gps_hgt','mag','mag_hdg','mag_3d','opt_flow','rng_hgt','aux_gpos'))
    first=f['first_stop_rejection']
    checks['stopped']=first is not None and first['after_stop_s']<=.5 and first['reason'] in (
        'VIO_SOURCE_LOST','VIO_SOURCE_INVALID','VIO_TELEMETRY_STALE:source') and f['source_fault']!='' and f['gate_reason']!='READY'
    checks['no_control_writers']=not any(f['control_publishers'].values()) and not f['graph_violations']
    return dict(passed=all(checks.values()),checks={k:bool(v) for k,v in checks.items()},
        input_samples=len(outputs),echo_samples=len(echoes),scope='archived actual source/output/EKF chronology; disarmed only')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path);args=parser.parse_args()
    result=assess(args.folder);print(json.dumps(result,indent=2));raise SystemExit(0 if result['passed'] else 1)
