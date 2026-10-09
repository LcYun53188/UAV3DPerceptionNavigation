"""Read-only actual warehouse BT closure audit; low flight never qualifies 1.5 m."""
import argparse
import json
import math
from pathlib import Path
from assess_vio_warehouse import read,digest_file


def assess(folder):
    m=read(folder,'manifest.json');r=read(folder,'result.json');f=read(folder,'flight-observation.json')
    events=read(folder,'flight-events.json');truth=read(folder,'flight-truth.json')
    outputs=read(folder,'fusion-outputs.json');telemetry=read(folder,'fusion-telemetry.json')
    source=read(folder,'normalized-poses.json');session=r['flight_pose_session']
    expected_hover=[step['duration_s'] for step in f.get('steps',[]) if step['type']=='HOVER']
    hovers=[]
    for e,next_e in zip(events,events[1:]):
        if e['phase']=='HOVER':
            rows=[t for t in truth if e['mono']<=t['mono']<=next_e['mono']]
            if rows:hovers.append(dict(duration_s=next_e['mono']-e['mono'],samples=len(rows),
                max_drift_m=max(math.dist(t['position'],rows[0]['position']) for t in rows)))
    source_stamps={(p['stamp']) for p in source}
    original_samples=bool(outputs)
    for o in outputs:
        stamp=o['aligned']['header']['stamp'];t=stamp['sec']+stamp['nanosec']/1e9
        original_samples &= t in source_stamps and o['ev']['timestamp_sample']==round(t*1e6)
    flags=[e['message'] for e in telemetry if e['name']=='flags']
    no_gnss=bool(flags) and all(not s['cs_gnss_pos'] and not s['cs_gnss_vel'] for s in flags)
    frames=read(folder,'assets/frames.json')
    sensor=read(folder,'sensor-preflight.json')
    config=read(folder,'flight-assets.json')
    checks=dict(raw_passed=r['passed'] is True,flight_passed=f['passed'] is True,
        actual_bt=f.get('bt_exit_code')==0 and f.get('bt_dispatch_count')==1 and
            f.get('bt_step_accepts')==f.get('bt_step_completes')==list(range(len(f.get('steps',[])))),root_success=f.get('action_status')==4 and
            f.get('result',{}).get('code')=='SUCCEEDED' and f['result'].get('cleanup_confirmed') is True and
            f['result'].get('mock') is False,
        native_land=f.get('final_land') is True and f.get('final_arming')==1 and f.get('landing_cancel_rejected') is True,
        hover_complete=len(hovers)==len(expected_hover)>0 and all(h['duration_s']>=d and h['max_drift_m']<=.15
            for h,d in zip(hovers,expected_hover)),
        hover_replay=hovers==f.get('hovers'),
        actual_displacement=f.get('max_truth_displacement_m',0)>.6 and bool(truth),
        near_home=bool(truth) and math.dist(truth[-1]['position'],f['home_truth'])<=.15,
        source_original_time=original_samples,source_healthy=not session['source_fault'],
        unique_control=not session['graph_violations'] and session['sole_gateway_observed'],
        no_gnss=no_gnss,no_ev_velocity=session['fused_samples'].get('ev_vel',0)==0,
        runtime_tf=all(sensor['checks'].get('runtime_tf:'+name) for name in frames),
        scene_assets=all(digest_file(folder/name)==digest for name,digest in config['files'].items()),
        cleanup=r['cleanup_confirmed'] is True and not r['remaining_owned_processes'])
    height=next((s['height_m'] for s in f.get('steps',[]) if s['type']=='TAKEOFF'),None)
    return dict(schema=1,run_id=m['run_id'],case='WH-F01-low' if height==.8 else 'WH-F01',
        passed=all(checks.values()),checks=checks,takeoff_height_m=height,hovers=hovers,
        planned_1p5m_hover_passed=height==1.5 and all(checks.values()),
        navigation_obstacle_avoidance_evaluated=False,
        scope='actual BT VIO closure and independently replayed truth hover; no navigation/avoidance acceptance')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory',type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args();report=assess(a.directory)
    if a.output:a.output.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
