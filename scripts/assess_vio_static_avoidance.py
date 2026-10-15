#!/usr/bin/env python3
"""Audit actual EGO/truth evidence; an empty-path mission cannot pass avoidance."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.interpolate import BSpline
from sim_validation import file_hash, write_json


def clearance(points, obstacles):
    points=np.asarray(points,float)
    if points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.all(np.isfinite(points)):
        raise ValueError('Missing finite trajectory points')
    closest=np.full(len(points),np.inf)
    for obstacle in obstacles:
        delta=np.maximum(np.abs(points-np.asarray(obstacle['center']))-np.asarray(obstacle['size'])/2,0)
        closest=np.minimum(closest,np.linalg.norm(delta,axis=1))
    return float(closest.min())


def px4_return_verified(references,events):
    targets=[e['target_enu'] for e in events if e['phase']=='RETURN']
    if not references or len(targets)!=len(references):return False
    try:
        for reference,target in zip(references,targets):
            raw=np.asarray(reference['native_position_ned'],float)
            enu=np.asarray(reference['position_enu'],float)
            if (reference['source']!='PX4_HOME_POSITION' or reference['timestamp']<=0 or
                    raw.shape!=(3,) or enu.shape!=(3,) or not np.all(np.isfinite(raw)) or
                    not np.allclose(enu,[raw[1],raw[0],-raw[2]],atol=1e-7,rtol=0) or
                    not np.allclose(target[:2],enu[:2],atol=1e-6,rtol=0)):return False
    except (KeyError,TypeError,ValueError):return False
    return True


def assess(directory):
    directory=Path(directory).resolve()
    read=lambda name:json.loads((directory/name).read_text())
    manifest=read('manifest.json');flight=read('flight-observation.json');result=read('result.json')
    layout=read('assets/warehouse-layout.json')['layout']
    region=read('consumed-inputs/simulation/safe_regions/vio_avoidance.json')
    obstacles=layout['obstacles']
    plans=read('accepted-plans.json');truth=read('flight-truth.json')
    body=region['body_radius_m']+region['tracking_margin_m']
    checks=dict(correct_scene=layout['scene_id']=='vio_warehouse_avoidance_v1',
        actual_vio_ego=flight.get('vio_required') and flight.get('navigation_backend')=='EGO' and not flight.get('mock',True),
        complete_mission=flight.get('passed') is True and flight.get('action_status')==4 and result.get('experimental_mission_passed') is True,
        landed_disarmed=flight.get('final_land') is True and flight.get('final_arming')==1,
        owned_forward_and_return_plans=len(plans)>=2,
        no_gnss_joint_profile=all(manifest['px4_parameter_overrides'].get('PX4_PARAM_'+k)==str(v) for k,v in
            dict(EKF2_GPS_CTRL=0,EKF2_EV_CTRL=11,EKF2_OF_CTRL=1,EKF2_RNG_CTRL=2).items()))
    if region.get('return_home_source')=='PX4_HOME_POSITION':
        checks['return_uses_px4_home']=px4_return_verified(flight.get('return_home_references'),read('flight-events.json'))
    evidence=[]
    shift=np.asarray(flight['home_truth'])-np.asarray(flight['home_enu'])
    for plan in plans:
        auth=plan['authorization'];t=plan['bound']['trajectory'];context=plan['bound']['context']
        controls=np.array([[p[k] for k in ('x','y','z')] for p in t['control_points']])
        curve=BSpline((np.arange(len(controls)+4)-3)*t['knot_interval'],controls,3,extrapolate=False)
        points=curve(np.linspace(0,float(curve.t[-4]),2001))
        line=np.linspace(points[0],points[-1],2001)
        physical_clearance=clearance(points+shift,obstacles)
        direct_clearance=clearance(line+shift,obstacles)
        evidence.append(dict(step_index=auth['step_index'],trajectory_id=t['trajectory_id'],
            duration_s=float(curve.t[-4]),direct_clearance_m=direct_clearance,
            planned_clearance_m=physical_clearance,
            ownership_valid=auth['coordinator_instance']==context['localization_session']==manifest['run_id'],
            max_deviation_from_straight_m=float(np.linalg.norm(points-line,axis=1).max())))
    forward=next((p for p in evidence if p['step_index']==2),None)
    checks['straight_route_actually_blocked']=bool(forward and forward['direct_clearance_m']<body)
    checks['planned_routes_clear']=bool(evidence) and all(p['planned_clearance_m']>=body and p['ownership_valid'] for p in evidence)
    flying=[p['position'] for p in truth if p['phase'] in ('NAVIGATE','RETURN')]
    actual_clearance=clearance(flying,obstacles) if flying else None
    checks['actual_aircraft_routes_clear']=actual_clearance is not None and actual_clearance>=body
    # Occupancy evidence must be observed obstacle cells, not simply unknown space.
    blocked_observed_cells=0
    if forward:
        grid_path=directory/'observation-grid-returned.npz'
        metadata=read('observation-grid-returned.json')
        checks['frozen_map_hash']=file_hash(grid_path)==metadata['grid_sha256']
        with np.load(grid_path,allow_pickle=False) as grid:
            origin=grid['origin'];resolution=float(grid['resolution']);distance=grid['distance'];observed=grid['observed']
            crate=next(o for o in obstacles if o['name']=='avoidance_crate')
            lo=np.floor((np.asarray(crate['center'])-np.asarray(crate['size'])/2-shift-origin)/resolution).astype(int)
            hi=np.floor((np.asarray(crate['center'])+np.asarray(crate['size'])/2-shift-origin)/resolution).astype(int)
            lo=np.maximum(lo,0);hi=np.minimum(hi,np.asarray(distance.shape)-1)
            sl=tuple(slice(a,b+1) for a,b in zip(lo,hi))
            blocked_observed_cells=int(np.count_nonzero(observed[sl] & np.isfinite(distance[sl]) & (distance[sl]<=0)))
    checks['depth_map_contains_route_obstacle']=blocked_observed_cells>0
    return dict(passed=all(checks.values()),checks=checks,plans=evidence,
        body_and_tracking_radius_m=body,actual_min_clearance_m=actual_clearance,
        route_obstacle_observed_occupied_cells=blocked_observed_cells,
        scope='Static simulation obstacle; source covariance qualification deferred; no dynamic or hardware avoidance acceptance')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path);args=parser.parse_args()
    result=assess(args.directory);write_json(args.directory/'static-avoidance-assessment.json',result)
    print(json.dumps(result,indent=2));return 0 if result['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
