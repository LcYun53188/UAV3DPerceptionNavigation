#!/usr/bin/env python3
"""Accept a completed simulated mission against an explicit positioning limit."""
import argparse
import hashlib
import json
import math
from pathlib import Path


def assess(mission, errors, native, loaded, limit):
    def position_ok(key):
        row=errors.get(key) or {}
        value=row.get('max_m')
        return bool(row.get('samples',0)>=1000 and isinstance(value,(int,float))
                    and math.isfinite(value) and 0<=value<limit)
    flags=native.get('flags',{})
    def flag_is(key,enabled):
        row=flags.get(key,{})
        count=row.get('in_air_records',0)
        return count>0 and row.get('true_in_air_records')==(count if enabled else 0)
    aids=native.get('aids',{})
    def aid_ok(key):
        row=aids.get(key,{})
        ratio=row.get('test_ratio_max',float('nan'))
        return (row.get('fused_samples',0)>100 and row.get('innovation_rejected_samples')==0
                and isinstance(ratio,(int,float)) and math.isfinite(ratio) and 0<=ratio<1)
    checks=dict(actual_full_mission=bool(mission.get('passed') and mission.get('mock') is False
                and mission.get('bt_exit_code')==0 and mission.get('bt_dispatch_count')==1
                and mission.get('bt_step_accepts')==list(range(7))
                and mission.get('bt_step_completes')==list(range(7))),
        landed_disarmed=mission.get('final_land') is True and mission.get('final_arming')==1,
        sdk16_position=position_ok('sdk16_aligned_position'),px4_position=position_ok('px4_position'),
        waypoint_positions=bool(mission.get('waypoint_truth')) and all(
            isinstance(x.get('error_m'),(int,float)) and math.isfinite(x['error_m']) and 0<=x['error_m']<limit
            for x in mission.get('waypoint_truth',[])),
        actual_joint_fusion=all(flag_is(k,True) for k in ('cs_ev_pos','cs_ev_hgt','cs_ev_yaw','cs_opt_flow','cs_rng_hgt')),
        no_gnss=all(flag_is(k,False) for k in ('cs_gnss_pos','cs_gps_hgt','cs_gnss_yaw')),
        flow_range_innovations=all(aid_ok(k) for k in ('estimator_aid_src_optical_flow','estimator_aid_src_rng_hgt')),
        actual_sdk16=loaded.get('passed') is True and bool(loaded.get('loaded_libraries_sha256')) and all(
            '/v16.0.0-' in p for p in loaded.get('loaded_libraries_sha256',{})))
    return dict(passed=all(checks.values()),checks=checks,max_position_error_limit_m=limit,
        scope='one complete SDK16 joint simulated mission; receipt-time positioning screening',
        trajectory_tracking_requirement='reported separately, not accepted by the positioning threshold',
        covariance_qualification='DEFERRED',active_obstacle_avoidance='NOT_RUN',hardware='NOT_RUN')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('run',type=Path);p.add_argument('--limit-m',type=float,default=.2)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    if not math.isfinite(a.limit_m) or a.limit_m<=0:p.error('limit must be finite and positive')
    names=('flight-observation.json','flight-error-screening.json','native-joint-flight-screening.json','sdk16-loaded-libraries.json')
    data=[json.loads((a.run/n).read_text()) for n in names]
    result=assess(*data,a.limit_m)
    result.update(input_sha256={n:hashlib.sha256((a.run/n).read_bytes()).hexdigest() for n in names},
                  script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
