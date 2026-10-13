"""Assess actual PX4 uORB receipts for the isolated flow/range reference."""
import math
import re


def fields(output):
    result = {}
    for line in output.splitlines():
        match = re.match(r'\s*([a-z][a-z0-9_]*):\s*(.+)', line)
        if not match:
            continue
        key, raw = match.groups()
        if key=='timestamp':
            age = re.search(r'([-+\d.]+) seconds ago', raw)
            if age:
                result['_observed_now_us'] = float(raw.split()[0]) + float(age.group(1))*1e6
        raw = raw.split('(')[0].strip()
        if raw in ('True','False'):
            result[key] = raw == 'True'
            continue
        try:
            value = [float(x.strip()) for x in raw.strip('[]').split(',')] if ',' in raw else float(raw)
            if isinstance(value, list):
                result[key] = [x if math.isfinite(x) else None for x in value]
            else:
                result[key] = value if math.isfinite(value) else None
        except ValueError:
            continue
    return result


def assessment(receipts, profile, *, with_ev=False):
    """Require fresh, accepted aid receipts over time, never just enabled flags."""
    checks = {'enough_receipts':len(receipts)>=5}
    # Discard startup; all remaining snapshots must meet each condition.
    recent = receipts[-5:]
    def every(predicate):
        return bool(recent) and all(predicate(r) for r in recent)
    def number(r,topic,key,default=-math.inf):
        value=r[topic].get(key)
        return value if isinstance(value,(float,int)) and math.isfinite(value) else default
    def fresh(r, topic, age_s):
        now = max((v.get('_observed_now_us',0) for v in r.values()), default=0)
        sample = r[topic].get('timestamp', 0)
        return now > 0 and sample is not None and sample > 0 and 0 <= now-sample < age_s*1e6
    checks['disarmed'] = every(lambda r:fresh(r,'vehicle_status',2) and r['vehicle_status'].get('arming_state')==1)
    checks['flow_received'] = every(lambda r:fresh(r,'sensor_optical_flow',.5) and r['sensor_optical_flow'].get('integration_timespan_us',0)>0)
    checks['flow_quality'] = every(lambda r:fresh(r,'vehicle_optical_flow',.5) and r['vehicle_optical_flow'].get('quality',0)>0)
    checks['range_received'] = every(lambda r:fresh(r,'distance_sensor',.5) and r['distance_sensor'].get('orientation')==25 and profile['range_min_m']<=number(r,'distance_sensor','current_distance')<=profile['range_max_m'] and r['distance_sensor'].get('max_distance')==profile['range_max_m'])
    checks['effective_limits'] = every(lambda r:r['vehicle_optical_flow'].get('max_ground_distance')==8 and abs(number(r,'vehicle_optical_flow','min_ground_distance')-.08)<1e-5 and r['vehicle_optical_flow'].get('max_flow_rate')==7)
    for aid in ('optical_flow','rng_hgt'):
        name='estimator_aid_src_'+aid
        checks[aid+'_fused'] = every(lambda r, name=name:fresh(r,name,.5) and r[name].get('fused')==1 and r[name].get('innovation_rejected')==0 and 0 <= r[name].get('_observed_now_us',0)-r[name].get('time_last_fuse',0)<500000 and r[name].get('time_last_fuse',0)>0)
        stamps=[r[name].get('time_last_fuse',0) for r in recent]
        checks[aid+'_advancing'] = len(stamps)>=2 and all(a<b for a,b in zip(stamps,stamps[1:]))
    forbidden=('cs_gnss_pos','cs_gnss_vel','cs_gnss_yaw','cs_gps_hgt','cs_ev_vel') if with_ev else ('cs_gnss_pos','cs_gnss_vel','cs_ev_pos','cs_ev_vel','cs_ev_hgt','cs_ev_yaw')
    checks['no_gnss_or_ev_velocity' if with_ev else 'no_gnss_or_ev'] = every(lambda r:all(r['estimator_status_flags'].get(k)==0 for k in forbidden))
    checks['fusion_flags'] = every(lambda r:fresh(r,'estimator_status_flags',2) and r['estimator_status_flags'].get('cs_opt_flow')==1 and r['estimator_status_flags'].get('cs_rng_hgt')==1)
    checks['local_velocity_valid'] = every(lambda r:r['vehicle_local_position'].get('v_xy_valid')==1 and r['vehicle_local_position'].get('z_valid')==1)
    return dict(passed=all(checks.values()), checks=checks, scope='disarmed stationary flow/range fusion only; no flight, motion accuracy or VIO fallback qualification')


def parameters_match(parameters, expected):
    for name, value in expected.items():
        match = re.search(r'\b'+re.escape(name)+r'\s*\[[^\]]+\]\s*:\s*([-+\d.eE]+)', parameters.get(name,''))
        if not match or not math.isclose(float(match.group(1)), value, rel_tol=1e-6, abs_tol=1e-6):
            return False
    return True


def loss_assessment(receipts):
    last=receipts[-1] if receipts else {}
    flags=last.get('estimator_status_flags',{})
    now=max((v.get('_observed_now_us',0) for v in last.values()),default=0)
    checks=dict(disarmed=last.get('vehicle_status',{}).get('arming_state')==1,
        optical_flow_stopped=flags.get('cs_opt_flow')==0,
        range_height_stopped=flags.get('cs_rng_hgt')==0)
    for topic in ('sensor_optical_flow','distance_sensor'):
        t=last.get(topic,{}).get('timestamp',0)
        checks[topic+'_stale']=now>0 and t>0 and now-t>2e6
    return dict(passed=all(checks.values()),checks=checks,
        scope='disarmed loss of both sources; no airborne recovery validation')
