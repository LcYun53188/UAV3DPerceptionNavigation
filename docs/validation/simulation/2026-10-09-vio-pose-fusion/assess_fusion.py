"""Independent aggregate-evidence checks; does not claim camera or flight validation."""
import json,hashlib,struct
from extract_local import decoder
from pathlib import Path
root=Path(__file__).resolve().parent
results={}
for case,aids in [('default-four-aids',('ev_pos','ev_hgt','ev_vel','ev_yaw')),
                  ('pose-three-aids',('ev_pos','ev_hgt','ev_yaw'))]:
    folder=root/case;r=json.loads((folder/'vision-fusion.json').read_text())
    checks=dict(ready=r['ready_before_stop'] and r['ready_count']>=5,
        fused=all(r['fused_samples'].get(n,0)>=5 and r['last_messages'][n]['fused'] for n in aids),
        stopped=all(r['last_fuse_after_drain'][n]==r['last_fuse_at_end'][n] for n in aids),
        controls_absent=not any(r['control_publishers'].values()),
        current_sources=json.loads((folder/'current-source-check.json').read_text())['all_current'],
        parameters=json.loads((folder/'effective-parameters.json').read_text())['passed'])
    hidden=('cs_gnss_pos','cs_gnss_vel','cs_gnss_yaw','cs_gps_hgt','cs_mag','cs_mag_hdg','cs_mag_3d','cs_opt_flow')
    checks['other_aiding']=not any(r['other_aiding_before_stop'][n] for n in hidden)
    if case=='pose-three-aids':
        m=r['last_observed_input'];stop=r['first_stop_rejection']
        raw=(folder/'local-before-stop-ulog.bin').read_bytes()
        descriptor=json.loads((folder/'local-before-stop-ulog.format.json').read_text())
        checks['local_record_hash']=hashlib.sha256(raw).hexdigest()==descriptor['raw_sha256']
        local=decoder(descriptor['format'])(raw[5:])
        checks['local_record_fresh']=0<=r['local_before_stop']['timestamp']-local['timestamp']<=50000
        checks['unknown_velocity']=(m['velocity_frame']==0 and all(v is None for n in
            ('velocity','velocity_variance','angular_velocity') for v in m[n])
            and r['observed_input_count']==r['unknown_velocity_samples']==r['input_count'])
        checks['ev_velocity_disabled']=not r['other_aiding_before_stop']['cs_ev_vel'] and r['fused_samples'].get('ev_vel',0)==0
        checks['range_aux_disabled']=not any(r['other_aiding_before_stop'][n] for n in ('cs_rng_hgt','cs_aux_gpos'))
        checks['ekf_velocity']=all(local[n] for n in ('xy_valid','z_valid','v_xy_valid','v_z_valid','heading_good_for_control'))
        checks['ekf_velocity'] &= not local['dead_reckoning'] and all(0<local[n]<=.5 for n in ('eph','epv','evh','evv'))
        checks['stop_rejection']=stop['after_stop_s']<=.5 and stop['reason'] in ('VIO_SOURCE_INVALID','VIO_TELEMETRY_STALE:source')
        checks['terminal_rejected']=r['final_gate_reason'] in ('VIO_EKF_LOCAL_RESET','VIO_TELEMETRY_STALE:source')
        source=r['last_messages']['source'];stamp=source['sample_stamp']
        checks['preserved_sample']=m['timestamp_sample']==stamp['sec']*1000000+round(stamp['nanosec']/1000)
        checks['alignment_identity']=r['alignment']['alignment_id'] in source['localization_session']
    results[case]=dict(passed=all(checks.values()),checks=checks)
print(json.dumps(results,indent=2))
raise SystemExit(0 if all(r['passed'] for r in results.values()) else 1)
