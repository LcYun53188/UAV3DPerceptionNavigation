#!/usr/bin/env python3
"""Screen actual PX4 native fusion in one completed flight's ULog."""

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
from pyulog import ULog


def analyze(folder):
    paths = list(folder.rglob('*.ulg'))
    if len(paths) != 1:
        raise ValueError('Expected exactly one owned ULog')
    path = paths[0]
    names = ['vehicle_status', 'estimator_status_flags',
             'estimator_aid_src_optical_flow', 'estimator_aid_src_rng_hgt']
    data = {d.name:d.data for d in ULog(str(path), message_name_filter_list=names).data_list}
    status = data['vehicle_status']
    times = status['timestamp']
    armed = times[status['arming_state'] == 2]
    start = int(armed.min())
    end = int(times[(times > armed.max()) & (status['arming_state'] == 1)].min())
    flags = data['estimator_status_flags']
    in_air = flags['cs_in_air'].astype(bool) & (flags['timestamp'] >= start) & (flags['timestamp'] <= end)
    fields = ['cs_gnss_pos', 'cs_gps_hgt', 'cs_gnss_yaw', 'cs_ev_pos',
              'cs_ev_hgt', 'cs_ev_yaw', 'cs_ev_vel', 'cs_opt_flow', 'cs_rng_hgt', 'cs_rng_stuck']
    report = dict(schema=1, run_id=folder.name,
        scope='native PX4 ULog armed-window observations; flow fusion gaps include native LAND; not covariance or hardware qualification',
        ulog_path=str(path.relative_to(folder)),
        ulog_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        armed_start_us=start, disarmed_us=end, armed_duration_s=(end-start)/1e6,
        flags={key:dict(in_air_records=int(in_air.sum()),
            true_in_air_records=int(np.count_nonzero(flags[key][in_air])),
            true_whole_log=int(np.count_nonzero(flags[key]))) for key in fields}, aids={},
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    for name in names[2:]:
        aid = data[name]
        window = (aid['timestamp'] >= start) & (aid['timestamp'] <= end)
        success = aid['timestamp'][window & aid['fused'].astype(bool)]
        ratios = np.concatenate([v[window] for k,v in aid.items()
                                 if k == 'test_ratio' or k.startswith('test_ratio[')])
        report['aids'][name] = dict(samples=int(window.sum()),
            fused_samples=int(np.count_nonzero(aid['fused'][window])),
            innovation_rejected_samples=int(np.count_nonzero(aid['innovation_rejected'][window])),
            test_ratio_max=float(ratios.max()),
            successful_fusion_max_gap_s=float(np.diff(success).max()/1e6) if len(success)>1 else None)
    (folder / 'native-joint-flight-screening.json').write_text(json.dumps(report, indent=2)+'\n')
    snapshots = folder / 'analysis-inputs'
    snapshots.mkdir(exist_ok=True)
    (snapshots / Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    analyze(parser.parse_args().run)
