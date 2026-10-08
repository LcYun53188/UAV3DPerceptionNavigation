"""Replay quality-limit retirement from archived traces, independently of passed flags."""
import gzip
import json
from pathlib import Path
import numpy as np

root = Path(__file__).resolve().parent/'quality-limit-90'
def read(name):
    return json.loads(gzip.decompress((root/(name+'.gz')).read_bytes()))

statuses = read('normalized-status.json')
poses = read('normalized-poses.json')
raw = read('sdk-poses.json')
valid = [s for s in statuses if s['valid']]
fault = next(s for s in statuses if s['mono']>valid[0]['mono'] and not s['valid'])
last = max((p for p in poses if p['mono']<fault['mono']),key=lambda p:p['stamp'])
rejected = next(r for r in raw if r['stamp']>last['stamp'])
x,y,z,w = np.asarray(rejected['quaternion'])/np.linalg.norm(rejected['quaternion'])
r = np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
              [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
              [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
a = np.zeros((6,6)); a[:3,:3]=a[3:,3:]=r
world = a@np.asarray(rejected['covariance']).reshape(6,6)@a.T
late = [s for s in statuses if s['mono']>=fault['mono']+.5]
checks = dict(covariance_over_limit=bool(np.max(np.diag(world))>.25),
    reason=fault['reason']=='VIO_UNCERTAINTY_INVALID',
    latched=bool(late) and all(not s['valid'] and s['reason']==fault['reason'] for s in late),
    no_late_output=not any(p['mono']>fault['mono']+.5 for p in poses),
    source_continues=raw[-1]['stamp']>fault['stamp']+10,
    no_identity_change=len({(s['session'],s['reset']) for s in statuses})==1)
print(json.dumps(dict(scope='uncertainty retirement replay only; sustained VIO readiness FAILED',
    safety_reaction_passed=all(checks.values()),checks=checks,
    last_normalized_stamp=last['stamp'],rejected_sdk_stamp=rejected['stamp'],
    rejected_world_variances=np.diag(world).tolist(),first_invalid=fault,
    healthy_window_s=valid[-1]['mono']-valid[0]['mono'],raw_sdk_last_stamp=raw[-1]['stamp']),indent=2))
raise SystemExit(0 if all(checks.values()) else 1)
