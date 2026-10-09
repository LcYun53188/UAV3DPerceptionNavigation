"""Summarize source confidence separately from realized position errors."""
import gzip,json
from pathlib import Path
import numpy as np
from assess_motion import rotation
root=Path(__file__).resolve().parent
output={}
for case in ('planar-90','layered-90','layered-120'):
    folder=root/case
    if not folder.exists():continue
    def read(name):
        p=folder/(name+'.json')
        return json.loads(p.read_text() if p.exists() else gzip.decompress(p.with_suffix('.json.gz').read_bytes()))
    raw,norm,status,result=[read(n) for n in ('sdk-poses','normalized-poses','normalized-status','result')]
    raw=[r for r in raw if r['stamp']>=norm[0]['stamp']]
    variances=[]
    for r in raw:
        R=rotation(r['quaternion']);A=np.zeros((6,6));A[:3,:3]=A[3:,3:]=R
        variances.append(np.diag(A@np.array(r['covariance']).reshape(6,6)@A.T))
    v=np.array(variances)
    valid=[s for s in status if s['valid']]
    later=[s for s in status if s['mono']>=valid[0]['mono']]
    output[case]=dict(overall_passed=result['passed'],raw_window_start=raw[0]['stamp'],
        raw_window_end=raw[-1]['stamp'],normalized_window_start=norm[0]['stamp'],
        normalized_window_end=norm[-1]['stamp'],normalized_samples=len(norm),
        valid_status_span_wall_s=valid[-1]['mono']-valid[0]['mono'],
        status_continuous_after_binding=all(s['valid'] for s in later),
        raw_max_world_variances=v.max(axis=0).tolist(),
        raw_covariance_limit_exceeded_samples=int(np.count_nonzero(np.any(v>.25,axis=1))),
        raw_samples=len(raw),cleanup_confirmed=result['cleanup_confirmed'])
(root/'quality-summary.json').write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps(output,indent=2))
