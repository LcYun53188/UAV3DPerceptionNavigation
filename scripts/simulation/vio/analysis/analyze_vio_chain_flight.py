#!/usr/bin/env python3
"""Offline receipt-time trajectory screening; never a control/estimator input."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import argparse
import gzip
import hashlib
import json
from pathlib import Path
import numpy as np


def read(folder,name):
    path=folder/name
    raw=path.read_bytes() if path.exists() else gzip.decompress(path.with_suffix(path.suffix+'.gz').read_bytes())
    return json.loads(raw),hashlib.sha256(raw).hexdigest()


def statistics(errors):
    if not len(errors):return None
    values=np.linalg.norm(np.asarray(errors),axis=1)
    return dict(samples=len(values),rmse_m=float(np.sqrt(np.mean(values**2))),
                max_m=float(values.max()),p95_m=float(np.percentile(values,95))) if len(values) else None


def analyze(folder):
    loaded={name:read(folder,name) for name in ('result.json','flight-truth.json','flight-trace.json','fusion-outputs.json')}
    result,truth,trace,outputs=[loaded[name][0] for name in loaded]
    trace=[r for r in trace if r['armed']==2]
    if not trace:raise ValueError('No actual armed trajectory')
    times=np.array([r['mono'] for r in truth]);positions=np.array([r['position'] for r in truth])
    anchor=positions[0]  # stationary launch origin; no trajectory fit
    def reference(time):
        return np.array([np.interp(time,times,positions[:,i]) for i in range(3)])-anchor
    measured=np.array([reference(r['mono']) for r in trace]);estimated=np.array([r['position_enu'] for r in trace])
    desired=np.array([r['reference'] for r in trace]);phases=sorted({r['phase'] for r in trace})
    controlled=np.array([r['owner']=='TASK' and r['phase'] not in ('LAND_REQUEST','LANDING','FAULT') for r in trace])
    sdk_errors=[]
    for r in outputs:
        if trace[0]['mono']<=r['mono']<=trace[-1]['mono']:
            p=r['aligned']['pose']['pose']['position']
            sdk_errors.append(np.array([p[k] for k in ('x','y','z')])-reference(r['mono']))
    analysis=dict(schema=1,scope='offline receipt-time screening; launch-origin alignment only; no trajectory fit; includes transport/processing latency, not exact sensor-time accuracy qualification',
        run_id=folder.name,mission_passed=result.get('flight',{}).get('passed',False),
        sdk16_aligned_position=statistics(sdk_errors),px4_position=statistics(estimated-measured),
        tracking=statistics((desired-measured)[controlled]),per_phase={},
        input_sha256={name:entry[1] for name,entry in loaded.items()},
        script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    for phase in phases:
        mask=np.array([r['phase']==phase for r in trace])
        analysis['per_phase'][phase]=dict(px4_position=statistics((estimated-measured)[mask]),
                                         tracking=statistics((desired-measured)[mask & controlled]))
    (folder/'flight-error-screening.json').write_text(json.dumps(analysis,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    desired[~controlled]=np.nan  # Autopilot LAND has no gateway altitude reference.
    fig,axes=plt.subplots(1,2,figsize=(10,4))
    axes[0].plot(desired[:,0],desired[:,1],'--',label='Control reference')
    axes[0].plot(measured[:,0],measured[:,1],label='Gazebo truth')
    axes[0].plot(estimated[:,0],estimated[:,1],alpha=.6,label='PX4 estimate')
    axes[0].set(xlabel='East / m',ylabel='North / m',title='Launch-aligned trajectory');axes[0].axis('equal')
    seconds=np.array([r['ros'] for r in trace]);seconds-=seconds[0]
    axes[1].plot(seconds,desired[:,2],'--',label='Control reference')
    axes[1].plot(seconds,measured[:,2],label='Gazebo truth')
    axes[1].plot(seconds,estimated[:,2],alpha=.6,label='PX4 estimate')
    axes[1].set(xlabel='Simulation time from arming / s',ylabel='Height / m',title='Flight altitude')
    for axis in axes:axis.grid(alpha=.25);axis.legend(fontsize=8)
    fig.tight_layout();fig.savefig(folder/'flight-trajectory.png',dpi=160);plt.close(fig)
    copy=folder/'analysis-inputs';copy.mkdir(exist_ok=True);(copy/Path(__file__).name).write_bytes(Path(__file__).read_bytes())
    print(json.dumps({key:analysis[key] for key in ('run_id','mission_passed','sdk16_aligned_position','px4_position','tracking')},indent=2))

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
    analyze(p.parse_args().run)
