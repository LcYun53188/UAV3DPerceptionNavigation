#!/usr/bin/env python3
"""Reproduce the archived EKF reference/measurement plot; truth is checked separately."""
import gzip
import json
from pathlib import Path
import sys
import numpy as np
from scipy.interpolate import BSpline
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

folder=Path(sys.argv[1])
def load(name):
    p=folder/name
    return json.loads(p.read_bytes() if p.exists() else gzip.decompress(p.with_suffix(p.suffix+'.gz').read_bytes()))
trace=load('flight-trace.json');plans=load('accepted-plans.json')
fig,axes=plt.subplots(1,3,figsize=(14,4),constrained_layout=True)
points=np.array([t['position_enu'] for t in trace if t['phase']=='OBSERVE'])
axes[0].plot(points[:,0],points[:,1],lw=.6,label='observed EKF position')
axes[0].set(title='Four-view survey (fixed camera pitch -5 deg)',xlabel='ENU x (m)',ylabel='ENU y (m)',aspect='equal')
for phase,color in [('NAVIGATE','tab:blue'),('RETURN','tab:orange')]:
    samples=[t for t in trace if t['phase']==phase]
    p=np.array([t['position_enu'] for t in samples]);r=np.array([t['reference'] for t in samples])
    axes[1].plot(p[:,0],p[:,1],color=color,label=phase+' measured')
    axes[2].plot([t['ros'] for t in samples],np.linalg.norm(p-r,axis=1),color=color,label=phase)
for plan in plans:
    trajectory=plan['bound']['trajectory'];c=np.array([[p[k] for k in ('x','y','z')]for p in trajectory['control_points']]);dt=trajectory['knot_interval']
    curve=BSpline((np.arange(len(c)+4)-3)*dt,c,3,extrapolate=False)
    p=curve(np.linspace(0,curve.t[-4],300));axes[1].plot(p[:,0],p[:,1],'k--',lw=.7)
axes[1].set(title='Admitted EGO curves and actual EKF motion',xlabel='ENU x (m)',ylabel='ENU y (m)',aspect='equal',ylim=(-.18,.18))
axes[2].axhline(.3,c='red',ls='--',label='tracking limit')
axes[2].set(title='Active-curve tracking error',xlabel='simulation time (s)',ylabel='error (m)',ylim=(0,.33))
for ax in axes:ax.grid(alpha=.25);ax.legend(fontsize=7)
fig.savefig(folder/'flight-path.png',dpi=160)
