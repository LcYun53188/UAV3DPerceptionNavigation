"""Generate an offline comparison with system Python, numpy and matplotlib."""
import gzip,json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from assess_motion import rotation
root=Path(__file__).resolve().parent

def read(case,name):
    p=root/case/(name+'.json')
    return json.loads(p.read_text() if p.exists() else gzip.decompress(p.with_suffix('.json.gz').read_bytes()))
fig,axes=plt.subplots(2,2,figsize=(11,6.5),sharex='col')
for col,case in enumerate(('planar-90','layered-120')):
    truth=read(case,'truth');raw=read(case,'sdk-poses');norm=read(case,'normalized-poses')
    raw=[r for r in raw if r['stamp']>=norm[0]['stamp']]
    ts=np.array([r['stamp'] for r in truth]);pos=np.array([r['position'] for r in truth])
    raw=[r for r in raw if ts[0]<=r['stamp']<=ts[-1]]
    t=np.array([r['stamp'] for r in raw]);reference=np.column_stack([np.interp(t,ts,pos[:,i]) for i in range(3)])
    initial=raw[0];i=int(np.clip(np.searchsorted(ts,t[0],side='right')-1,0,len(ts)-2));f=(t[0]-ts[i])/(ts[i+1]-ts[i])
    q0,q1=np.array(truth[i]['quaternion']),np.array(truth[i+1]['quaternion'])
    if np.dot(q0,q1)<0:q1=-q1
    align=rotation((1-f)*q0+f*q1)@rotation(initial['quaternion']).T
    aligned=np.array([r['position'] for r in raw])@align.T
    aligned+=reference[0]-aligned[0]
    errors=np.linalg.norm(aligned-reference,axis=1)
    axes[0,col].plot(t,errors,label='Raw SDK error after fixed initial alignment')
    axes[0,col].axhline(.30,color='r',linestyle='--',label='Max error limit 0.30 m')
    variances=[]
    for r in raw:
        R=rotation(r['quaternion']);A=np.zeros((6,6));A[:3,:3]=A[3:,3:]=R
        variances.append(np.diag(A@np.array(r['covariance']).reshape(6,6)@A.T)[:3])
    for i,axis in enumerate('xyz'):axes[1,col].plot(t,np.asarray(variances)[:,i],label=f'{axis} variance')
    axes[1,col].axhline(.25,color='r',linestyle='--',label='Variance limit 0.25 m^2')
    for ax in axes[:,col]:
        ax.axvline(norm[-1]['stamp'],color='k',linestyle=':',label='Last normalized pose')
        ax.grid(alpha=.25);ax.legend(fontsize=7)
    axes[0,col].set_title(case);axes[0,col].set_ylabel('Position error (m)')
    axes[1,col].set_ylabel('World position variance (m^2)');axes[1,col].set_xlabel('Simulation time (s)')
fig.suptitle('Reference stereo/IMU motion; confidence and error assessed separately')
fig.tight_layout();fig.savefig(root/'motion-comparison.png',dpi=150)
