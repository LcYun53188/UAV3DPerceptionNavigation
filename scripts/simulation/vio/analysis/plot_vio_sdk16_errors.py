#!/usr/bin/env python3
"""Plot measured SDK16 error and covariance, without altering estimates."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import argparse
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('inputs',type=Path,nargs='+');p.add_argument('--out',type=Path,required=True)
    a=p.parse_args();fig,axes=plt.subplots(3,len(a.inputs),figsize=(5*len(a.inputs),9),squeeze=False,layout='constrained')
    for col,path in enumerate(a.inputs):
        r=json.loads(path.read_text());rows=r['measurement_rows'];start=r['phases']['motion']['interval_s'][0]
        t=np.array([v['stamp']-start for v in rows]);e=np.array([v['position_error_m'] for v in rows])*100
        axes[0,col].plot(t,np.linalg.norm(e,axis=1),label='VIO position')
        ekf=r['ekf_measurement_rows'];axes[0,col].plot([v['stamp']-start for v in ekf],
            [np.linalg.norm(v['position_error_m'])*100 for v in ekf],lw=.7,alpha=.7,label='PX4 EKF position')
        axes[0,col].axhline(r['limit_m']*100,color='red',ls='--',label='10 cm maximum')
        axes[0,col].set_title(path.stem);axes[0,col].set_ylabel('Position error (cm)');axes[0,col].legend(fontsize=8)
        for i,label in enumerate(['x','y','z']):axes[1,col].plot(t,e[:,i],lw=.8,label=label)
        axes[1,col].set_ylabel('Signed axis error (cm)');axes[1,col].legend(fontsize=8)
        o=np.array([v['orientation_error_deg'] for v in rows]);axes[2,col].plot(t,np.linalg.norm(o,axis=1),label='Rotation error')
        axes[2,col].plot(t,[abs(v['yaw_error_deg']) for v in rows],lw=.8,label='Yaw error')
        axes[2,col].set_ylabel('Orientation error (deg)');axes[2,col].legend(fontsize=8)
        for ax in axes[:,col]:
            ax.axvspan(0,100,color='orange',alpha=.08);ax.axvspan(105,max(t),color='blue',alpha=.04)
            ax.grid(alpha=.25);ax.set_xlabel('Seconds from physical motion trigger')
    fig.suptitle('cuVSLAM 16: original timestamps, fixed launch position/yaw, no trajectory fit\nDisarmed physical fixture; simulation evidence, not motor flight')
    fig.savefig(a.out,dpi=150)


if __name__=='__main__':main()
