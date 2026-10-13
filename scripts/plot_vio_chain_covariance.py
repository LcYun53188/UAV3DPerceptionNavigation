#!/usr/bin/env python3
"""Plot frozen consistency measurements; does not modify or calibrate covariance."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('evidence',type=Path)
    parser.add_argument('--observability',action='store_true')
    args=parser.parse_args()
    names=['openvins-dynamic-v1','openvins-declared-v1','openvins-rest-v1','openvins-rest-repeat']
    labels=['Legacy IMU model','Declared IMU model','Rest update, first run','Rest update, repeat']
    if args.observability:
        names=['openvins-rest-repeat','openvins-attitude-v1','openvins-attitude-gc-v1','openvins-attitude-gc-repeat']
        labels=['Translation / yaw only','Roll / pitch; observer GC fault','Excitation and observer GC fix','Same configuration, repeat']
    fig,axes=plt.subplots(2,2,figsize=(11,7),sharey=True,layout='constrained')
    for ax,name,label in zip(axes.flat,names,labels):
        c=json.loads((args.evidence/name/'covariance-consistency.json').read_text())
        # Motion is observed independently; use the recorded fixture trigger.
        profile=args.evidence/name/'plot-fixture.json'
        if profile.exists():start=json.loads(profile.read_text())['trigger_ros']
        else:
            run=args.evidence.parents[4]/'.cache/simulation/vio-sensors'/c['run_id']
            r=json.loads((run/'result.json').read_text());start=r['same_aircraft_dynamic']['trigger']['ros']
        ax.semilogy([v['stamp']-start for v in c['rows']],[v['nees'] for v in c['rows']],lw=.8)
        ax.axhline(c['chi2_6d_threshold95'],color='tab:red',ls='--',label='6D 95% threshold')
        ax.axhline(c['chi2_6d_threshold99'],color='tab:orange',ls=':',label='6D 99% threshold')
        ax.axvspan(12,88,color='tab:orange',alpha=.1,label='Motion window')
        ax.axvspan(102,max(v['stamp']-start for v in c['rows']),color='tab:blue',alpha=.07,label='Final rest')
        ax.set_ylim(.001,100);ax.set_title(f'{label}\n95% coverage {100*c["coverage95"]:.2f}%, 99% {100*c["coverage99"]:.2f}%')
        ax.set_xlabel('Seconds from fixture trigger');ax.set_ylabel('6D NEES');ax.grid(alpha=.2)
    axes.flat[0].legend(fontsize=8,loc='lower right')
    fig.suptitle('Same-aircraft VIO covariance: fixed launch gauge, full cross covariance\nTemporal samples are correlated; coverage is a rejection screen, not calibration proof')
    fig.savefig(args.evidence/('observability-comparison.png' if args.observability else 'covariance-comparison.png'),dpi=160)

if __name__=='__main__':main()
