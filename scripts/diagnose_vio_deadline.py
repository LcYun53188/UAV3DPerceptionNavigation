#!/usr/bin/env python3
"""Locate original-sample deadline handover; read-only, never reauthorize a source."""
import argparse
import json
from pathlib import Path
from assess_vio_timing import read,assess


def diagnose(folder):
    fusion=read(folder,'fusion-timing.json')['records']
    normalizer=read(folder,'normalizer-timing.json')['records']
    sensor=read(folder,'sensor-timing.json')['records']
    faults=[r for r in fusion if r['stage'] in ('accept_fault','watchdog_fault')]
    fault=min(faults,key=lambda r:r['mono']) if faults else None
    result=dict(schema=1,flight_authorized=False,first_fusion_fault=fault,
        first_normalizer_retirement=next((r for r in normalizer if r['stage']=='retire'),None),
        scope='Recorded callback chronology only; no resumed source, predicted pose or acceptance change')
    if fault is None:return result
    accepted=[r for r in fusion if r['stage']=='accepted' and r['mono']<=fault['mono']]
    last=max(accepted,key=lambda r:r['mono']) if accepted else None
    result['last_accepted']=last
    if last is None:return result
    future=[r for r in normalizer if r['stage']=='pose_emit' and r['sample']>last['sample']]
    next_emit=min(future,key=lambda r:r['mono']) if future else None
    result['next_normalizer_emit']=next_emit
    if next_emit is not None:
        result['next_emit_after_fault_s']=next_emit['mono']-fault['mono']
        arrivals=[r for r in fusion if r['stage']=='pose_rx' and r['sample']==next_emit['sample']]
        result['next_fusion_receive']=min(arrivals,key=lambda r:r['mono']) if arrivals else None
        tracking=[r for r in sensor if r['stage']=='tracking' and r['sample']==next_emit['sample']]
        result['next_sdk_tracking']=tracking[0] if tracking else None
    # Include both preceding frame processing and the first subsequent SDK output.
    stop=max(fault['mono']+.35,next_emit['mono']+.05 if next_emit is not None else fault['mono']+.35)
    result['fault_window_timing']=assess(folder,after_mono=fault['mono']-1.,before_mono=stop)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('folder',type=Path)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    args.output.write_text(json.dumps(diagnose(args.folder),indent=2)+'\n')
