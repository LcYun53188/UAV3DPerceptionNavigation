#!/usr/bin/env python3
"""Compare original-sample timing across owned publishers; no control/admission effects."""
import argparse,gzip,json
from pathlib import Path
import numpy as np


def summary(values):
    values=np.asarray(values,dtype=float)
    if not len(values):return dict(count=0)
    return dict(count=len(values),min_s=float(values.min()),p50_s=float(np.quantile(values,.5)),
        p95_s=float(np.quantile(values,.95)),p99_s=float(np.quantile(values,.99)),max_s=float(values.max()))


def read(folder,name):
    path=folder/name
    if path.is_file():return json.loads(path.read_text())
    with gzip.open(str(path)+'.gz','rt') as stream:return json.load(stream)


def assess(folder, *, after_mono=None, before_mono=None):
    sensor=read(folder,'sensor-timing.json')
    normalizer=read(folder,'normalizer-timing.json')
    fusion_path=folder/'fusion-timing.json'
    fusion=read(folder,'fusion-timing.json') if fusion_path.is_file() or Path(str(fusion_path)+'.gz').is_file() else None
    streams=[(sensor,'observer'),(normalizer,'normalizer')]+([(fusion,'fusion')] if fusion is not None else [])
    if after_mono is not None or before_mono is not None:
        if after_mono is not None and before_mono is not None and after_mono>before_mono:
            raise ValueError('Invalid monotonic observation window')
        for data,_ in streams:
            data['records']=[r for r in data['records']
                if (after_mono is None or r['mono']>=after_mono) and
                   (before_mono is None or r['mono']<=before_mono)]
    rows=[r for data,_ in streams for r in data['records']]
    # All owned processes use the same host system clock. Reject interpretation
    # if that clock jumps relative to monotonic reception time.
    offsets=[r['system_ns']/1e9-r['mono'] for r in rows]
    stable=bool(offsets) and max(offsets)-min(offsets)<.02
    metrics={}
    for data,name in streams:
        for stage in sorted({r['stage'] for r in data['records']}):
            records=[r for r in data['records'] if r['stage']==stage]
            source_delays=[(r['system_ns']-r['rmw']['source_timestamp'])/1e9 for r in records
                if r['rmw'].get('source_timestamp',0)>1e18]
            metrics[name+':'+stage]=dict(age=summary([r['age_s'] for r in records if r['age_s'] is not None]),
                callback_elapsed=summary([r['elapsed_s'] for r in records if 'elapsed_s' in r]),
                publisher_to_callback=summary(source_delays))
    samples={}
    for r in sensor['records']:
        if r['sample'] is not None:samples.setdefault(round(r['sample']*1e9),{})[r['stage']]=r
    pipeline=[]
    for t,stages in samples.items():
        if not all(n in stages for n in ('left','right','pose_cov')):continue
        times=[stages[n]['rmw'].get('source_timestamp',0) for n in ('left','right','pose_cov')]
        if not all(v>1e18 for v in times):continue
        pipeline.append(dict(sample=t/1e9,delay_s=(times[2]-max(times[:2]))/1e9))
    faults=[r for r in normalizer['records'] if r['stage']=='retire']
    fault=faults[0] if faults else None
    fusion_faults=[r for r in fusion['records'] if r['stage'] in ('accept_fault','watchdog_fault')] if fusion is not None else []
    expired_previous=[r for r in fusion['records'] if r['stage']=='accepted' and r.get('previous') is not None
        and r['ros']-r['previous']>.2 and -.05<=r['age_s']<=.2] if fusion is not None else []
    around={}
    if fault is not None:
        for label,data in (('observer',sensor),('normalizer',normalizer)):
            around[label]=[r for r in data['records'] if fault['mono']-.5<=r['mono']<=fault['mono']+.3
                and r['stage']!='imu']
    result=dict(schema=1,host_clock_stable=stable,host_clock_offset_range_s=max(offsets)-min(offsets) if offsets else None,
        trace_complete=all(d['total']==d['retained'] for d,_ in streams),
        fusion_faults=fusion_faults[:5],fresh_pairs_with_expired_previous=len(expired_previous),
        fresh_pair_examples=expired_previous[:5],
        scope='shared host DDS publish-to-callback and stereo-to-SDK publish timing; not physical sensor synchronization',
        metrics=metrics,stereo_to_sdk_publish=summary([p['delay_s'] for p in pipeline]) if stable else None,
        worst_pipeline=sorted(pipeline,key=lambda p:p['delay_s'],reverse=True)[:20] if stable else [],
        first_source_retirement=fault,around_retirement=around)
    sdk=[r for r in sensor['records'] if r['stage']=='tracking' and 'sdk_track_s' in r]
    if sdk:
        valid=[r for r in sdk if all(np.isfinite(r.get(k,float('nan'))) and r[k]>=0
            for k in ('sdk_track_s','sdk_callback_s')) and r['sdk_callback_s']>=r['sdk_track_s']]
        matched=[]
        timing_by_sample={round(r['sample']*1e9):r for r in valid if r['sample'] is not None}
        for p in pipeline:
            r=timing_by_sample.get(round(p['sample']*1e9))
            if r is not None:
                matched.append(dict(sample=p['sample'],pipeline_s=p['delay_s'],
                    track_s=r['sdk_track_s'],callback_s=r['sdk_callback_s'],
                    outside_callback_lower_bound_s=max(0.,p['delay_s']-r['sdk_callback_s'])))
        result['sdk_execution']=dict(valid_count=len(valid),invalid_count=len(sdk)-len(valid),
            track=summary([r['sdk_track_s'] for r in valid]),
            callback=summary([r['sdk_callback_s'] for r in valid]),
            callback_without_track=summary([r['sdk_callback_s']-r['sdk_track_s'] for r in valid]),
            outside_callback_lower_bound=summary([r['outside_callback_lower_bound_s'] for r in matched]) if stable else None,
            worst_pipeline=sorted(matched,key=lambda r:r['pipeline_s'],reverse=True)[:20] if stable else [],
            scope='SDK Track and UpdatePose wall duration; pipeline minus full callback is only a lower bound outside UpdatePose, not exact queue or synchronization time')
    if after_mono is not None or before_mono is not None:
        result['observation_window']=dict(after_mono=after_mono,before_mono=before_mono,
            scope='Explicit monotonic window only; full-run clock stability and acceptance unchanged')
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('folder',type=Path);parser.add_argument('--output',type=Path)
    parser.add_argument('--after-mono',type=float);parser.add_argument('--before-mono',type=float)
    args=parser.parse_args();result=assess(args.folder,after_mono=args.after_mono,before_mono=args.before_mono);text=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(text)
    else:print(text,end='')
