"""Read Nsight's exported SQLite; overlap is evidence, not causal attribution."""

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
import sqlite3
import statistics


def assess(path):
    with sqlite3.connect('file:'+str(path)+'?mode=ro',uri=True) as db:
        tracks=db.execute("""SELECT n.start,n.end,n.globalTid FROM NVTX_EVENTS n
            LEFT JOIN StringIds s ON s.id=n.textId
            WHERE COALESCE(n.text,s.value)='cuvslam::Odometry::Track' AND n.end IS NOT NULL
            ORDER BY n.end-n.start""").fetchall()
        if not tracks:raise ValueError('No cuVSLAM Track ranges')
        start,end,tid=tracks[-1]
        def overlap(table):
            rows=db.execute('SELECT start,end FROM '+table+' WHERE start<? AND end>?',(end,start)).fetchall()
            spans=[min(b,end)-max(a,start) for a,b in rows]
            return dict(count=len(rows),summed_overlap_ms=sum(spans)/1e6,
                maximum_overlap_ms=max(spans,default=0)/1e6)
        durations=[(b-a)/1e6 for a,b,_ in tracks]
        return dict(schema=1,scope='Profiled run only; summed overlaps across all threads/streams are not utilization or proof of a CPU root cause',
            flight_qualified=False,track_count=len(tracks),track_thread_count=len({t for _,_,t in tracks}),
            track_ms=dict(median=statistics.median(durations),p99=durations[int(.99*(len(durations)-1))],maximum=durations[-1]),
            worst_track=dict(start_ns=start,end_ns=end,global_tid=tid,
                cuda_api=overlap('CUPTI_ACTIVITY_KIND_RUNTIME'),
                gpu_kernels=overlap('CUPTI_ACTIVITY_KIND_KERNEL')),
            maximum_cuda_api_ms=db.execute('SELECT MAX(end-start)/1e6 FROM CUPTI_ACTIVITY_KIND_RUNTIME').fetchone()[0])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('sqlite');args=parser.parse_args()
    print(json.dumps(assess(args.sqlite),indent=2))
