"""Passive Python GC pause evidence; never changes thresholds or GC behavior."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import gc
import time


class GcPauseAudit:
    def __init__(self):
        self.starts={};self.rows=[]
        gc.callbacks.append(self.callback)

    def callback(self,phase,info):
        generation=info['generation'];now=time.monotonic()
        if phase=='start':self.starts[generation]=now
        elif generation in self.starts:
            start=self.starts.pop(generation)
            self.rows.append(dict(mono_start=start,mono_end=now,duration_s=now-start,
                generation=generation,collected=info['collected'],uncollectable=info['uncollectable']))

    def stop(self):
        gc.callbacks.remove(self.callback)
        return dict(schema=1,scope='passive GC callbacks only',rows=self.rows,
                    max_pause_s=max((r['duration_s'] for r in self.rows),default=0.))
