"""Freeze completed mission evidence in a process independent of ROS callbacks."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import json
import multiprocessing
import threading
import shutil


def start_mission_snapshot(out,result,outputs,encode_message):
    """The worker has no ROS context, publisher, or control ownership."""
    (out/'mission-evidence').mkdir(exist_ok=True)
    prefix=tuple(outputs)
    def spawn_and_join():
        worker=multiprocessing.get_context('spawn').Process(target=_write_snapshot,
            args=(out,result,prefix,encode_message),name='completed_mission_evidence',daemon=True)
        worker.start()
        worker.join()
    # Child startup/imports can block a pipe write. Keep that wait outside the
    # observer; JSON encoding itself runs with the child's independent GIL.
    starter=threading.Thread(target=spawn_and_join,name='mission_snapshot_launcher',daemon=True)
    starter.start()
    return starter


def _write_snapshot(out,result,prefix,encode_message):
    target=out/'mission-evidence'
    try:
        for name in ('flight-truth.json','flight-trace.json','flight-observation.json'):
            shutil.copy2(out/name,target/name)
        with (target/'result.json').open('w') as f:
            json.dump(dict(result,cleanup_confirmed=False,live_snapshot=True,qualification=False),f,indent=2)
        with (target/'fusion-outputs.pending.json').open('w') as f:
            f.write('[\n')
            for index,row in enumerate(prefix):
                if index:f.write(',\n')
                json.dump(dict(row,aligned=encode_message(row['aligned']),ev=encode_message(row['ev'])),f,indent=2)
            f.write('\n]\n')
        (target/'fusion-outputs.pending.json').replace(target/'fusion-outputs.json')
        receipt=dict(status='COMPLETED',samples=len(prefix),live_snapshot=True,
            scope='completed mission prefix; independent process, no ROS context or publishing')
    except Exception as exc:receipt=dict(status='ERROR',error=str(exc),live_snapshot=True)
    (target/'snapshot-receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
