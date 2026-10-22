"""Explicit process-local scheduling comparison; no system-wide CPU changes."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import os
import shutil


def cpu_set(value):
    result=set()
    try:
        for part in value.split(','):
            ends=part.split('-')
            if len(ends)>2:raise ValueError()
            first=int(ends[0]);last=int(ends[-1])
            if first<0 or last<first or last-first>4096:raise ValueError()
            result.update(range(first,last+1))
    except ValueError:raise ValueError('Invalid SDK CPU list') from None
    if not result or not result<=os.sched_getaffinity(0):
        raise ValueError('SDK CPU list must be within this process allowed CPUs')
    return result


def command(binary_command,value):
    cpu_set(value)
    executable=shutil.which('taskset')
    if executable is None:raise RuntimeError('taskset is not installed')
    return [executable,'--cpu-list',value]+list(binary_command)


def receipt(pid,value):
    expected=cpu_set(value)
    actual=os.sched_getaffinity(pid)
    return dict(passed=actual==expected,pid=pid,expected=sorted(expected),actual=sorted(actual),
        scope='SDK launch process affinity after SDK parameter service is ready; children inherit the mask')
