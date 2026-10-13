"""Explicit process-local scheduling comparison; no system-wide CPU changes."""
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
