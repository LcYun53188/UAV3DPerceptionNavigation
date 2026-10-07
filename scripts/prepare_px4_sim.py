#!/usr/bin/env python3
"""Prepare locked SITL dependencies without changing existing checkouts."""
import argparse
import hashlib
from pathlib import Path
import subprocess

from sim_validation import ROOT, read

SUBMODULES = ['src/modules/mavlink/mavlink', 'src/lib/events/libevents',
              'src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client',
              'Tools/simulation/gz', 'src/drivers/gps/devices']


def checked(args, **kwargs):
    subprocess.run(args, cwd=ROOT, check=True, **kwargs)


def ensure_source(entry):
    path = ROOT / entry['path']
    if not path.exists():
        checked(['git', 'clone', '--depth', '1', '--branch', entry['version'], entry['repository'], str(path)])
    head = subprocess.check_output(['git', '-C', str(path), 'rev-parse', 'HEAD'], text=True).strip()
    top = subprocess.check_output(['git', '-C', str(path), 'rev-parse', '--show-toplevel'], text=True).strip()
    if head != entry['commit'] or Path(top).resolve() != path.resolve():
        raise RuntimeError(f'{path}: checkout differs from lock; preserve work before correcting it')
    dirty = subprocess.check_output(['git', '-C', str(path), 'status', '--porcelain', '--untracked-files=no'], text=True)
    if dirty:
        raise RuntimeError(f'{path}: tracked modifications; source preparation refuses to overwrite them')
    return path


def check_external(lock):
    for entry in lock['agent'].get('external_sources', []):
        repo = ROOT / entry['path']
        commit = subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip()
        diff = subprocess.check_output(['git', '-C', str(repo), 'diff', '--binary', 'HEAD'])
        if commit != entry['commit'] or hashlib.sha256(diff).hexdigest() != entry['tracked_diff_sha256']:
            raise RuntimeError(f'{repo}: Agent dependency drift; update/revalidate the lock before using the build')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Check pinned roots without downloading submodules')
    parser.add_argument('--check-external', action='store_true', help='Verify resolved Agent build dependency commits/diffs')
    args = parser.parse_args()
    lock = read(ROOT / 'simulation/px4/versions.lock.yaml')
    for name in ['sitl', 'agent', 'px4_msgs']:
        if args.check and not (ROOT / lock[name]['path']).is_dir():
            raise RuntimeError(f'{name}: missing checkout')
        ensure_source(lock[name])
    if not args.check:
        checked(['git', '-C', str(ROOT / lock['sitl']['path']), 'submodule', 'update',
                 '--init', '--recursive', '--depth', '1', '--jobs', '4', '--', *SUBMODULES])
    if args.check_external:
        check_external(lock)
    print('Pinned SITL, Agent and px4_msgs source roots verified.')


if __name__ == '__main__':
    raise SystemExit(main())
