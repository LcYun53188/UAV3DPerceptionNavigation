#!/usr/bin/env python3
"""Read-only S0 dependency audit. Runtime topic/TF/clock readiness is not asserted."""
import argparse
import platform
import os
import signal
from pathlib import Path
import subprocess

from sim_validation import ROOT, file_hash, read, write_json


def command(args, timeout=20):
    try:
        process = subprocess.Popen(args, cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                   text=True, start_new_session=True,
                                   env=dict(os.environ, GIT_LFS_SKIP_SMUDGE='1', GIT_TERMINAL_PROMPT='0'))
        try:
            stdout, stderr = process.communicate(timeout=timeout)
            return process.returncode, stdout.strip(), stderr.strip()
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5)
            return 124, '', f'command timed out after {timeout}s'
    except OSError as exc:
        return 127, '', str(exc)



def audit(lock):
    checks = []

    def check(name, expected, actual, error=''):
        checks.append(dict(name=name, expected=expected, actual=actual,
                           status='PASS' if expected == actual and not error else 'FAIL', error=error))

    check('architecture', lock['host']['architecture'], platform.machine())
    os_info = read_os_release()
    check('os_id', lock['host']['os_id'], os_info.get('ID'))
    check('os_version', lock['host']['os_version'], os_info.get('VERSION_ID'))
    check('ros_install', True, (Path('/opt/ros') / lock['host']['ros_distro']).is_dir())
    for name, expected in lock['packages'].items():
        code, actual, error = command(['dpkg-query', '-W', '-f=${Version}', name])
        check(f'package:{name}', expected, actual, error if code else '')
    for source in lock['sources']:
        code, actual, error = command(['git', '-C', str(ROOT / source['path']), 'rev-parse', 'HEAD'])
        check(f"source:{source['path']}", source['commit'], actual, error if code else '')
    for entry in lock['files']:
        path = ROOT / entry['path']
        check(f"file:{entry['path']}", entry['sha256'], file_hash(path) if path.is_file() else None)
    code, vendor_output, vendor_error = command(['python3', 'scripts/vendor_patches.py', '--check'], timeout=180)
    checks.append(dict(name='managed_vendor_patches', status='PASS' if code == 0 else 'FAIL',
                       output=vendor_output, error=vendor_error))
    return dict(schema=1, scope='offline_dependency_audit',
                status='FAIL' if any(c['status'] == 'FAIL' for c in checks) else 'PASS',
                checks=checks, pending=lock['pending'],
                runtime_readiness='INCONCLUSIVE: topics, TF, clock and live navigation not sampled')


def read_os_release():
    from pathlib import Path
    return {key: value.strip('"') for line in Path('/etc/os-release').read_text().splitlines()
            if '=' in line for key, value in [line.split('=', 1)]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lock', default=str(ROOT / 'simulation/px4/versions.lock.yaml'))
    parser.add_argument('--output')
    args = parser.parse_args()
    result = audit(read(args.lock))
    if args.output:
        write_json(args.output, result)
    for check in result['checks']:
        print(f"{check['status']} {check['name']}")
    print(f"{result['status']} offline dependencies; runtime readiness: INCONCLUSIVE")
    for name, reason in result['pending'].items():
        print(f'PENDING {name}: {reason}')
    return 0 if result['status'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
