#!/usr/bin/env python3
"""S0 host regression runner with immutable configuration and bounded subprocesses.

Only S0_regression is implemented. Does not start/stop or command a simulation.
"""
import argparse
from datetime import datetime, timezone
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time
import uuid

from check_sim_environment import audit, command
from sim_validation import ROOT, digest, file_hash, load_acceptance, overall, read, validate, write_json


COMMANDS = {
    'S0_NAV_REGRESSION': ['python', '-m', 'pytest', '-q',
                          'scripts/test_sim_control.py', 'scripts/test_sim_validation.py', 'src/uav_nav_sim/test'],
    'S0_PX4_BRIDGE_REGRESSION': ['python', '-m', 'pytest', '-q', 'src/px4_comm_bridge/test'],
}


def run_command(args, log, timeout, env):
    """Own a new process group; bounded timeout cleans up only that group."""
    with Path(log).open('w') as stream:
        process = subprocess.Popen(args, cwd=ROOT, env=env, stdout=stream,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return process.wait(timeout=timeout), False
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
            return process.returncode, True
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', type=Path)
    parser.add_argument('--output-root', type=Path, default=ROOT / '.cache/simulation/runs')
    parser.add_argument('--timeout', type=float, default=300)
    args = parser.parse_args()
    if not 0 < args.timeout < 3600:
        parser.error('--timeout must be finite and within (0, 3600) seconds')
    scenario = read(args.scenario)
    if scenario.get('scenario_id') != 'S0_regression' or scenario.get('profile') != 'algorithm':
        parser.error('only algorithm S0_regression is implemented')
    acceptance_path = ROOT / scenario['acceptance']
    acceptance = load_acceptance(acceptance_path)
    if acceptance['profile'] != scenario['profile']:
        parser.error('scenario/acceptance profile mismatch')
    unknown = set(acceptance['tests']) - (COMMANDS.keys() | {'S0_JETSON_SAMPLE'})
    if unknown:
        parser.error(f'unsupported tests: {sorted(unknown)}')
    run_id = str(uuid.uuid4())
    run_dir = args.output_root.resolve() / run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    started_at = datetime.now(timezone.utc).isoformat()
    start = time.monotonic()
    lock = read(ROOT / 'simulation/px4/versions.lock.yaml')
    snapshot = dict(scenario=scenario, acceptance=acceptance, versions=lock)
    snapshot['tool_hashes'] = {str(path.relative_to(ROOT)): file_hash(path)
                              for path in sorted((ROOT / 'scripts').glob('*sim*.py'))}
    write_json(run_dir / 'config.snapshot.json', snapshot)
    shutil.copyfile(args.scenario, run_dir / 'scenario.yaml')
    (run_dir / 'config.snapshot.json').chmod(0o444)
    (run_dir / 'scenario.yaml').chmod(0o444)
    code, status, error = command(['git', 'status', '--short'])
    write_json(run_dir / 'git.json', dict(head=command(['git', 'rev-parse', 'HEAD'])[1],
                                        status=status, status_error=error,
                                        submodules=command(['git', 'submodule', 'status', '--recursive'])[1]))
    code, diff, error = command(['git', 'diff', '--binary', 'HEAD'])
    (run_dir / 'working-tree.diff').write_text(diff + '\n')
    env_result = audit(lock)
    write_json(run_dir / 'environment.json', env_result)
    results = []
    for test_id, declaration in acceptance['tests'].items():
        required = declaration['requirement'] == 'required'
        evidence = ['config.snapshot.json']
        if not required:
            status, reason = 'N/A', declaration['reason']
        elif env_result['status'] != 'PASS':
            status, reason = 'INCONCLUSIVE', 'Dependency audit failed; see environment.json'
            evidence.append('environment.json')
        elif test_id == 'S0_JETSON_SAMPLE':
            status, reason = 'INCONCLUSIVE', 'Required Jetson sample has no implemented target collector or runtime evidence'
        else:
            log = run_dir / f'{test_id}.log'
            env = dict(os.environ, PYTEST_DISABLE_PLUGIN_AUTOLOAD='1', ROS_DOMAIN_ID='180')
            if test_id == 'S0_PX4_BRIDGE_REGRESSION':
                env['PYTHONPATH'] = str(ROOT / 'src/px4_comm_bridge') + os.pathsep + env.get('PYTHONPATH', '')
            code, timed_out = run_command([str(ROOT / 'scripts/with_venv.sh'), *COMMANDS[test_id]],
                                          log, args.timeout, env)
            write_json(run_dir / f'{test_id}.process.json', dict(command=COMMANDS[test_id],
                       exit_code=code, timeout=timed_out))
            status = 'PASS' if code == 0 and not timed_out else 'FAIL'
            reason = f'exit_code={code}, timeout={timed_out}; host regression only'
            evidence.extend([log.name, f'{test_id}.process.json'])
        results.append(dict(test_id=test_id, required=required, status=status,
                            reason=reason, evidence=evidence))
        print(f'{status} {test_id}: {reason}', flush=True)
    result = dict(schema=1, run_id=run_id, profile=scenario['profile'], scope=acceptance['scope'],
                  status=overall(results), started_at=started_at,
                  duration_monotonic_s=time.monotonic() - start,
                  config_hash=digest(snapshot), tests=results)
    validate(result, 'result')
    write_json(run_dir / 'result.json', result)
    print(f"{result['status']}: {run_dir / 'result.json'}", flush=True)
    return {'PASS': 0, 'FAIL': 1, 'INCONCLUSIVE': 2, 'N/A': 0}[result['status']]


if __name__ == '__main__':
    raise SystemExit(main())
