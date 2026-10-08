#!/usr/bin/env python3
"""Read-only acceptance of four owned BT/SITL runs; never starts flight or writes FMU."""
import argparse
import hashlib
import gzip
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
CASES = ('full', 'pause-resume', 'cancel', 'runner-stall')


def bounded(value, limit):
    return isinstance(value, (int, float)) and math.isfinite(value) and 0 <= value <= limit


def assess_case(case, observation, flight, runner_log, truth):
    result = flight.get('result', {})
    checks = dict(
        supervisor_passed=observation.get('passed') is True,
        real_backend=flight.get('mock') is False and result.get('mock') is False,
        scenario_matches=flight.get('scenario') == case,
        baseline_profile=flight.get('profile') == 'known_region_control' and flight.get('vio_required') is False,
        flight_audit_passed=flight.get('passed') is True,
        cleanup_confirmed=result.get('cleanup_confirmed') is True,
        one_root=flight.get('bt_dispatch_count') == 1,
        landed_disarmed=flight.get('final_land') is True and flight.get('final_arming') == 1,
    )
    if case in ('full', 'pause-resume'):
        steps = flight.get('steps', [])
        indices = list(range(len(steps)))
        checks.update(
            real_success=flight.get('action_status') == 4 and result.get('code') == 'SUCCEEDED'
                         and result.get('reason') == 'LANDED_AND_DISARMED',
            tree_success=flight.get('bt_exit_code') == 0 and 'BT_TREE_TERMINAL SUCCESS' in runner_log,
            exact_step_sequence=bool(steps) and flight.get('bt_step_accepts') == indices
                                and flight.get('bt_step_completes') == indices,
            native_land_not_cancelled=flight.get('landing_cancel_rejected') is True,
        )
    else:
        hold = flight.get('cancel_hold', {})
        checks.update(
            stopped_before_next_step=flight.get('bt_step_accepts') == [0, 1]
                                     and flight.get('bt_step_completes') == [0],
            bounded_hold=bounded(hold.get('max_drift_m'), .15)
                         and isinstance(hold.get('samples'), int) and hold['samples'] >= 2
                         and bounded(hold.get('duration_s'), 30.5) and hold['duration_s'] >= 29.,
            fallback_landed=hold.get('final_landed_disarmed') is True,
            tree_not_success='BT_TREE_TERMINAL SUCCESS' not in runner_log,
        )
        if case == 'cancel':
            checks['cancel_result'] = (flight.get('action_status') == 5 and result.get('code') == 'CANCELED'
                                      and result.get('reason') == 'STOPPED_AND_HOLDING'
                                      and flight.get('bt_exit_code') == 130)
        else:
            checks['progress_timeout'] = (flight.get('action_status') == 6 and result.get('code') == 'ABORTED'
                                         and result.get('reason') == 'BT_PROGRESS_TIMEOUT')
            checks['brake_latency'] = bounded(flight.get('bt_progress', {}).get('loss_to_brake_s'), .75)
    if case == 'pause-resume':
        pause = flight.get('pause_resume', {})
        points = [p for p in truth if p.get('phase') == 'PAUSED']
        coverage = points[-1]['mono'] - points[0]['mono'] if len(points) >= 2 else 0.
        checks['pause_stillness'] = (pause.get('passed') is True and pause.get('new_child') is True
                                    and bounded(pause.get('pause_drift_m'), .15) and coverage >= 2.5)
    return dict(passed=all(checks.values()), checks=checks)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for case in CASES:
        parser.add_argument('--'+case, type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    report = dict(scope='W0 BT/SITL stage 1 only; no VIO or obstacle avoidance acceptance', cases={})
    hashes = []
    run_ids = []
    for case in CASES:
        directory = getattr(args, case.replace('-', '_'))
        names = ('manifest.json', 'observation.json', 'flight-observation.json', 'bt-runner.log', 'flight-truth.json')
        payloads = {n:((directory/n).read_bytes() if (directory/n).is_file()
                       else gzip.decompress((directory/(n+'.gz')).read_bytes())) for n in names}
        manifest = json.loads(payloads['manifest.json'])
        flight = json.loads(payloads['flight-observation.json'])
        outcome = assess_case(case, json.loads(payloads['observation.json']), flight,
                              payloads['bt-runner.log'].decode(), json.loads(payloads['flight-truth.json']))
        outcome['checks']['frozen_w0'] = (
            manifest.get('require_vio') is False and manifest.get('vision_fusion_smoke') is False
            and manifest.get('flight_recipe_sha256') == hashlib.sha256(
                (ROOT/'simulation/missions/W0_flight_sequence.json').read_bytes()).hexdigest()
            and manifest.get('flight_profile_sha256') == hashlib.sha256(
                (ROOT/'simulation/safe_regions/W0.json').read_bytes()).hexdigest())
        outcome['passed'] = all(outcome['checks'].values())
        outcome.update(run_id=manifest['run_id'], evidence_directory=str(directory),
                       evidence_sha256={n:hashlib.sha256(v).hexdigest() for n,v in payloads.items()})
        report['cases'][case] = outcome
        hashes.append(flight.get('source_sha256', {}))
        run_ids.append(manifest['run_id'])
    report['distinct_runs'] = len(set(run_ids)) == len(CASES)
    report['same_implementation'] = bool(hashes[0]) and all(h == hashes[0] for h in hashes)
    report['current_implementation'] = bool(hashes[0]) and all(
        (ROOT/name).is_file() and hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == digest
        for name, digest in hashes[0].items())
    report['passed'] = (report['distinct_runs'] and report['same_implementation']
                        and report['current_implementation'] and all(c['passed'] for c in report['cases'].values()))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print('BT stage 1:', 'PASS' if report['passed'] else 'FAIL', args.output)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
