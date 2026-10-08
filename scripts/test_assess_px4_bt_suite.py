"""Reject misleading BT success and unsafe pause/fallback acceptance."""
import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from assess_px4_bt_suite import assess_case


def fixture(case):
    flight = dict(mock=False, scenario=case, profile='known_region_control', vio_required=False,
        passed=True, result=dict(mock=False, cleanup_confirmed=True, code='SUCCEEDED', reason='LANDED_AND_DISARMED'),
        bt_dispatch_count=1, final_land=True, final_arming=1, action_status=4, bt_exit_code=0,
        steps=[dict(type=s) for s in ('TAKEOFF','NAVIGATE','HOVER','NAVIGATE','RETURN','HOVER','LAND')],
        bt_step_accepts=list(range(7)), bt_step_completes=list(range(7)), landing_cancel_rejected=True)
    log = 'BT_TREE_TERMINAL SUCCESS'
    truth = []
    if case == 'pause-resume':
        flight['pause_resume'] = dict(passed=True, new_child=True, pause_drift_m=.05)
        truth = [dict(phase='PAUSED', mono=t) for t in (1., 2., 3.99)]
    if case in ('cancel', 'runner-stall'):
        flight.update(action_status=5 if case == 'cancel' else 6,
            bt_exit_code=130 if case == 'cancel' else None, bt_step_accepts=[0,1],bt_step_completes=[0],
            cancel_hold=dict(max_drift_m=.1,samples=100,duration_s=29.99,final_landed_disarmed=True),
            bt_progress=dict(loss_to_brake_s=.51))
        flight['result'].update(code='CANCELED' if case == 'cancel' else 'ABORTED',
            reason='STOPPED_AND_HOLDING' if case == 'cancel' else 'BT_PROGRESS_TIMEOUT')
        log = ''
    return dict(passed=True),flight,log,truth


@pytest.mark.parametrize('case', ['full','pause-resume','cancel','runner-stall'])
def test_expected_terminal_semantics(case):
    assert assess_case(case,*fixture(case))['passed']


@pytest.mark.parametrize('fault', ['mock','no_cleanup','duplicate_root','not_landed','wrong_steps','no_tree_success'])
def test_reported_pass_does_not_override_missing_evidence(fault):
    obs,flight,log,truth = fixture('full')
    if fault == 'mock': flight['result']['mock'] = True
    if fault == 'no_cleanup': flight['result']['cleanup_confirmed'] = False
    if fault == 'duplicate_root': flight['bt_dispatch_count'] = 2
    if fault == 'not_landed': flight['final_land'] = False
    if fault == 'wrong_steps': flight['bt_step_completes'] = [0,1]
    if fault == 'no_tree_success': log = ''
    assert not assess_case('full',obs,flight,log,truth)['passed']


@pytest.mark.parametrize('fault', ['drift','nan','missing_truth','short_window','same_child'])
def test_pause_pass_requires_measured_hold_and_new_child(fault):
    obs,flight,log,truth = fixture('pause-resume')
    if fault == 'drift': flight['pause_resume']['pause_drift_m'] = .2
    if fault == 'nan': flight['pause_resume']['pause_drift_m'] = float('nan')
    if fault == 'missing_truth': truth = []
    if fault == 'short_window': truth = truth[:2]
    if fault == 'same_child': flight['pause_resume']['new_child'] = False
    assert not assess_case('pause-resume',obs,flight,log,truth)['passed']


@pytest.mark.parametrize('fault', ['next_step','drift','late_brake','no_fallback','tree_success'])
def test_runner_loss_cannot_pass_on_aborted_alone(fault):
    obs,flight,log,truth = fixture('runner-stall')
    if fault == 'next_step': flight['bt_step_accepts'].append(2)
    if fault == 'drift': flight['cancel_hold']['max_drift_m'] = .2
    if fault == 'late_brake': flight['bt_progress']['loss_to_brake_s'] = 2.
    if fault == 'no_fallback': flight['cancel_hold']['final_landed_disarmed'] = False
    if fault == 'tree_success': log = 'BT_TREE_TERMINAL SUCCESS'
    assert not assess_case('runner-stall',obs,flight,log,truth)['passed']


@pytest.mark.parametrize('fault', [None, 'different_source', 'source_drift', 'same_run'])
@pytest.mark.parametrize('compressed', [False, True])
def test_suite_requires_four_distinct_runs_of_current_frozen_implementation(tmp_path, monkeypatch, fault, compressed):
    import hashlib
    import json
    import assess_px4_bt_suite as suite
    monkeypatch.setattr(suite, 'ROOT', tmp_path)
    for name in ('simulation/missions/W0_flight_sequence.json', 'simulation/safe_regions/W0.json', 'control.py'):
        path = tmp_path/name
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text('{}')
    digest = hashlib.sha256(b'{}').hexdigest()
    argv = ['assess']
    for index,case in enumerate(suite.CASES):
        path = tmp_path/case
        path.mkdir()
        obs,flight,log,truth = fixture(case)
        flight['source_sha256'] = {'control.py': '0'*64 if fault == 'different_source' and index else digest}
        manifest = dict(run_id=str(0 if fault == 'same_run' else index),require_vio=False,
                        vision_fusion_smoke=False,flight_recipe_sha256=digest,flight_profile_sha256=digest)
        for name,value in [('manifest.json',manifest),('observation.json',obs),
                           ('flight-observation.json',flight),('flight-truth.json',truth)]:
            (path/name).write_text(json.dumps(value))
        (path/'bt-runner.log').write_text(log)
        if compressed:
            import gzip
            for name in ('bt-runner.log','flight-truth.json'):
                (path/(name+'.gz')).write_bytes(gzip.compress((path/name).read_bytes()))
                (path/name).unlink()
        argv.extend(['--'+case,str(path)])
    if fault == 'source_drift': (tmp_path/'control.py').write_text('changed')
    output = tmp_path/'assessment.json'
    argv.extend(['--output',str(output)])
    monkeypatch.setattr(sys, 'argv', argv)
    assert suite.main() == (0 if fault is None else 1)
    assert json.loads(output.read_text())['passed'] is (fault is None)
