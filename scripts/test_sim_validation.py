"""Acceptance false-pass prevention and owned-process timeout regression."""
import copy
import json
import os
from pathlib import Path
import sys

import jsonschema
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from sim_validation import digest, evaluate, load_acceptance, overall, validate
from run_sim_scenario import run_command


def metric():
    value = dict(metric_id='control_interval', source='gateway.output', clock_domain='monotonic',
                 operator='<', threshold=0.1, unit='s', window=30, timeout=40,
                 on_violation='FAIL', applicable_tests=['T03'], evidence_path='control.json',
                 freeze_stage='S4')
    value['config_hash'] = digest(value)
    return value


def acceptance():
    return dict(schema=1, profile='algorithm', scope='test',
                tests={'T03': dict(requirement='required', reason='navigation')}, metrics=[metric()])


def save(tmp_path, value):
    path = tmp_path / 'acceptance.yaml'
    path.write_text(json.dumps(value))
    return path


def test_valid_config(tmp_path):
    assert load_acceptance(save(tmp_path, acceptance()))['metrics'][0]['threshold'] == 0.1


@pytest.mark.parametrize('missing', ['threshold', 'source', 'clock_domain', 'evidence_path', 'config_hash'])
def test_incomplete_metric_rejected(tmp_path, missing):
    value = acceptance()
    del value['metrics'][0][missing]
    with pytest.raises(jsonschema.ValidationError):
        load_acceptance(save(tmp_path, value))


def test_threshold_change_requires_new_hash(tmp_path):
    value = acceptance()
    value['metrics'][0]['threshold'] = 10
    with pytest.raises(ValueError, match='stale config_hash'):
        load_acceptance(save(tmp_path, value))


def test_na_requires_reason(tmp_path):
    value = acceptance()
    value['tests']['T03'] = dict(requirement='N/A', reason='')
    with pytest.raises(jsonschema.ValidationError):
        load_acceptance(save(tmp_path, value))


def test_duplicate_metrics_rejected(tmp_path):
    value = acceptance()
    value['metrics'].append(copy.deepcopy(value['metrics'][0]))
    with pytest.raises(ValueError, match='duplicate'):
        load_acceptance(save(tmp_path, value))


def test_undeclared_metric_test_rejected(tmp_path):
    value = acceptance()
    value['metrics'][0]['applicable_tests'] = ['T99']
    value['metrics'][0]['config_hash'] = digest({k: v for k, v in value['metrics'][0].items() if k != 'config_hash'})
    with pytest.raises(ValueError, match='undeclared'):
        load_acceptance(save(tmp_path, value))


@pytest.mark.parametrize('samples', [[], [float('nan')], [float('inf')], [None], [True]])
def test_missing_or_invalid_observations_do_not_pass(samples):
    assert evaluate(metric(), samples) == 'INCONCLUSIVE'


def test_threshold_is_strict():
    assert evaluate(metric(), [0.09]) == 'PASS'
    assert evaluate(metric(), [0.1]) == 'FAIL'


@pytest.mark.parametrize('statuses,expected', [([], 'INCONCLUSIVE'),
    (['PASS', 'INCONCLUSIVE'], 'INCONCLUSIVE'), (['PASS', 'N/A'], 'INCONCLUSIVE'),
    (['PASS', 'FAIL'], 'FAIL'), (['PASS', 'PASS'], 'PASS')])
def test_required_evidence_not_skipped(statuses, expected):
    assert overall([dict(required=True, status=s) for s in statuses]) == expected


def test_owned_process_timeout(tmp_path):
    code, timed_out = run_command([sys.executable, '-c', 'import time; time.sleep(30)'],
                                  tmp_path / 'timeout.log', .05, os.environ.copy())
    assert timed_out and code != 0


def test_successful_command_preserves_log(tmp_path):
    log = tmp_path / 'success.log'
    assert run_command([sys.executable, '-c', 'print("evidence")'], log, 2, os.environ.copy()) == (0, False)
    assert log.read_text().strip() == 'evidence'


def test_full_acceptance_without_metrics_is_invalid(tmp_path):
    value = acceptance()
    value['metrics'] = []
    with pytest.raises(ValueError, match='metrics not frozen'):
        load_acceptance(save(tmp_path, value))


def test_shipped_s0_acceptance_loads():
    from sim_validation import ROOT
    assert load_acceptance(ROOT / 'simulation/acceptance/algorithm_s0.yaml')['profile'] == 'algorithm'
