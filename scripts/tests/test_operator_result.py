"""Closing an operator session is distinct from source-stop or flight qualification."""
import pytest

from vio_ui_lifecycle import operator_session_result


def valid_result():
    return dict(operator_session_closed=True, operator_preflight_admitted=True,
        operator_final_grounded=True, operator_final_source_ready=True,
        cleanup_confirmed=True, qgc_connected=True, root_exit_codes={'normalizer': 0},
        **{key: {'passed': True} for key in ('renderer', 'sdk_runtime_receipt',
            'sdk16_inertial_state', 'sdk16_loaded_libraries', 'sdk_cpu_affinity_receipt')})


def test_clean_operator_close_does_not_claim_an_automatic_flight():
    result = valid_result()
    result.update(diagnostic_checks_passed=False, experimental_mission_passed=False,
                  flight_pose_session={'source_fault': 'intentional post-close retirement'})
    final = operator_session_result(result)
    assert final['passed'] and final['operator_session_passed']
    assert not final['automatic_flight'] and not final['flight_authorized']
    assert not final['experimental_mission_passed'] and not final['diagnostic_checks_passed']


@pytest.mark.parametrize('key', ['operator_preflight_admitted', 'operator_final_grounded',
    'operator_final_source_ready', 'cleanup_confirmed', 'qgc_connected', 'sdk_runtime_receipt',
    'sdk16_inertial_state', 'sdk16_loaded_libraries', 'sdk_cpu_affinity_receipt', 'root_exit_codes'])
def test_failed_or_missing_session_evidence_stays_failed(key):
    result = valid_result()
    del result[key]
    assert not operator_session_result(result)['passed']


def test_session_error_cannot_be_masked_by_a_good_final_ground_state():
    result = valid_result()
    result['error'] = 'owned process failed'
    assert not operator_session_result(result)['passed']
