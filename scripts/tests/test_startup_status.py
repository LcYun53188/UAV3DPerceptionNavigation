"""Prevent historical READY and missing operator input from masquerading as VIO waits."""
import json

from simulation.readiness import startup_status, status_text


NOW = 100_000_000_000


def record(folder, name, **values):
    (folder/(name+'.json')).write_text(json.dumps(values))


def test_old_admission_cannot_mark_stopped_or_stale_session_ready(tmp_path):
    record(tmp_path, 'flight-admission', sensor_checks_passed=True)
    record(tmp_path, 'pocket-live', system_ns=NOW-3_000_000_000,
           source_preflight_admitted=True, source_ready=True, operator_input_ready=True)
    assert startup_status(tmp_path, active=False, now_ns=NOW)['phase'] == 'STOPPED'
    assert startup_status(tmp_path, now_ns=NOW)['phase'] == 'TELEMETRY_STALE'


def test_input_wait_is_separate_from_vio_and_task_preparation(tmp_path):
    live = dict(system_ns=NOW, source_preflight_admitted=True, source_ready=True,
                operator_input_ready=False, input_evidence={'native': {'samples': 0}})
    record(tmp_path, 'pocket-live', **live)
    status = startup_status(tmp_path, now_ns=NOW)
    assert status['phase'] == 'WAITING_OPERATOR_INPUT'
    assert status['native_samples'] == 0
    record(tmp_path, 'pocket-live', **dict(live, operator_input_ready=True))
    status = startup_status(tmp_path, now_ns=NOW)
    assert status['phase'] == 'READY_FOR_OPERATOR'
    assert status['auto_prepared'] is False
    record(tmp_path, 'pocket-live', **dict(live, source_ready=False, source_reason='VIO_RECEIVE_GAP'))
    assert startup_status(tmp_path, now_ns=NOW)['phase'] == 'SOURCE_UNAVAILABLE'


def test_source_ready_cannot_clear_latched_preflight_failure(tmp_path):
    record(tmp_path, 'live-session', state='POCKET_PREFLIGHT_BLOCKED', reason='VIO_RECEIVE_GAP')
    record(tmp_path, 'pocket-live', system_ns=NOW, source_ready=True)
    status = startup_status(tmp_path, now_ns=NOW)
    assert status['phase'] == 'PREFLIGHT_BLOCKED'
    assert 'VIO_RECEIVE_GAP' in status_text(status)


def test_progress_reports_actual_window_and_not_only_ready(tmp_path):
    record(tmp_path, 'map-startup-stability', system_ns=NOW, source_reason='READY',
           window_s=30., elapsed_s=40., sdk_startup={'checks': {'initialized': True}},
           sources={'vio': {'span_s': 20., 'position_span_m': .04}})
    status = startup_status(tmp_path, now_ns=NOW)
    assert status['phase'] == 'STABILITY_WINDOW'
    assert '4.00 cm / 3 cm' in status_text(status)
    record(tmp_path, 'map-admission', system_ns=NOW)
    assert startup_status(tmp_path, now_ns=NOW)['phase'] == 'MAP_SETTLING'
    assert startup_status(tmp_path, now_ns=NOW+20_000_000_000)['phase'] == 'TELEMETRY_STALE'


def test_partially_written_records_do_not_crash_status(tmp_path):
    (tmp_path/'pocket-live.json').write_text('{')
    assert startup_status(tmp_path, now_ns=NOW)['phase'] == 'STARTING'
