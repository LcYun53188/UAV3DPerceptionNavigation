"""Viewer retention requires current independent PX4 ground telemetry."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

def grounded_for_viewers(last, received, now):
    vehicle,land=last.get('vehicle'),last.get('land')
    return bool(vehicle is not None and land is not None and vehicle.arming_state==1
        and land.landed and 0<=now-received.get('vehicle',float('-inf'))<.75
        and 0<=now-received.get('land',float('-inf'))<1.5)

def manual_session_result(result):
    """A manual-session outcome is separate from automatic mission qualification."""
    result = dict(result)
    passed = all(result.get(key) is True for key in (
        'manual_session_closed', 'manual_preflight_admitted', 'manual_final_grounded',
        'manual_final_source_ready', 'cleanup_confirmed'))
    passed = passed and not result.get('error') and not result.get('evidence_error')
    result.update(passed=passed, manual_session_passed=passed, automatic_flight=False,
                  experimental_mission_passed=False, flight_authorized=False,
                  scope='owned manual operator session; no automatic mission or flight qualification')
    return result


def operator_session_result(result):
    """Judge the closed full-project session before intentional source retirement."""
    result = dict(result)
    passed = all(result.get(key) is True for key in (
        'operator_session_closed', 'operator_preflight_admitted', 'operator_final_grounded',
        'operator_final_source_ready', 'cleanup_confirmed', 'qgc_connected'))
    passed &= all(result.get(key, {}).get('passed') is True for key in (
        'renderer', 'sdk_runtime_receipt', 'sdk16_inertial_state', 'sdk16_loaded_libraries',
        'sdk_cpu_affinity_receipt'))
    passed &= result.get('root_exit_codes', {}).get('normalizer') == 0
    passed &= not result.get('error') and not result.get('evidence_error')
    result.update(passed=bool(passed), operator_session_passed=bool(passed), automatic_flight=False,
        flight_authorized=False, experimental_mission_passed=False,
        scope='owned full-project operator session; mission outcomes and flight qualification separate')
    return result
