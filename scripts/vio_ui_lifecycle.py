"""Viewer retention requires current independent PX4 ground telemetry."""
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
