"""Keep the dedicated telemetry profile read-only and preserve the control baseline."""
import copy
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from build_px4_vio import CONFIG, ROOT, validate_topics
from sim_validation import read


def profiles():
    return (read(ROOT/'.deps/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml'), read(CONFIG))


def test_profile_keeps_baseline_and_only_adds_read_only_telemetry():
    validate_topics(*profiles())


@pytest.mark.parametrize('change', ['subscription', 'command', 'duplicate', 'missing', 'wrong_type'])
def test_profile_rejects_control_or_telemetry_drift(change):
    base, extended = profiles()
    extended = copy.deepcopy(extended)
    if change == 'subscription': extended['subscriptions'][0]['topic'] += '_changed'
    if change == 'command': extended['publications'].append(dict(topic='/fmu/in/vehicle_command', type='px4_msgs::msg::VehicleCommand'))
    if change == 'duplicate': extended['publications'].append(extended['publications'][-1])
    if change == 'missing': extended['publications'].pop(0)
    if change == 'wrong_type': extended['publications'][-1]['type'] = 'px4_msgs::msg::VehicleCommand'
    with pytest.raises(ValueError): validate_topics(base, extended)
