import math
import pytest

from vio_map_startup import MapStartupStability, initialized_window


def fill(gate, start=1., moving=False, rotating=False):
    for i in range(251):
        t = start+i*.04
        yaw = i*.001 if rotating else 0.
        for source in gate.samples:
            gate.observe(source, t, (i*.001 if moving else .001*math.sin(i), 0., 0.),
                         (0., 0., math.sin(yaw/2), math.cos(yaw/2)), t)


def test_full_original_time_stable_ground_window_required():
    gate = MapStartupStability()
    fill(gate)
    assert gate.result(11., 11., True)['passed']
    assert not gate.result(11., 11., False)['passed']
    assert not gate.result(11.3, 11.3, True)['passed']
    assert not gate.result(11., 11.3, True)['passed']


def test_drift_or_rotation_blocks_then_ages_out_of_window():
    for kwargs in ({'moving': True}, {'rotating': True}):
        gate = MapStartupStability()
        fill(gate, **kwargs)
        assert not gate.result(11., 11., True)['passed']
        fill(gate, start=12.)
        assert gate.result(22., 22., True)['passed']


def test_missing_source_short_window_and_replayed_sample_cannot_admit():
    gate = MapStartupStability()
    for i in range(300):
        gate.observe('vio', 10., (0., 0., 0.), (0., 0., 0., 1.), 10.+i*.001)
    assert not gate.result(10., 10.3, True)['passed']
    assert len(gate.samples['vio']) == 1
    fill(gate)
    gate.observe('px4', 2., (0., 0., 0.), (0., 0., 0., 1.), 11.)
    assert not gate.result(11., 11., True)['passed']


def test_nan_or_original_timestamp_gap_cannot_pass_as_stable():
    gate = MapStartupStability()
    fill(gate)
    gate.observe('vio', 11.01, (float('nan'), 0., 0.), (0., 0., 0., 1.), 11.01)
    assert not gate.result(11.01, 11.01, True)['passed']
    fill(gate)
    gate.samples['px4'] = type(gate.samples['px4'])(r for r in gate.samples['px4'] if not 5.<r[0]<6.)
    assert not gate.result(11., 11., True)['passed']


def test_operator_window_requires_thirty_seconds_without_loosening_drift():
    gate = MapStartupStability(window_s=30.)
    fill(gate)
    assert not gate.result(11., 11., True)['passed']
    for i in range(251, 751):
        t = 1+i*.04
        for source in gate.samples:
            gate.observe(source, t, (.001*math.sin(i), 0., 0.), (0., 0., 0., 1.), t)
    assert gate.result(31., 31., True)['passed']
    gate.observe('vio', 31.04, (.04, 0., 0.), (0., 0., 0., 1.), 31.04)
    assert not gate.result(31.04, 31.04, True)['passed']
    for window in (0., 9., 31., float('nan')):
        with pytest.raises(ValueError): MapStartupStability(window_s=window)


def sdk_samples():
    states = [[round((1+i*.04)*1e9), 1., 0., 9.81, 0., *([0.]*9)] for i in range(751)]
    return states, [row[0] for row in states]


def test_recent_initialized_window_is_matched_fresh_continuous_and_unique():
    states, stamps = sdk_samples()
    assert initialized_window(states, stamps, 31., True)['passed']
    assert not initialized_window(states, stamps, 31., False)['passed']
    assert not initialized_window(states, stamps, 31.3, True)['passed']
    assert not initialized_window(states[:250], stamps[:250], 10.96, True)['passed']
    assert not initialized_window(states, [], 31., True)['passed']


@pytest.mark.parametrize('fault', ['invalid', 'nan', 'gravity', 'gap', 'duplicate', 'backward', 'unmatched'])
def test_bad_sdk_evidence_cannot_be_replaced_by_elapsed_time(fault):
    states, stamps = sdk_samples()
    if fault == 'invalid': states[400] = [states[400][0], 0.]
    elif fault == 'nan': states[400][8] = float('nan')
    elif fault == 'gravity': states[400][3] = 0.
    elif fault == 'gap': del states[400:410]
    elif fault == 'duplicate': stamps[400] = stamps[399]
    elif fault == 'backward': states[400][0] = states[398][0]
    elif fault == 'unmatched': stamps[400] += 1000000
    assert not initialized_window(states, stamps, 31., True)['passed']


def test_sdk_initialization_must_retain_full_recent_window_after_restart():
    states, stamps = sdk_samples()
    states[600] = [states[600][0], 0.]
    assert not initialized_window(states, stamps, 31., True)['passed']
    fresh, fresh_stamps = sdk_samples()
    for row in fresh: row[0] += 31*10**9
    fresh_stamps = [s+31*10**9 for s in fresh_stamps]
    assert initialized_window(states+fresh, stamps+fresh_stamps, 62., True)['passed']
