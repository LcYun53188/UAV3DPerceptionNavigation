import math

from vio_map_startup import MapStartupStability


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
