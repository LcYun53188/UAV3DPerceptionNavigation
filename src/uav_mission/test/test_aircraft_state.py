import math

import pytest

from uav_mission.aircraft_state import AircraftStateAggregator


def fresh():
    a = AircraftStateAggregator()
    a.observe('status', 10., dict(arming_state=2, nav_state=14, failsafe=False), 10., 1.)
    a.observe('land', 10., dict(landed=False), 10., 1.)
    a.observe('local', 10., dict(x=0., y=0., z=-1., xy_valid=True, z_valid=True,
                                eph=.1, epv=.2, dead_reckoning=False,
                                xy_reset_counter=0, z_reset_counter=0,
                                heading_reset_counter=0), 10., 1.)
    return a


def test_startup_and_identity():
    a, b = AircraftStateAggregator(), AircraftStateAggregator()
    s = a.snapshot(0., 1.)
    assert s['arming'].value == 'UNKNOWN' and not s['arming'].valid
    assert not s['navigation'].valid and not s['ownership'].valid
    assert a.instance != b.instance
    assert a.snapshot(0., 1.1)['sequence'] == 2


def test_dimensions_and_no_control_authority():
    s = fresh().snapshot(10.1, 1.1)
    assert s['arming'].value == 'ARMED'
    assert s['ground'].value == 'IN_AIR'
    assert s['mode'].value == 'OFFBOARD' and s['raw_nav_state'] == 14
    assert s['localization'].value == 'LOCAL_POSITION'
    assert s['autopilot_exception'].value == 'NOMINAL'
    assert s['navigation'].value == 'NOT_READY'
    assert not s['ownership'].valid


def test_independent_staleness_and_disarmed_is_not_landed():
    a = fresh()
    a.observe('status', 10.4, dict(arming_state=1, nav_state=2), 10.4, 1.4)
    s = a.snapshot(10.6, 1.6)
    assert s['arming'].value == 'DISARMED'
    assert s['ground'].value == 'UNKNOWN' and not s['ground'].valid
    assert s['localization'].value == 'UNKNOWN'


@pytest.mark.parametrize('ros,mono', [(10.6, 1.1), (10.1, 1.6)])
def test_both_freshness_gates(ros, mono):
    s = fresh().snapshot(ros, mono)
    assert not s['arming'].valid and s['link'].value == 'STALE'


def test_duplicates_cannot_refresh_and_source_reset_latches():
    a = fresh()
    assert not a.observe('status', 10., {}, 10.4, 1.4)
    assert not a.snapshot(10.6, 1.6)['arming'].valid
    assert not a.observe('status', 9., {}, 10.7, 1.7)
    assert not a.observe('status', 10.8, {}, 10.8, 1.8)
    assert a.snapshot(10.8, 1.8)['arming'].reason == 'SOURCE_TIME_RESET'


@pytest.mark.parametrize('ros,mono,reason', [
    (9., 1.1, 'ROS_TIME_RESET'), (10., 1.6, 'ROS_TIME_STALLED'),
    (10.1, .9, 'MONOTONIC_TIME_RESET'), (math.nan, 1.1, 'NONFINITE_CLOCK')])
def test_clock_fault_is_latched(ros, mono, reason):
    a = fresh()
    assert a.snapshot(ros, mono)['arming'].reason == reason
    assert not a.observe('status', 11., {}, 11., 2.)
    assert a.snapshot(11., 2.)['arming'].reason == reason


@pytest.mark.parametrize('stamp', [math.nan, math.inf, -1., 0., 11., 9.])
def test_invalid_source_not_accepted(stamp):
    a = AircraftStateAggregator()
    assert not a.observe('status', stamp, {}, 10., 1.)
    assert a.snapshot(10., 1.)['arming'].reason == 'NO_SAMPLE'


@pytest.mark.parametrize('fields,expected', [
    (dict(landed=True), 'ON_GROUND'),
    (dict(landed=True, horizontal_movement=True), 'TRANSITION'),
    (dict(landed=False, horizontal_movement=True), 'IN_AIR'),
    (dict(landed=False, ground_contact=True), 'TRANSITION'),
    (dict(landed=True, freefall=True), 'TRANSITION')])
def test_ground_conflicts(fields, expected):
    a = fresh()
    a.observe('land', 10.1, fields, 10.1, 1.1)
    assert a.snapshot(10.1, 1.1)['ground'].value == expected


@pytest.mark.parametrize('mode,expected', [(18, 'AUTO_LAND'), (2, 'PILOT'), (5, 'AUTO_OTHER'), (31, 'UNKNOWN')])
def test_modes_and_failsafe(mode, expected):
    a = fresh()
    a.observe('status', 10.1, dict(arming_state=2, nav_state=mode, failsafe=True), 10.1, 1.1)
    s = a.snapshot(10.1, 1.1)
    assert s['mode'].value == expected
    assert s['autopilot_exception'].value == 'FAILSAFE'
    assert not s['ownership'].valid


@pytest.mark.parametrize('change', [dict(x=math.nan), dict(eph=2.), dict(dead_reckoning=True)])
def test_localization_quality(change):
    a = fresh()
    local = dict(a.samples['local'][2], **change)
    a.observe('local', 10.1, local, 10.1, 1.1)
    assert a.snapshot(10.1, 1.1)['localization'].value != 'LOCAL_POSITION'


def test_local_reset_requires_new_instance():
    a = fresh()
    local = dict(a.samples['local'][2], xy_reset_counter=1)
    assert not a.observe('local', 10.1, local, 10.1, 1.1)
    assert a.snapshot(10.1, 1.1)['localization'].reason == 'LOCAL_POSITION_RESET'
    assert a.snapshot(10.1, 1.1)['arming'].valid


@pytest.mark.parametrize('kwargs', [dict(max_age_s=0), dict(clock_stall_s=-1),
                                    dict(future_tolerance_s=-1), dict(max_age_s=math.nan)])
def test_invalid_parameters(kwargs):
    with pytest.raises(ValueError):
        AircraftStateAggregator(**kwargs)


def test_unknown_mode_retains_raw_diagnostics_while_fresh():
    a = fresh()
    a.observe('status', 10.1, dict(arming_state=1, nav_state=31), 10.1, 1.1)
    s = a.snapshot(10.1, 1.1)
    assert not s['mode'].valid and s['raw_nav_state'] == 31
    assert a.snapshot(10.7, 1.7)['raw_nav_state'] == -1


def test_altitude_requires_quality_evidence():
    a = fresh()
    local = dict(a.samples['local'][2], epv=2.)
    a.observe('local', 10.1, local, 10.1, 1.1)
    s = a.snapshot(10.1, 1.1)
    assert s['localization'].value == 'INVALID' and not s['localization'].valid


def test_lost_link_cannot_keep_armed_fact():
    s = fresh().snapshot(12., 3.)
    assert s['link'].value == 'LOST' and not s['link'].valid
    assert s['arming'].value == 'UNKNOWN' and not s['arming'].valid
