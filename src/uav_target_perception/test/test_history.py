from types import SimpleNamespace as NS
from unittest.mock import Mock

import numpy as np
import pytest
from visualization_msgs.msg import Marker, MarkerArray

from uav_target_perception.history import PersonHistory
from uav_target_perception.node import PersonMapper
from uav_target_perception.core import ContextGate, PersonTracker


def test_history_survives_no_live_observations_and_merges_reobserved_place():
    history = PersonHistory()
    history.update([('track-1', [1, 2, 3], 10., .9)])
    history.update([])
    history.update([('track-9', [1.1, 2, 3], 100., .8)])
    assert len(history.records) == 1
    item = next(iter(history.records.values()))
    assert item.id == 1 and item.stamp == 100.
    assert np.allclose(item.position, [1.1, 2, 3])


def test_two_current_people_do_not_merge_into_one_history_record():
    history = PersonHistory()
    history.update([('a', [0, 0, 1], 1., .9), ('b', [.3, 0, 1], 1., .9)])
    history.update([('a', [.05, 0, 1], 2., .9), ('b', [.25, 0, 1], 2., .9)])
    assert len(history.records) == 2
    assert {r.source_track_id for r in history.records.values()} == {'a', 'b'}


def test_history_is_bounded_and_an_old_frame_cannot_replace_a_new_point():
    history = PersonHistory(max_records=2)
    history.update([('a', [0, 0, 1], 10., .9)])
    history.update([('a', [.1, 0, 1], 1., .8)])
    assert history.records[1].stamp == 10.
    history.update([('b', [2, 0, 1], 20., .9), ('c', [4, 0, 1], 30., .9)])
    assert len(history.records) == 2 and 1 not in history.records


@pytest.mark.parametrize('observation', [('a', [np.nan, 0, 0], 1., .8),
                                       ('a', [0, 0, 0], 0., .8), ('a', [0, 0, 0], 1., 2.)])
def test_history_rejects_invalid_geometry_or_source_time(observation):
    with pytest.raises(ValueError):
        PersonHistory().update([observation])


def test_history_markers_remain_after_live_ttl_and_are_cleared_explicitly():
    from uav_nav_interfaces.msg import PersonObservationArray
    history = PersonHistory()
    history.update([('old', [1, 2, 3], 10., .8)])
    state = NS(state_message=lambda _: PersonObservationArray(), observations_pub=Mock(), markers_pub=Mock(),
               history_pub=Mock(), diagnostics_pub=Mock(), options={'retain_history': True},
               history=history, reason='NO_VALID_PERSON_DEPTH', tracker=NS(tracks={}), pending=None, now_s=lambda: 20.)
    PersonMapper.publish_state(state, False)
    live = state.markers_pub.publish.call_args.args[0]
    historical = state.history_pub.publish.call_args.args[0]
    assert all(m.action == Marker.DELETEALL for m in live.markers)
    points = [m for m in historical.markers if m.action == Marker.ADD]
    assert len(points) == 2 and all(m.lifetime.sec == m.lifetime.nanosec == 0 for m in points)
    assert 'history' in points[1].text and '10.0s' in points[1].text
    response = NS(success=False, message='')
    PersonMapper.clear_history(state, None, response)
    assert response.success and not history.records
    assert state.history_pub.publish.call_args.args[0].markers[0].action == Marker.DELETEALL


def test_clear_history_does_not_reinsert_cached_live_frame():
    from uav_nav_interfaces.msg import PersonObservationArray, PersonObservation
    from rclpy.time import Time
    message = PersonObservationArray(observations=[PersonObservation(track_id='a',
        last_observed=Time(seconds=10).to_msg(), detection_confidence=.9)])
    state = NS(state_message=lambda _: message, observations_pub=Mock(), markers_pub=Mock(),
        history_pub=Mock(), diagnostics_pub=Mock(), options={'retain_history': True, 'track_ttl': 1.5},
        history=PersonHistory(), reason='OK', tracker=NS(tracks={}), pending=None, now_s=lambda: 20.)
    PersonMapper.publish_state(state, True)
    assert state.history.records
    PersonMapper.clear_history(state, None, NS(success=False, message=''))
    PersonMapper.publish_state(state, True)
    assert not state.history.records
    message.observations[0].last_observed = Time(seconds=21).to_msg()
    PersonMapper.publish_state(state, True)
    assert state.history.records


def test_same_coordinate_context_loss_keeps_history_but_new_alignment_clears_it():
    from uav_nav_interfaces.msg import PlanningContext
    from rclpy.time import Time
    state = NS(options={'context_source': 'planning_context'}, gate=ContextGate(),
               history=PersonHistory(), tracker=PersonTracker(), pending=None,
               last_processed=None, map_context=None, publish_state=Mock())
    message = PlanningContext(map_id='m', map_epoch=1, localization_session='s', alignment_id='a',
                              alignment_generation=1, context_id='valid-token', valid=True)
    message.header.frame_id = 'map'
    message.header.stamp = Time(seconds=1).to_msg()
    PersonMapper.context_cb(state, message)
    state.history.update([('a', [1, 2, 3], 1., .9)])
    message.valid, message.context_id = False, ''
    message.header.stamp = Time(seconds=2).to_msg()
    PersonMapper.context_cb(state, message)
    assert len(state.history.records) == 1
    message.alignment_generation = 2
    message.header.stamp = Time(seconds=3).to_msg()
    PersonMapper.context_cb(state, message)
    assert not state.history.records


def test_clock_rollback_clears_retained_positions():
    history = PersonHistory()
    history.update([('old', [1, 2, 3], 10., .9)])
    state = NS(history=history, history_barrier=20., last_history_clock=20., now_s=lambda: 5.,
               gate=NS(ready=lambda *_: False), get_publishers_info_by_topic=lambda _: [],
               options={'context_topic': '/fixture'}, tracker=Mock(), process_pending=Mock(), publish_state=Mock())
    PersonMapper.tick(state)
    assert not history.records and state.history_barrier == 0.
