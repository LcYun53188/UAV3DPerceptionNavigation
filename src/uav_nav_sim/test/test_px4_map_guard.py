"""The full project must authorize maps using real PX4 ground telemetry."""
from types import SimpleNamespace as NS
from unittest.mock import Mock
import time

import pytest

from uav_nav_sim.map_session import MapSession


def state():
    return NS(operation_guard='px4_ground', busy=False, epoch=1, version=3,
        invalidate=Mock(), get_clock=lambda: NS(now=lambda: NS(nanoseconds=10*10**9)),
        get_publishers_info_by_topic=lambda topic: [object()],
        px4_ground_samples=dict(
            vehicle=(NS(timestamp=10*10**6, arming_state=1), time.monotonic(), '/vehicle'),
            land=(NS(timestamp=10*10**6, landed=True), time.monotonic(), '/land')))


def test_map_operation_uses_px4_ground_without_a_synthetic_hold_publisher():
    node = state()
    MapSession.begin_operation(node)
    assert node.busy and node.epoch == 2
    node.invalidate.assert_called_once()


@pytest.mark.parametrize('failure', ['armed', 'airborne', 'missing', 'stale_receive', 'stale_source',
                                   'future_source', 'duplicate_writer', 'busy'])
def test_unsafe_or_ambiguous_px4_state_does_not_change_map(failure):
    node = state()
    if failure == 'armed': node.px4_ground_samples['vehicle'][0].arming_state = 2
    if failure == 'airborne': node.px4_ground_samples['land'][0].landed = False
    if failure == 'missing': del node.px4_ground_samples['land']
    if failure == 'stale_receive':
        msg, _, topic = node.px4_ground_samples['land']
        node.px4_ground_samples['land'] = (msg, time.monotonic()-2, topic)
    if failure == 'stale_source': node.px4_ground_samples['vehicle'][0].timestamp = 8*10**6
    if failure == 'future_source': node.px4_ground_samples['vehicle'][0].timestamp = 11*10**6
    if failure == 'duplicate_writer': node.get_publishers_info_by_topic = lambda topic: [1, 2]
    if failure == 'busy': node.busy = True
    with pytest.raises(ValueError):
        MapSession.begin_operation(node)
    assert node.epoch == 1 and node.version == 3
    node.invalidate.assert_not_called()
