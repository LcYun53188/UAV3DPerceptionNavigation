"""Mesh export publishes only completed files and releases the input gate on errors."""
import asyncio
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock, Mock

import pytest
from rclpy.clock import ClockType
from rclpy.time import Time
from uav_nav_interfaces.msg import MapSnapshot
from uav_nav_sim.map_session import MapSession


@pytest.fixture
def session():
    snapshot = MapSnapshot(valid=True)
    snapshot.header.stamp = Time(seconds=10).to_msg()
    node = NS(latest=snapshot, mode='mapping', busy=False,
              get_clock=lambda: NS(now=lambda: Time(seconds=10.1, clock_type=ClockType.ROS_TIME)),
              get_parameter=lambda key: NS(value=2.), mesh_client=Mock(),
              get_logger=lambda: Mock(), delay=AsyncMock())
    def begin():
        node.busy = True
    node.begin_operation = Mock(side_effect=begin)
    async def write(req):
        Path(req.file_path).write_text('ply\nformat ascii 1.0\nend_header\n')
        return NS(success=True)
    node.mesh_client.call_async = AsyncMock(side_effect=write)
    return node


def export(node, path):
    return asyncio.run(MapSession.export_mesh(node, NS(file_path=str(path)), NS(success=False)))


@pytest.mark.parametrize('mode', ['mapping', 'localization'])
def test_completed_export_creates_parent_and_releases_gate(session, tmp_path, mode):
    session.mode = mode
    if mode == 'localization':
        session.latest.header.stamp = Time(seconds=1).to_msg()
    dest = tmp_path / 'new' / 'mesh.ply'
    assert export(session, dest).success
    assert dest.read_text().startswith('ply\n')
    assert not session.busy
    assert list(dest.parent.iterdir()) == [dest]


@pytest.mark.parametrize('failure', ['rejected', 'missing', 'exception', 'race'])
def test_failed_export_does_not_publish_partial_file(session, tmp_path, failure):
    dest = tmp_path / 'mesh.ply'
    async def fail(req):
        if failure == 'exception':
            raise RuntimeError('service failed')
        if failure == 'race':
            Path(req.file_path).write_text('ply\n')
            dest.write_text('other writer')
        return NS(success=failure != 'rejected')
    session.mesh_client.call_async.side_effect = fail
    assert not export(session, dest).success
    assert not session.busy
    assert list(tmp_path.iterdir()) == ([dest] if failure == 'race' else [])
    if failure == 'race':
        assert dest.read_text() == 'other writer'


@pytest.mark.parametrize('reason', ['stale', 'invalid', 'exists', 'suffix', 'service', 'executing'])
def test_rejected_export_keeps_map_session(session, tmp_path, reason):
    dest = tmp_path / 'mesh.ply'
    if reason == 'stale':
        session.latest.header.stamp = Time(seconds=1).to_msg()
    elif reason == 'invalid':
        session.latest.valid = False
    elif reason == 'exists':
        dest.write_text('existing')
    elif reason == 'suffix':
        dest = dest.with_suffix('.obj')
    elif reason == 'service':
        session.mesh_client.service_is_ready.return_value = False
    elif reason == 'executing':
        session.begin_operation.side_effect = ValueError('HOLD required')
    assert not export(session, dest).success
    assert not session.busy
    session.mesh_client.call_async.assert_not_called()
