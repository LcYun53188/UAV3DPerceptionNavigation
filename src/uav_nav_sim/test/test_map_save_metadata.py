"""PX4 maps carry their actual localization identity, distinct from truth maps."""
import asyncio
import json
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import AsyncMock, Mock

from rclpy.clock import ClockType
from rclpy.time import Time
from uav_nav_interfaces.msg import MapSnapshot
from uav_nav_sim.map_session import MapSession


def test_save_preserves_px4_localization_binding(tmp_path):
    snapshot = MapSnapshot(valid=True)
    snapshot.header.stamp = Time(seconds=10).to_msg()
    values = dict(map_timeout=2., localization_source='px4_vio_local_enu',
                  localization_session='owned-px4-session', alignment_id='ekf-odom',
                  aabb_min=[-5., -5., 0.], aabb_size=[10., 10., 4.])
    async def write(request):
        Path(request.file_path).write_bytes(b'owned map payload')
        return NS(success=True)
    node = NS(mode='mapping', latest=snapshot, map_id='owned-map', scene_id='warehouse',
        build_id='build', resolution=.2, busy=False, get_logger=lambda: Mock(),
        get_clock=lambda: NS(now=lambda: Time(seconds=10.1, clock_type=ClockType.ROS_TIME)),
        get_parameter=lambda key: NS(value=values[key]), begin_operation=Mock(), delay=AsyncMock(),
        save_client=NS(service_is_ready=lambda: True, call_async=write))
    dest = tmp_path/'map'
    response = asyncio.run(MapSession.save(node, NS(file_path=str(dest)), NS(success=False)))
    assert response.success
    manifest = json.loads((dest/'manifest.json').read_text())
    assert manifest['localization'] == 'px4_vio_local_enu'
    assert manifest['localization_session'] == 'owned-px4-session'
    assert manifest['alignment_id'] == 'ekf-odom'
