"""Synthetic ROS messages: display math and lifecycle, without Gazebo or flight."""
import time
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, Point32
from nvblox_msgs.msg import Index3D, Mesh, MeshBlock, VoxelBlock, VoxelBlockLayer
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import ColorRGBA, Header
from uav_nav_interfaces.msg import MapSnapshot, PersonObservation, PersonObservationArray
from visualization_msgs.msg import MarkerArray

from uav_nav_sim.map_visualization_core import snapshot_slice, slice_colors, surface_colors
from uav_nav_sim.map_visualizer import MapVisualizer


def header(seconds=100):
    return Header(frame_id='map', stamp=Time(sec=seconds))


def snapshot(seconds=100, **kwargs):
    return MapSnapshot(header=header(seconds), source_stamp=Time(sec=seconds), map_id='fixture',
                       epoch=1, version=1, valid=True, resolution=1., shape=[2, 2, 2],
                       distance=[0., 1., 2., 3., 4., 5., 6., 7.], observed=[1]*8, **kwargs)


def mesh(seconds=100, index=0, clear=False, remove=False):
    return Mesh(header=header(seconds), block_size_m=1.6, clear=clear,
                block_indices=[Index3D(x=index)], blocks=[MeshBlock() if remove else MeshBlock(
                    vertices=[Point32(x=0.), Point32(x=1.), Point32(y=1.)], triangles=[0, 1, 2])])


def test_slice_uses_canonical_xyz_order_and_voxel_centers():
    msg = snapshot()
    msg.observed[3] = 0
    msg.distance[5] = float('nan')
    points, colors = snapshot_slice(msg, 1., 'esdf')
    np.testing.assert_allclose(points, [[.5, .5, 1.5], [1.5, 1.5, 1.5]])
    assert colors[0, 0] > colors[1, 0]  # nearer obstacle is redder
    assert len(snapshot_slice(msg, -1., 'esdf')[0]) == 0
    msg.distance.pop()
    with pytest.raises(ValueError, match='Truncated'):
        snapshot_slice(msg, 1., 'esdf')


def test_clearance_is_conservative_and_unknown_never_green():
    r = .2
    half_diagonal = r*np.sqrt(3)/2
    colors = slice_colors([.8, .8+half_diagonal+.1, .8+half_diagonal+.4], r, 'traversability')
    assert colors[0, 0] > colors[0, 1]  # unsafe center after voxel extent
    assert colors[1, 0] > colors[1, 1] > colors[1, 2]  # caution yellow
    assert colors[2, 1] > colors[2, 0]  # green
    msg = snapshot()
    msg.observed = [0]*8
    assert len(snapshot_slice(msg, 1., 'traversability')[0]) == 0


def test_person_association_and_age_saturation_have_distinct_colors():
    colors = surface_colors([[0, 0, 0], [2, 0, 0]], 'semantic', people=[[0, 0, 0]], radius=.4)
    assert colors[0, 0] > colors[0, 1]
    assert colors[1, 0] == colors[1, 1] == colors[1, 2]
    recent = surface_colors([[0, 0, 0]], 'age', age=0.)
    old = surface_colors([[0, 0, 0]], 'age', age=100.)
    assert recent[0, 2] > recent[0, 0] and old[0, 0] > old[0, 2]


@pytest.fixture
def make_node():
    nodes = []
    def create(mode):
        # Explicit test namespace remaps every application topic, even for callbacks.
        topics = ['/uav/map/snapshot', '/nvblox_node/mesh', '/nvblox_node/tsdf_layer',
                  '/uav/people/observations', '/uav/map/colored_mesh', '/uav/map/colored_voxels',
                  '/uav/map/color_slice', '/uav/map/color_legend', '/uav/map/color_status']
        arguments = ['--ros-args', '-p', f'mode:={mode}']
        for topic in topics:
            arguments += ['-r', topic+':=/test/map_colors'+topic]
        rclpy.init(args=arguments)
        node = MapVisualizer()
        nodes.append(node)
        node.now = lambda: 100.
        return node
    yield create
    for node in nodes:
        node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def test_incremental_mesh_deletion_epoch_reset_and_stale_clearing(make_node):
    node = make_node('age')
    node.mesh_pub = Mock()
    node.mesh_cb(mesh())
    assert not node.blocks  # no admission
    node.snapshot_cb(snapshot())
    node.mesh_cb(mesh(clear=True))
    node.mesh_cb(mesh(index=1))
    node.tick()
    assert len(node.mesh_pub.publish.call_args.args[0].blocks) == 2
    node.mesh_cb(mesh(remove=True))
    node.tick()
    assert len(node.mesh_pub.publish.call_args.args[0].blocks) == 1
    changed = snapshot(101)
    changed.epoch = 2
    node.now = lambda: 101.
    node.snapshot_cb(changed)
    assert not node.blocks
    node.mesh_cb(mesh())
    assert not node.blocks  # queued geometry older than the new map's source boundary
    node.mesh_cb(mesh(101))
    assert node.blocks
    node.received = time.monotonic()-4
    node.tick()
    assert node.mesh_pub.publish.call_args.args[0].clear
    assert not node.mesh_pub.publish.call_args.args[0].blocks


def test_semantic_context_and_capture_expiry_return_to_unknown(make_node):
    node = make_node('semantic')
    node.mesh_pub = Mock()
    node.snapshot_cb(snapshot())
    node.mesh_cb(mesh())
    observations = PersonObservationArray(header=header(), map_id='fixture', map_epoch=1, valid=True,
        point_semantics='visible_torso_surface', observations=[PersonObservation(
            position=Point(), last_observed=Time(sec=100))])
    node.people_cb(observations)
    node.tick()
    assert node.mesh_pub.publish.call_args.args[0].blocks[0].colors[0].r == 1.
    node.now = lambda: 102.5
    node.snapshot_cb(snapshot(102))
    node.tick()
    color = node.mesh_pub.publish.call_args.args[0].blocks[0].colors[0]
    assert color.r == color.g == color.b
    observations.map_epoch = 99
    node.people_cb(observations)
    assert not node.person_points()


def test_weight_uses_native_fusion_scalar_and_replays_full_cache(make_node):
    node = make_node('weight')
    node.voxel_pub = Mock()
    node.snapshot_cb(snapshot())
    raw = VoxelBlockLayer(header=header(), block_size_m=1.6, voxel_size_m=.2, clear=True,
                         block_indices=[Index3D()], blocks=[VoxelBlock(
                             centers=[Point32(), Point32(x=1.)], colors=[ColorRGBA(r=0.), ColorRGBA(r=1.)])])
    node.weight_cb(raw)
    node.tick()
    result = node.voxel_pub.publish.call_args.args[0]
    assert result.clear
    low, high = result.blocks[0].colors
    assert low.b > low.r and high.r > high.b
    assert raw.blocks[0].colors[1].b == 0.  # input preserved
    raw.clear = False
    raw.blocks[0] = VoxelBlock()
    node.weight_cb(raw)
    node.tick()
    assert not node.voxel_pub.publish.call_args.args[0].blocks


def test_clock_rewind_and_invalid_snapshot_clear_display(make_node):
    node = make_node('age')
    node.snapshot_cb(snapshot())
    node.mesh_cb(mesh())
    node.tick()
    node.now = lambda: 90.
    node.tick()
    assert not node.blocks and node.snapshot is None
    bad = snapshot()
    bad.valid = False
    node.snapshot_cb(bad)
    assert not node.ready()


def test_actual_ros_snapshot_delivery_to_colored_slice(make_node):
    node = make_node('esdf')
    probe = rclpy.create_node('map_colors_probe')
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.add_node(probe)
    received = []
    pub = probe.create_publisher(MapSnapshot, '/test/map_colors/uav/map/snapshot',
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    probe.create_subscription(MarkerArray, '/test/map_colors/uav/map/color_slice', received.append, 1)
    def spin_until(predicate):
        deadline = time.monotonic()+5
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.05)
        assert predicate()
    try:
        spin_until(lambda: pub.get_subscription_count() == 1 and node.slice_pub.get_subscription_count() == 1)
        pub.publish(snapshot())
        spin_until(lambda: node.snapshot is not None)
        node.tick()
        spin_until(lambda: any(m.points for a in received for m in a.markers))
        assert len(next(m for a in received for m in a.markers if m.points).points) == 4
        topics = {name for name, _ in node.get_publisher_names_and_types_by_node(node.get_name(), '/')}
        assert not any('/fmu/' in name or name.endswith('/goal') or name.endswith('/trajectory') for name in topics)
    finally:
        executor.shutdown()
        probe.destroy_node()
