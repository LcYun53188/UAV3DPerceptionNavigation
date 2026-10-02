from types import SimpleNamespace as NS
import json
import numpy as np
import pytest
from uav_nav_sim.core import Grid, CellState, parse_esdf, spline, validate_trajectory, sha256, validate_bundle


def response(shape=(2,3,4), offset=0, pad=0):
    sz=shape[2]+pad
    sy=shape[1]*sz
    data=np.full(shape[0]*sy+offset,2.0,dtype=float)
    return NS(success=True, header=NS(frame_id='map'), voxel_size_m=0.1,
              origin_m=NS(x=-1.0,y=-2.0,z=-3.0), esdf_and_gradients=NS(
                layout=NS(data_offset=offset,dim=[NS(label=a,size=n,stride=s) for a,n,s in zip('xyz',shape,(shape[0]*sy,sy,sz))]),data=data))


def test_parse_negative_origin_offset_and_strides():
    msg=response(offset=5,pad=2)
    msg.esdf_and_gradients.data[5+18+6+2]=-1000
    grid=parse_esdf(msg)
    assert grid.distance.shape==(2,3,4)
    assert not grid.observed[1,1,2]
    assert grid.state([-0.85,-1.85,-2.75])==CellState.UNKNOWN
    assert grid.state([-1.001,-2,-3])==CellState.OUT_OF_MAP


@pytest.mark.parametrize('value',[-1000,float('nan'),float('inf')])
def test_unknown_never_free(value):
    msg=response()
    msg.esdf_and_gradients.data[0]=value
    grid=parse_esdf(msg)
    assert grid.state([-0.95,-1.95,-2.95])==CellState.UNKNOWN
    assert grid.collision([-0.95,-1.95,-2.95],0)


@pytest.mark.parametrize('fault',['failure','frame','stride','truncated','shape'])
def test_invalid_response_rejected(fault):
    msg=response()
    if fault=='failure': msg.success=False
    if fault=='frame': msg.header.frame_id='odom'
    if fault=='stride': msg.esdf_and_gradients.layout.dim[1].stride=1
    if fault=='truncated': msg.esdf_and_gradients.data=[]
    if fault=='shape': msg.esdf_and_gradients.layout.dim[0].size=0
    with pytest.raises(ValueError): parse_esdf(msg)


def free_grid():
    return Grid(np.array([-2.,-2.,-2.]),0.1,np.full((50,50,50),3,dtype=np.float32),np.ones((50,50,50),dtype=bool))


def test_curve_and_dynamic_limits():
    controls=np.array([[0,0,0]]*3+[[0.4,0,0],[0.8,0,0]]+[[1.2,0,0]]*3,dtype=float)
    curve=spline(controls,2)
    validate_trajectory(curve,free_grid(),0.2,[1,1,1])
    with pytest.raises(ValueError,match='Dynamic'): validate_trajectory(curve,free_grid(),0.2,[0.01,1,1])


def test_curve_interior_collision_even_when_controls_free():
    grid=free_grid()
    distance=grid.distance.copy();distance[25,20,20]=-0.1
    blocked=Grid(grid.origin.copy(),0.1,distance,grid.observed.copy())
    controls=np.array([[0,0,0]]*3+[[1,0,0]]*4,dtype=float)
    assert all(not blocked.collision(p,0.01) for p in controls)
    with pytest.raises(ValueError,match='intersects'): validate_trajectory(spline(controls,2),blocked,0.05,[1,1,1])


def test_unknown_body_volume_and_boundary():
    grid=free_grid(); observed=grid.observed.copy();observed[21,20,20]=False
    grid=Grid(grid.origin.copy(),0.1,grid.distance.copy(),observed)
    assert grid.collision([0.05,0.05,0.05],0.2)
    assert grid.collision([-1.95,0,0],0.2)


@pytest.mark.parametrize('value',[-.1,0.,float('nan'),float('inf')])
def test_clear_centre_does_not_hide_blocked_voxel_inside_body(value):
    g=free_grid()
    distance=g.distance.copy()
    distance[22,20,20]=value
    blocked=Grid(g.origin.copy(),g.resolution,distance,g.observed.copy())
    assert blocked.distance[20,20,20]==3.
    assert blocked.collision([.05,.05,.05],.3)
    assert not blocked.collision([.05,.05,.05],.05)


def test_bundle_checksum_scene_and_resolution(tmp_path):
    (tmp_path/'static_map.nvblx').write_bytes(b'test fixture')
    manifest=dict(schema=1,frame_id='map',scene_id='scene',resolution=0.1,localization='gazebo_world_identity',sha256=sha256(tmp_path/'static_map.nvblx'))
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    validate_bundle(tmp_path,'scene',0.1)
    with pytest.raises(ValueError): validate_bundle(tmp_path,'other',0.1)
    with pytest.raises(ValueError): validate_bundle(tmp_path,'scene',0.2)
    (tmp_path/'static_map.nvblx').write_bytes(b'corrupted')
    with pytest.raises(ValueError): validate_bundle(tmp_path,'scene',0.1)


def test_snapshot_buffers_preserve_ros_wire_data_and_own_storage():
    from rclpy.serialization import serialize_message, deserialize_message
    from uav_nav_interfaces.msg import MapSnapshot
    from uav_nav_sim.core import pack_grid_data
    # Non-contiguous arrays must still use canonical x/y/z ordering. Include
    # unknown sentinels/nonfinite values and float64 input conversion.
    distance = np.arange(24, dtype=np.float64).reshape(2,3,4).transpose(2,0,1)
    distance[0,0,0] = np.nan
    distance[1,0,0] = np.inf
    distance[2,0,0] = -1000.
    observed = np.isfinite(distance) & (distance != -1000.)
    grid = Grid(np.zeros(3), .1, distance, observed)
    expected = MapSnapshot()
    expected.distance = grid.distance.ravel().tolist()
    expected.observed = grid.observed.astype(np.uint8).ravel().tolist()
    actual = MapSnapshot()
    actual.distance, actual.observed = pack_grid_data(grid)
    assert serialize_message(actual) == serialize_message(expected)
    restored = deserialize_message(serialize_message(actual), MapSnapshot)
    np.testing.assert_equal(restored.distance, grid.distance.ravel().astype(np.float32))
    assert list(restored.observed) == grid.observed.ravel().astype(np.uint8).tolist()
    actual.distance[3] = -123.
    assert grid.distance.ravel()[3] != -123.
