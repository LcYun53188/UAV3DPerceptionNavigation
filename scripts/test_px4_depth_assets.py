"""The generated mapping camera and advertised optical TF must agree."""
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from prepare_px4_depth_assets import depth_assets
from px4_depth_audit import DepthAudit
from types import SimpleNamespace as NS


def test_same_body_fixed_pitch_optical_axes_and_reproducible_assets(tmp_path):
    world = tmp_path / 'world.sdf'
    original = b'<sdf version="1.9"><world name="default"/></sdf>'
    world.write_bytes(original)
    first, second = tmp_path / 'first', tmp_path / 'second'
    profile, frames = depth_assets(first, world)
    depth_assets(second, world)
    assert world.read_bytes() == original
    for file in first.rglob('*'):
        if file.is_file():
            assert file.read_bytes() == (second / file.relative_to(first)).read_bytes()
    model = ET.parse(first / profile['model'] / 'model.sdf').find('model')
    assert model.findtext('include/uri') == 'model://x500'
    assert model.find("link[@name='vio_rig_link']/pose").get('relative_to')=='base_link'
    sensors = model.findall('.//sensor')
    assert len(sensors) == 1 and sensors[0].get('type') == 'rgbd_camera'
    assert float(sensors[0].findtext('pose').split()[4]) == math.radians(5)
    frame = frames[sensors[0].findtext('gz_frame_id')]
    roll, _, yaw = frame['rpy']
    rx = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    # Positive optical Z points forward and down in body FLU.
    assert np.allclose((rz @ rx)[:, 2], [math.cos(math.radians(5)), 0, -math.sin(math.radians(5))])
    assert frame['position'] == profile['rig_position_flu_m']


def test_reference_calibration_accepted_legacy_intrinsics_rejected(tmp_path):
    world = tmp_path / 'world.sdf'
    world.write_text('<sdf version="1.9"><world name="default"/></sdf>')
    profile, _ = depth_assets(tmp_path / 'assets', world)
    audit = DepthAudit(profile)
    width, height = profile['width'], profile['height']
    focal = width / (2 * math.tan(profile['horizontal_fov_rad'] / 2))
    for stamp in range(1, 6):
        header = NS(stamp=NS(sec=stamp, nanosec=0), frame_id='oakd_camera_optical_frame')
        audit.image(NS(header=header, width=width, height=height, encoding='32FC1',
                       step=width*4, is_bigendian=False,
                       data=np.full((height, width), 2., dtype='<f4').tobytes()), 10.)
        info = NS(header=header, width=width, height=height,
                  k=[focal, 0., width/2, 0., focal, height/2, 0., 0., 1.],
                  p=[focal, 0., width/2, 0., 0., focal, height/2, 0., 0., 0., 1., 0.],
                  r=[1., 0., 0., 0., 1., 0., 0., 0., 1.], d=[])
        audit.info(info, 10.)
    writers = {'/px4_depth/image': 1, '/px4_depth/camera_info': 1}
    assert audit.result(10.1, 5.1, writers)['passed']
    info.k[0] = 640 / (2 * math.tan(1.274 / 2))
    audit.info(info, 10.)
    assert not audit.result(10.1, 5.1, writers)['passed']
