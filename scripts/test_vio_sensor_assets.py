"""Check the calibration geometry that the reference stereo/IMU data relies on."""
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parent))
from prepare_vio_sensor_assets import assets


def test_stereo_triangulation_and_imu_mount(tmp_path):
    world = tmp_path/'world.sdf'
    world.write_text('<sdf version="1.9"><world name="default"/></sdf>')
    profile,frames = assets(tmp_path/'generated',world)
    # Optical axes: right/down/forward expressed in the FLU body.
    rotation = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    point = np.asarray(profile['rig_position_flu_m'])+[3.,.2,-.1]
    focal = profile['width']/(2*math.tan(profile['horizontal_fov_rad']/2))
    pixels = []
    for side in ('left','right'):
        frame = frames['vio_'+side+'_optical']
        roll,pitch,yaw = frame['rpy']
        rx = np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
        rz = np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        assert pitch == 0 and np.allclose(rz@rx,rotation)
        p = rotation.T@(point-np.asarray(frame['position']))
        pixels.append(focal*p[:2]/p[2]+[profile['width']/2,profile['height']/2])
    disparity = pixels[0][0]-pixels[1][0]
    assert disparity > 0 and abs(focal*profile['baseline_m']/disparity-3.) < 1e-10
    assert abs(pixels[0][1]-pixels[1][1]) < 1e-10
    assert frames['vio_imu']['position'] == profile['rig_position_flu_m']
    assert frames['vio_imu']['rpy'] == [0,0,0]
    model = ET.parse(tmp_path/'generated'/profile['model']/'model.sdf')
    cameras = model.findall('.//sensor[@type="camera"]')
    origins = [float(c.findtext('pose').split()[1]) for c in cameras]
    assert abs(origins[0]-origins[1]-profile['baseline_m']) < 1e-12
    # CameraInfo P does not carry a second baseline: the algorithm uses TF.
    assert [c.findtext('gz_frame_id') for c in cameras] == ['vio_left_optical','vio_right_optical']


def test_assets_are_reproducible_without_mutating_upstream(tmp_path):
    world = tmp_path/'world.sdf'
    original = b'<sdf version="1.9"><world name="default"/></sdf>'
    world.write_bytes(original)
    assets(tmp_path/'a',world)
    assets(tmp_path/'b',world)
    assert world.read_bytes() == original
    for relative in ('default.sdf','frames.json','x500_vio_ref/model.sdf'):
        assert (tmp_path/'a'/relative).read_bytes() == (tmp_path/'b'/relative).read_bytes()
