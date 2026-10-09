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
    # SDK FillIntrinsics reads K; FillExtrinsics uses TF, independently of CameraInfo P.
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


def test_motion_fixture_has_physics_and_separate_truth(tmp_path):
    world = tmp_path/'world.sdf'
    world.write_text('<sdf version="1.9"><world name="default"/></sdf>')
    profile,_ = assets(tmp_path/'motion',world,motion_plugin=Path('/fixture.so'),scene='layered')
    model = ET.parse(tmp_path/'motion'/profile['model']/'model.sdf')
    assert not model.findall('.//include')  # No PX4 motors or flight model on fixture.
    assert model.findtext('.//link[@name="base_link"]/inertial/mass') == '1'
    assert model.find('.//link[@name="base_link"]/collision') is not None
    assert model.find('.//plugin[@name="uav::test::MotionCarrier"]') is not None
    assert model.findtext('.//plugin/odom_topic') == '/vio/truth'
    generated = ET.parse(tmp_path/'motion/default.sdf')
    assert generated.findtext('.//include/pose') == '0 0 1.3 0 0 0'
    landmarks = generated.findall('.//model/link[@name="landmark"]')
    assert len(landmarks)==90 and all(m.find('collision') is not None for m in landmarks)


def test_layered_same_aircraft_rig_keeps_px4_model_and_has_no_motion_plugin(tmp_path):
    world=tmp_path/'world.sdf'
    world.write_text('<sdf version="1.9"><world name="default"/></sdf>')
    profile,_=assets(tmp_path/'layered',world,scene='layered')
    model=ET.parse(tmp_path/'layered'/profile['model']/'model.sdf')
    assert profile['model']=='x500_vio_ref' and profile['scene']=='layered'
    assert model.findtext('.//include/uri')=='model://x500'
    assert model.find('.//plugin[@name="uav::test::MotionCarrier"]') is None
    assert model.find('.//plugin/odom_topic') is None
    assert len(ET.parse(tmp_path/'layered/default.sdf').findall('.//model/link[@name="landmark"]'))==90


def test_explicit_pacing_changes_only_generated_world_and_preserves_simulation_sampling(tmp_path):
    world=tmp_path/'world.sdf'
    world.write_text('<sdf version="1.9"><world name="default"><physics><max_step_size>0.004</max_step_size><real_time_factor>1</real_time_factor><real_time_update_rate>250</real_time_update_rate></physics></world></sdf>')
    p,_=assets(tmp_path/'paced',world,real_time_factor=.8)
    physics=ET.parse(tmp_path/'paced/default.sdf').find('.//physics')
    assert float(physics.findtext('max_step_size'))==.004
    assert float(physics.findtext('real_time_factor'))==.8
    assert float(physics.findtext('real_time_update_rate'))==200
    assert p['image_rate_hz']==25 and p['imu_rate_hz']==250
    assert ET.parse(world).findtext('.//real_time_factor')=='1'


def test_reduced_resolution_regenerates_camera_geometry(tmp_path):
    world=tmp_path/'world.sdf';world.write_text('<sdf version="1.9"><world name="default"/></sdf>')
    profile,_=assets(tmp_path/'assets',world,resolution=(480,300))
    model=ET.parse(tmp_path/'assets'/profile['model']/'model.sdf')
    assert profile['width']==480 and profile['height']==300 and profile['image_rate_hz']==25
    for image in model.findall('.//camera/image'):
        assert image.findtext('width')=='480' and image.findtext('height')=='300'


def test_downward_pitch_keeps_optical_tf_consistent_with_rendered_sensor(tmp_path):
    world=tmp_path/'world.sdf';world.write_text('<sdf version="1.9"><world name="default"/></sdf>')
    profile,frames=assets(tmp_path/'pitched',world,camera_pitch_deg=15)
    theta=math.radians(15)
    ry=np.array([[math.cos(theta),0,math.sin(theta)],[0,1,0],[-math.sin(theta),0,math.cos(theta)]])
    original=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
    for side in ('left','right'):
        roll,pitch,yaw=frames['vio_'+side+'_optical']['rpy']
        rx=np.array([[1,0,0],[0,math.cos(roll),-math.sin(roll)],[0,math.sin(roll),math.cos(roll)]])
        rz=np.array([[math.cos(yaw),-math.sin(yaw),0],[math.sin(yaw),math.cos(yaw),0],[0,0,1]])
        assert pitch==0 and np.allclose(rz@rx,ry@original)
    assert frames['vio_imu']['rpy']==[0,0,0]
    assert profile['camera_pitch_deg']==15
    model=ET.parse(tmp_path/'pitched'/profile['model']/'model.sdf')
    for camera in model.findall('.//sensor[@type="camera"]'):
        assert float(camera.findtext('pose').split()[4])==theta
    # Optical forward ray now points below the horizon; identical stereo baseline.
    assert (ry@original)[2,2]<0 and profile['baseline_m']==.075
