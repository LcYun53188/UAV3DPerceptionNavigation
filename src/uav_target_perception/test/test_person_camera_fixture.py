"""The optional RGB-D camera must not join Gazebo's VIO stereo group."""
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from support.paths import enable_imports
enable_imports()
from prepare_vio_sensor_assets import assets
from vio_chain_mapping import attach_depth


def test_independent_rgbd_preserves_vio_sensors_and_mount(tmp_path):
    profile, frames = assets(tmp_path/'assets',
        ROOT/'.deps/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf', camera_pitch_deg=5)
    path = tmp_path/'assets'/profile['model']/'model.sdf'
    before = ET.parse(path).getroot().find("model/link[@name='vio_rig_link']")
    originals = {item.get('name'): ET.tostring(item).strip() for item in before.findall('sensor')}
    mount, inertial = ET.tostring(before.find('pose')).strip(), ET.tostring(before.find('inertial')).strip()
    result = attach_depth(tmp_path/'assets', profile, frames, independent_rgbd=True)
    model = ET.parse(path).getroot().find('model')
    after = model.find("link[@name='vio_rig_link']")
    assert ET.tostring(after.find('pose')).strip() == mount
    assert ET.tostring(after.find('inertial')).strip() == inertial
    assert {item.get('name'): ET.tostring(item).strip() for item in after.findall('sensor')} == originals
    camera = model.find("link[@name='chain_rgbd_link']")
    assert camera.find('pose').get('relative_to') == 'vio_rig_link'
    assert camera.find('pose').text == '0 0 0 0 0 0'
    assert camera.find("sensor[@name='chain_rgbd']").get('type') == 'rgbd_camera'
    joint = model.find("joint[@name='chain_rgbd_joint']")
    assert joint.get('type') == 'fixed'
    assert joint.find('parent').text == 'vio_rig_link' and joint.find('child').text == 'chain_rgbd_link'
    assert 0 < float(camera.find('inertial/mass').text) <= 1e-6
    assert result['independent_rgbd_link'] and result['width'] == 320 and result['height'] == 240
