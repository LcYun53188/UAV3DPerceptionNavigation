"""Same-aircraft ideal RGBD reference; fixed 5 degrees down, not OAK-D calibration."""
from copy import deepcopy
import json
import math
import xml.etree.ElementTree as ET
from prepare_vio_sensor_assets import assets


def depth_assets(directory, upstream_world):
    # Reuse the deterministic reference scene and mount, but render only RGBD.
    profile, _ = assets(directory, upstream_world, camera_pitch_deg=5)
    old = directory / profile['model']
    model_dir = directory / 'x500_depth_ref'
    old.rename(model_dir)
    tree = ET.parse(model_dir / 'model.sdf')
    model = tree.getroot().find('model')
    model.set('name', 'x500_depth_ref')
    rig = model.find("link[@name='vio_rig_link']")
    # The merged X500 base is elevated in model coordinates. Mount in body FLU.
    rig.find('pose').set('relative_to','base_link')
    for sensor in list(rig.findall('sensor')):
        rig.remove(sensor)
    frame = 'oakd_camera_optical_frame'
    sensor = ET.SubElement(rig, 'sensor', name='mapping_rgbd', type='rgbd_camera')
    for key, value in [('pose', f'0 0 0 0 {math.radians(5)} 0'),
                       ('always_on', 'true'), ('update_rate', '15'),
                       ('topic', '/px4_reference/rgbd'), ('gz_frame_id', frame)]:
        ET.SubElement(sensor, key).text = value
    camera = ET.SubElement(sensor, 'camera')
    ET.SubElement(camera, 'horizontal_fov').text = str(profile['horizontal_fov_rad'])
    image = ET.SubElement(camera, 'image')
    for key in ('width', 'height'):
        ET.SubElement(image, key).text = str(profile[key])
    ET.SubElement(image, 'format').text = 'R8G8B8'
    clip = ET.SubElement(camera, 'clip')
    for key, value in [('near', profile['near_m']), ('far', profile['far_m'])]:
        ET.SubElement(clip, key).text = str(value)
    ET.SubElement(camera, 'camera_info_topic').text = '/px4_reference/rgbd/camera_info'
    ET.SubElement(camera, 'optical_frame_id').text = frame
    ET.indent(tree)
    tree.write(model_dir / 'model.sdf', encoding='utf-8', xml_declaration=True)
    (model_dir / 'model.config').write_text('<model><name>x500_depth_ref</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
    frames = {frame: dict(position=profile['rig_position_flu_m'],
                         rpy=[-math.pi/2-math.radians(5), 0., -math.pi/2])}
    (directory / 'frames.json').write_text(json.dumps(frames, indent=2)+'\n')
    # Enclose the west-facing depth rays with actual rendered/collidable geometry.
    world=ET.parse(directory/'default.sdf');scene=world.getroot().find('world')
    west=deepcopy(scene.find("model[@name='vio_wall_0']"))
    west.set('name','depth_wall_west');west.find('pose').text='-4 0 2.5 0 0 0'
    scene.append(west)
    world.write(directory/'default.sdf',encoding='utf-8',xml_declaration=True)
    profile.update(model='x500_depth_ref', image_rate_hz=15,
                   scope='ideal same-X500 RGBD reference; no stereo/VIO or hardware calibration',
                   topics=dict(depth='/px4_reference/rgbd/depth_image',
                               color='/px4_reference/rgbd/image',
                               camera_info='/px4_reference/rgbd/camera_info'))
    (directory / 'depth-profile.json').write_text(json.dumps(profile, indent=2)+'\n')
    return profile, frames
