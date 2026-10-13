"""MTF-01 reference geometry: rendered down-facing camera and GPU single ray."""
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT / 'simulation/px4/mtf01/sensors.json'


def add_sensor_links(model, p):
    """Attach the same fixed down-facing sensors to one aircraft body."""
    for name in ('flow_link', 'lidar_sensor_link'):
        link = ET.SubElement(model, 'link', name=name)
        ET.SubElement(link, 'pose', relative_to='base_link').text = ' '.join(map(str, p['position_flu_m'])) + ' 0 0 0'
        inertial = ET.SubElement(link, 'inertial')
        ET.SubElement(inertial, 'mass').text = '0.0045' if name == 'flow_link' else '0.000001'
        inertia = ET.SubElement(inertial, 'inertia')
        for axis in ('ixx', 'iyy', 'izz'):
            ET.SubElement(inertia, axis).text = '0.000001'
        joint = ET.SubElement(model, 'joint', name=name+'_joint', type='fixed')
        ET.SubElement(joint, 'parent').text = 'base_link'
        ET.SubElement(joint, 'child').text = name
        if name == 'flow_link':
            visual = ET.SubElement(link, 'visual', name='mtf01_housing')
            ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = '0.0293 0.017 0.0145'
            camera = ET.SubElement(link, 'sensor', name='flow_camera', type='camera')
            ET.SubElement(camera, 'pose').text = f'0 0 0 0 {math.pi/2} 0'
            ET.SubElement(camera, 'always_on').text = 'true'
            ET.SubElement(camera, 'topic').text = '/mtf01/flow/image'
            ET.SubElement(camera, 'update_rate').text = str(p['output_rate_hz'])
            optics = ET.SubElement(camera, 'camera')
            ET.SubElement(optics, 'horizontal_fov').text = str(math.radians(p['flow_fov_deg']))
            image = ET.SubElement(optics, 'image')
            for tag in ('width', 'height'):
                ET.SubElement(image, tag).text = '100'
            # RGB8 is supported by both Gazebo and the owned LK adapter.
            ET.SubElement(image, 'format').text = 'R8G8B8'
            clip = ET.SubElement(optics, 'clip')
            ET.SubElement(clip, 'near').text = '0.01'
            ET.SubElement(clip, 'far').text = str(p['range_max_m'])
        else:
            lidar = ET.SubElement(link, 'sensor', name='lidar', type='gpu_lidar')
            # Own adapter preserves sensor-relative mounting semantics. Native
            # GZBridge compares world orientation with a different basis.
            ET.SubElement(lidar, 'topic').text = '/mtf01/range/scan'
            # Same scan orientation as official x500_flow, exact angles.
            ET.SubElement(lidar, 'pose').text = f'0 0 0 {math.pi} {math.pi/2} 0'
            ET.SubElement(lidar, 'always_on').text = 'true'
            ET.SubElement(lidar, 'update_rate').text = str(p['output_rate_hz'])
            ray = ET.SubElement(lidar, 'ray')
            scan = ET.SubElement(ray, 'scan')
            for axis in ('horizontal', 'vertical'):
                section = ET.SubElement(scan, axis)
                for tag, value in (('samples','1'), ('resolution','1'), ('min_angle','0'), ('max_angle','0')):
                    ET.SubElement(section, tag).text = value
            range_element = ET.SubElement(ray, 'range')
            for tag, value in (('min',p['range_min_m']), ('max',p['range_max_m']), ('resolution',0.01)):
                ET.SubElement(range_element, tag).text = str(value)


def add_near_field_target(root, directory):
    from vio_warehouse_scene import box, corner_texture
    import random
    # Printed target is render-only, with no collision or pose input.
    corner_texture(directory/'warehouse_textures/materials/textures/near_field.png',random.Random(68079))
    target = ET.SubElement(root,'model',name='mtf01_near_field_target')
    ET.SubElement(target,'static').text='true'
    obj = box(ET.SubElement(target,'link',name='paint'),'target',[0,0,.003],[.5,.5,.001],[1,1,1])
    metal = ET.SubElement(ET.SubElement(obj.find('material'),'pbr'),'metal')
    ET.SubElement(metal,'albedo_map').text='model://warehouse_textures/materials/textures/near_field.png'
    ET.SubElement(metal,'metalness').text='0'
    ET.SubElement(metal,'roughness').text='1'


def assets(directory, upstream_world, *, textured=True):
    p = json.loads(PROFILE.read_text())
    directory.mkdir(parents=True)
    model_dir = directory / p['model']
    model_dir.mkdir()
    sdf = ET.Element('sdf', version='1.9')
    model = ET.SubElement(sdf, 'model', name=p['model'])
    ET.SubElement(ET.SubElement(model, 'include', merge='true'), 'uri').text = 'model://x500'
    add_sensor_links(model,p)
    ET.indent(sdf)
    ET.ElementTree(sdf).write(model_dir/'model.sdf', encoding='utf-8', xml_declaration=True)
    (model_dir/'model.config').write_text(f'<model><name>{p["model"]}</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
    world = ET.parse(upstream_world)
    from vio_warehouse_scene import add_warehouse
    root = world.getroot().find('world')
    add_warehouse(root, directory, floor_texture=textured)
    if textured:
        # Printed near-field target makes ground-start flow observable; regular
        # warehouse features remain at normal flight heights. Render-only.
        add_near_field_target(root,directory)
    else:
        # Ensure the comparison floor is featureless, including random patches.
        root.remove(root.find("model[@name='warehouse_floor_features']"))
    world.write(directory/'default.sdf', encoding='utf-8', xml_declaration=True)
    return p


def attach_to_vio_assets(directory, vio_profile):
    """Augment already generated same-aircraft VIO assets; preserve stereo rig."""
    if vio_profile.get('scene')!='warehouse' or not vio_profile.get('warehouse_floor_texture'):
        raise ValueError('Joint reference requires a textured warehouse')
    p=json.loads(PROFILE.read_text())
    model_dir=directory/vio_profile['model']
    sdf=ET.parse(model_dir/'model.sdf')
    add_sensor_links(sdf.getroot().find('model'),p)
    ET.indent(sdf)
    sdf.write(model_dir/'model.sdf',encoding='utf-8',xml_declaration=True)
    world=ET.parse(directory/'default.sdf')
    add_near_field_target(world.getroot().find('world'),directory)
    world.write(directory/'default.sdf',encoding='utf-8',xml_declaration=True)
    vio_profile['mtf01']=p
    return p
