"""Generate an explicit stereo/IMU reference rig and textured SITL test world."""
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
PROFILE = ROOT/'simulation/px4/vio/sensors.json'


def assets(directory, upstream_world):
    p = json.loads(PROFILE.read_text())
    if p['schema'] != 1 or not 0 < p['baseline_m'] < 1:
        raise ValueError('Invalid reference sensor profile')
    directory.mkdir(parents=True,exist_ok=True)
    model_dir = directory/p['model']
    model_dir.mkdir()
    sdf = ET.Element('sdf',version='1.9')
    model = ET.SubElement(sdf,'model',name=p['model'])
    include = ET.SubElement(model,'include',merge='true')
    ET.SubElement(include,'uri').text = 'model://x500'
    rig = ET.SubElement(model,'link',name='vio_rig_link')
    ET.SubElement(rig,'pose').text = ' '.join(map(str,p['rig_position_flu_m']))+' 0 0 0'
    inertial = ET.SubElement(rig,'inertial')
    ET.SubElement(inertial,'mass').text = '0.001'
    inertia = ET.SubElement(inertial,'inertia')
    for axis in ('ixx','iyy','izz'): ET.SubElement(inertia,axis).text = '0.000001'
    frames = {}
    for side,sign in (('left',1),('right',-1)):
        frame = 'vio_'+side+'_optical'
        center = [p['rig_position_flu_m'][0],p['rig_position_flu_m'][1]+sign*p['baseline_m']/2,p['rig_position_flu_m'][2]]
        frames[frame] = dict(position=center,rpy=[-math.pi/2,0.,-math.pi/2])
        sensor = ET.SubElement(rig,'sensor',name=side,type='camera')
        ET.SubElement(sensor,'pose').text = f"0 {sign*p['baseline_m']/2} 0 0 0 0"
        ET.SubElement(sensor,'always_on').text = 'true'
        ET.SubElement(sensor,'update_rate').text = str(p['image_rate_hz'])
        ET.SubElement(sensor,'topic').text = '/vio/'+side+'/image'
        ET.SubElement(sensor,'gz_frame_id').text = frame
        camera = ET.SubElement(sensor,'camera')
        ET.SubElement(camera,'horizontal_fov').text = str(p['horizontal_fov_rad'])
        image = ET.SubElement(camera,'image')
        for key in ('width','height'): ET.SubElement(image,key).text = str(p[key])
        ET.SubElement(image,'format').text = 'R8G8B8'
        clip = ET.SubElement(camera,'clip')
        ET.SubElement(clip,'near').text = str(p['near_m'])
        ET.SubElement(clip,'far').text = str(p['far_m'])
        ET.SubElement(camera,'camera_info_topic').text = '/vio/'+side+'/camera_info'
        ET.SubElement(camera,'optical_frame_id').text = frame
    imu = ET.SubElement(rig,'sensor',name='vio_imu',type='imu')
    for tag,value in [('always_on','true'),('update_rate',str(p['imu_rate_hz'])),('topic','/vio/imu'),('gz_frame_id','vio_imu')]:
        ET.SubElement(imu,tag).text = value
    frames['vio_imu'] = dict(position=p['rig_position_flu_m'],rpy=[0.,0.,0.])
    joint = ET.SubElement(model,'joint',name='vio_rig_joint',type='fixed')
    ET.SubElement(joint,'parent').text = 'base_link'
    ET.SubElement(joint,'child').text = 'vio_rig_link'
    ET.indent(sdf)
    ET.ElementTree(sdf).write(model_dir/'model.sdf',encoding='utf-8',xml_declaration=True)
    (model_dir/'model.config').write_text('<model><name>'+p['model']+'</name><version>1</version><sdf version="1.9">model.sdf</sdf></model>')
    world = ET.parse(upstream_world)
    root = world.getroot().find('world')
    # Seeded geometry gives real stereo parallax and image features, never odometry.
    rng = random.Random(68078)
    for wall in range(3):
        m = ET.SubElement(root,'model',name=f'vio_wall_{wall}')
        ET.SubElement(m,'static').text = 'true'
        ET.SubElement(m,'pose').text = ['4 0 2.5 0 0 0','0 4 2.5 0 0 1.57079632679','0 -4 2.5 0 0 -1.57079632679'][wall]
        link = ET.SubElement(m,'link',name='wall')
        collision = ET.SubElement(link,'collision',name='surface')
        box = ET.SubElement(ET.SubElement(collision,'geometry'),'box')
        ET.SubElement(box,'size').text = '0.04 9 5'
        for row in range(14):
            for col in range(24):
                visual = ET.SubElement(link,'visual',name=f'tile_{row}_{col}')
                ET.SubElement(visual,'pose').text = f"0 {(col-11.5)*.375} {(row-6.5)*.357} 0 0 0"
                box = ET.SubElement(ET.SubElement(visual,'geometry'),'box')
                ET.SubElement(box,'size').text = '0.04 0.375 0.357'
                material = ET.SubElement(visual,'material')
                color = ' '.join(str(rng.choice((.03,.15,.35,.65,.95))) for _ in range(3))+' 1'
                for tag in ('ambient','diffuse'): ET.SubElement(material,tag).text = color
    world.write(directory/'default.sdf',encoding='utf-8',xml_declaration=True)
    (directory/'frames.json').write_text(json.dumps(frames,indent=2)+'\n')
    return p,frames
