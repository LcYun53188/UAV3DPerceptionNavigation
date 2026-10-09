"""Deterministic warehouse geometry and independent free-volume checks."""
import hashlib
import json
import math
from pathlib import Path
import random
import struct
import zlib
import xml.etree.ElementTree as ET

LAYOUT = Path(__file__).resolve().parents[1]/'simulation/scenes/vio_warehouse.json'


def box_clearance(layout):
    """Reject intersecting AABBs including flight body, tracking and braking budget."""
    if layout['schema'] != 1 or layout['scene_id'] != 'vio_warehouse_v1':
        raise ValueError('Unsupported warehouse layout')
    lower, upper = layout['flight_box_min'], layout['flight_box_max']
    if (len(lower) != 3 or len(upper) != 3 or
            not all(math.isfinite(a) and math.isfinite(b) and a < b for a,b in zip(lower,upper))):
        raise ValueError('Invalid flight volume')
    margins = [layout[k] for k in ('body_radius_m','tracking_margin_m','braking_margin_m','ceiling_margin_m')]
    if not all(math.isfinite(x) and x >= 0 for x in margins):
        raise ValueError('Invalid flight margins')
    horizontal = sum(margins[:3])
    lo = [lower[0]-horizontal, lower[1]-horizontal, layout['support_plane_world_z']]
    hi = [upper[0]+horizontal, upper[1]+horizontal, upper[2]+margins[3]]
    checks = {}
    for obstacle in layout['obstacles']:
        center, size = obstacle['center'], obstacle['size']
        if (len(center) != 3 or len(size) != 3 or not all(math.isfinite(x) for x in center)
                or not all(math.isfinite(x) and x > 0 for x in size)):
            raise ValueError('Invalid obstacle geometry')
        # Touching counts as collision; no rotation is allowed in this schema.
        checks[obstacle['name']] = any(center[i]+size[i]/2 < lo[i] or center[i]-size[i]/2 > hi[i] for i in range(3))
        if not checks[obstacle['name']]:
            raise ValueError('Obstacle inside inflated flight volume: '+obstacle['name'])
    if len(checks) != len(layout['obstacles']):raise ValueError('Duplicate obstacle name')
    return dict(passed=True, expanded_min=lo, expanded_max=hi, obstacle_checks=checks,
                scope='static axis-aligned geometry only; not sensor clearance or flight authorization')


def box(link, name, center, size, color, collision=False):
    obj = ET.SubElement(link,'collision' if collision else 'visual',name=name)
    ET.SubElement(obj,'pose').text = ' '.join(map(str,center))+' 0 0 0'
    ET.SubElement(ET.SubElement(ET.SubElement(obj,'geometry'),'box'),'size').text = ' '.join(map(str,size))
    if not collision:
        mat = ET.SubElement(obj,'material')
        for kind in ('ambient','diffuse'):ET.SubElement(mat,kind).text = ' '.join(map(str,color))+' 1'
    return obj


def corner_texture(path, rng):
    """Create a reproducible RGB PNG; render-only features, never pose inputs."""
    width=512
    pixels=bytearray([220]*(width*width*3))
    def rectangle(x,y,w,h,color):
        for row in range(y,min(width,y+h)):
            start=(row*width+x)*3
            pixels[start:start+w*3]=bytes(color)*w
    for row in range(16):
        for col in range(16):
            x,y=col*32,row*32
            base=rng.choice((35,75,155,230))
            rectangle(x,y,32,32,[base]*3)
            # Irregular high contrast corners at two scales, rather than noise.
            for scale in (18,8):
                cx,cy=x+rng.randint(2,12),y+rng.randint(2,12)
                color=[rng.choice((10,245))]*3
                rectangle(cx,cy,scale,3,color)
                rectangle(cx,cy,3,scale,color)
    raw=b''.join(b'\x00'+pixels[row*width*3:(row+1)*width*3] for row in range(width))
    def chunk(kind,data):
        return struct.pack('>I',len(data))+kind+data+struct.pack('>I',zlib.crc32(kind+data)&0xffffffff)
    path.write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('>IIBBBBB',width,width,8,2,0,0,0))+
                    chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))


def add_warehouse(world, directory):
    layout = json.loads(LAYOUT.read_text())
    clearance = box_clearance(layout)
    rng = random.Random(layout['seed'])
    textures=directory/'warehouse_textures/materials/textures'
    textures.mkdir(parents=True)
    texture_hashes={}
    for obstacle in layout['obstacles']:
        model = ET.SubElement(world,'model',name=obstacle['name'])
        ET.SubElement(model,'static').text = 'true'
        ET.SubElement(model,'pose').text = ' '.join(map(str,obstacle['center']))+' 0 0 0'
        link = ET.SubElement(model,'link',name='structure')
        size = obstacle['size']
        box(link,'collision',[0,0,0],size,[.5,.5,.5],True)
        surface=box(link,'surface',[0,0,0],size,[1,1,1])
        texture=textures/(obstacle['name']+'.png')
        corner_texture(texture,random.Random(layout['seed']+len(texture_hashes)))
        texture_hashes[str(texture.relative_to(directory))]=hashlib.sha256(texture.read_bytes()).hexdigest()
        metal=ET.SubElement(ET.SubElement(surface.find('material'),'pbr'),'metal')
        ET.SubElement(metal,'albedo_map').text='model://warehouse_textures/materials/textures/'+texture.name
        ET.SubElement(metal,'metalness').text='0'
        ET.SubElement(metal,'roughness').text='1'
        # Rendered irregular patches on four faces give genuine image features.
        # Collisions remain simple enclosing boxes; no detached floating objects.
        for face in range(4):
            axis,sign = face//2,1 if face%2 else -1
            tangent = 1-axis
            for j in range(12):
                center = [0.,0.,rng.uniform(-size[2]*.45,size[2]*.45)]
                center[axis] = sign*(size[axis]/2+.002)
                center[tangent] = rng.uniform(-size[tangent]*.42,size[tangent]*.42)
                patch_size = [.004,.004,min(.3,size[2]*.15)]
                patch_size[tangent] = min(rng.uniform(.1,.4),size[tangent]*.35)
                box(link,f'feature_{face}_{j}',center,patch_size,[rng.choice((.04,.15,.7,.95)) for _ in range(3)])
        if obstacle['name'].startswith('rack'):
            for level in range(3):
                box(link,f'shelf_marker_{level}',[0,0,(level-1)*.8],[size[0]+.005,size[1]+.005,.05],[.8,.6,.15])
    floor = ET.SubElement(world,'model',name='warehouse_floor_features')
    ET.SubElement(floor,'static').text = 'true'
    link = ET.SubElement(floor,'link',name='paint')
    for i in range(layout['visual_floor_patches']):
        box(link,f'paint_{i}',[rng.uniform(-6.7,6.7),rng.uniform(-6.7,6.7),.002],
            [rng.uniform(.1,.35),rng.uniform(.1,.35),.002],[rng.choice((.03,.2,.7,.95)) for _ in range(3)])
    receipt = dict(schema=1, layout=layout, layout_sha256=hashlib.sha256(LAYOUT.read_bytes()).hexdigest(),clearance=clearance,texture_sha256=texture_hashes)
    (directory/'warehouse-layout.json').write_text(json.dumps(receipt,indent=2)+'\n')
