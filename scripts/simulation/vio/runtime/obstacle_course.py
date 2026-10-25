"""Deterministic four-zone course, shared geometry and bounded PX4 recipes."""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
LAYOUT = ROOT / 'simulation/scenes/obstacle_course.json'
WORLD = ROOT / 'src/uav_bringup/gazebo/worlds/uav_obstacle_course.sdf'
TASKS = ('none', 'pillars', 'rectangle', 'ring', 'trees', 'all')


def read_layout():
    return json.loads(LAYOUT.read_text())


def mission_path(task):
    if task not in TASKS[1:]:
        raise ValueError('Unsupported obstacle course task')
    return ROOT / f'simulation/missions/course_{task}.json'


def waypoints(layout, task):
    if task not in TASKS[1:]:
        raise ValueError('Unsupported obstacle course task')
    zones = layout['zones'] if task == 'all' else [z for z in layout['zones'] if z['id'] == task]
    points = [point for zone in zones for point in zone['waypoints_enu']]
    return points + (zones[0].get('return_waypoints_enu', []) if task != 'all' else [])


def recipe(layout, task):
    # Every offset is relative to task start, never to the preceding waypoint.
    return dict(schema=1, steps=[dict(type='TAKEOFF', height_m=2.5),
        dict(type='OBSERVE', timeout_s=190.0),
        *[dict(type='NAVIGATE', offset_enu=point) for point in waypoints(layout, task)],
        dict(type='RETURN'), dict(type='LAND')])


def obstacles(layout):
    """Primitive collisions; the ring is open and every tree part collides."""
    result = []
    def add(name, shape, center, size, rpy=(0., 0., 0.), color=(.35, .5, .65)):
        result.append(dict(name=name, shape=shape, center=list(center), size=list(size),
                           rpy=list(rpy), color=list(color)))
    half, height = layout['half_extent_m'], layout['height_m']
    for axis in (0, 1):
        for sign in (-1, 1):
            center = [0., 0., height/2]; center[axis] = sign*half
            size = [2*half+.2, 2*half+.2, height]; size[axis] = .2
            add(f'wall_{axis}_{sign}', 'box', center, size)
    add('ceiling', 'box', [0, 0, height+.1], [2*half, 2*half, .2])
    for zone in layout['zones']:
        if zone['id'] == 'pillars':
            for i, (x, y) in enumerate(zone['columns_xy']):
                add(f'column_{i:02}', 'cylinder', [x, y, 2.75], [.5, .5, 5.5], color=(.8, .4, .15))
        elif zone['id'] == 'rectangle':
            x, y, z = zone['center']; width, opening_height = zone['opening_m']; bar = zone['bar_m']
            for sign in (-1, 1):
                add(f'gate_post_{sign}', 'box', [x, y+sign*(width+bar)/2, z],
                    [bar, bar, opening_height+2*bar], color=(.2, .65, .85))
                add(f'gate_rail_{sign}', 'box', [x, y, z+sign*(opening_height+bar)/2],
                    [bar, width+2*bar, bar], color=(.2, .65, .85))
        elif zone['id'] == 'ring':
            x, y, z = zone['center']; count = zone['segments']; bar = zone['bar_m']
            radius = zone['inner_radius_m']+bar/2
            length = 2*radius*math.tan(math.pi/count)+.02
            for i in range(count):
                angle = 2*math.pi*i/count
                add(f'ring_segment_{i:02}', 'box', [x+radius*math.cos(angle), y, z+radius*math.sin(angle)],
                    [length, bar, bar], [0, -angle-math.pi/2, 0], color=(.85, .3, .2))
            for sign in (-1, 1):
                add(f'ring_support_{sign}', 'box', [x+sign*(radius+bar), y, z/2], [.18, .18, z])
        elif zone['id'] == 'trees':
            for i, (x, y) in enumerate(zone['trees_xy']):
                add(f'tree_{i}_trunk', 'cylinder', [x, y, 2.7], [.5, .5, 5.4], color=(.38, .22, .1))
                for j, yaw in enumerate((.3, 2.0, 4.2)):
                    add(f'tree_{i}_branch_{j}', 'cylinder', [x+.45*math.cos(yaw), y+.45*math.sin(yaw), 4.0],
                        [.16, .16, 1.5], [0, math.pi/2, yaw], color=(.38, .22, .1))
                for j, (dz, radius) in enumerate(((4.7, .85), (5.25, .65))):
                    add(f'tree_{i}_crown_{j}', 'sphere', [x, y, dz], [2*radius]*3, color=(.16, .44, .18))
    return result


def local_point(point, obstacle):
    """World point to primitive coordinates, using SDF Rz(yaw) Ry(pitch) Rx(roll)."""
    x, y, z = (a-b for a, b in zip(point, obstacle['center']))
    roll, pitch, yaw = obstacle['rpy']
    c, s = math.cos(yaw), math.sin(yaw); x, y = c*x+s*y, -s*x+c*y
    c, s = math.cos(pitch), math.sin(pitch); x, z = c*x-s*z, s*x+c*z
    c, s = math.cos(roll), math.sin(roll); y, z = c*y+s*z, -s*y+c*z
    return x, y, z


def clearance(point, obstacle):
    x, y, z = local_point(point, obstacle); size = obstacle['size']
    if obstacle['shape'] == 'sphere':
        return max(0., math.sqrt(x*x+y*y+z*z)-size[0]/2)
    if obstacle['shape'] == 'cylinder':
        return math.hypot(max(0., math.hypot(x, y)-size[0]/2), max(0., abs(z)-size[2]/2))
    return math.sqrt(sum(max(0., abs(v)-s/2)**2 for v, s in zip((x, y, z), size)))


def segment_points(a, b, spacing=.05):
    count = max(1, math.ceil(math.dist(a, b)/spacing))
    return [tuple(x+(y-x)*i/count for x, y in zip(a, b)) for i in range(count+1)]


def protected_clearance(layout):
    """Conservatively reject any collision AABB entering the launch/survey box."""
    lo, hi = layout['protected_min'], layout['protected_max']
    for obstacle in obstacles(layout):
        # Rotated bounding box via transformed corners, cylinders/spheres enclosed.
        extent = aabb_extent(obstacle)
        if not any(obstacle['center'][i]+extent[i] < lo[i] or obstacle['center'][i]-extent[i] > hi[i] for i in range(3)):
            raise ValueError('Obstacle in protected launch volume: '+obstacle['name'])
    return dict(passed=True, expanded_min=lo, expanded_max=hi,
                scope='collision AABBs outside protected launch/survey volume only')


def aabb_extent(obstacle):
    roll, pitch, yaw = obstacle['rpy']
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    rotation = ((cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr),
                (sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr), (-sp, cp*sr, cp*cr))
    return [sum(abs(row[j])*obstacle['size'][j]/2 for j in range(3)) for row in rotation]


def static_check(layout):
    protected = protected_clearance(layout)
    body = layout['body_radius_m']+layout['tracking_margin_m']
    solids = obstacles(layout); routes = {}
    reserve = body+layout['voxel_reserve_m']
    envelope = [reserve+layout['braking_margin_m']]*2+[reserve]
    boxes = [(o['center'], aabb_extent(o), o['name']) for o in solids]
    for task in TASKS[1:]:
        mission = recipe(layout, task)
        if len(mission['steps']) > 20:
            raise ValueError('Course exceeds the existing 20-step mission limit')
        points = [(0., 0., 2.5), *waypoints(layout, task), (0., 0., 2.5)]
        minimum = min(clearance(p, o) for a, b in zip(points, points[1:])
                      for p in segment_points(a, b) for o in solids)
        if minimum < body+.05:
            raise ValueError(f'{task} reference route lacks clearance: {minimum:.3f}')
        for a, b in zip(points, points[1:]):
            for point in segment_points(a, b):
                for center, extent, name in boxes:
                    if all(abs(point[i]-center[i]) <= extent[i]+envelope[i] for i in range(3)):
                        raise ValueError(f'{task} braking/voxel box intersects {name} at {point}')
        if any(abs(p[i]) > layout['half_extent_m']-2.0 for p in points for i in (0, 1)):
            raise ValueError('Waypoint outside body/tracking/braking inset')
        routes[task] = dict(steps=len(mission['steps']), min_clearance_m=minimum,
                            reference_length_m=sum(math.dist(a, b) for a, b in zip(points, points[1:])))
    return dict(passed=True, protected=protected, routes=routes,
                scope='sampled reference polylines only; not EGO, VIO or flight validation')


def validate_region(layout, config):
    static_check(layout)
    for key in ('body_radius_m', 'tracking_margin_m', 'braking_margin_m'):
        if config[key] != layout[key]: raise ValueError('Course region/margin mismatch: '+key)
    if (config['bounds_min'][:2] != [-layout['half_extent_m']]*2 or
            config['bounds_max'][:2] != [layout['half_extent_m']]*2 or
            config['known_clearance_bounds_min'][:2] != layout['protected_min'][:2] or
            config['known_clearance_bounds_max'][:2] != layout['protected_max'][:2] or
            config['known_clearance_bounds_max'][2] > layout['protected_max'][2]):
        raise ValueError('Course region/geometry mismatch')


def plan_svg(layout):
    """Repo-native plan view, using the same geometry and actual task order."""
    def point(p): return f'{360+18*p[0]:.1f},{360-18*p[1]:.1f}'
    items = ['<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 720 760">',
             '<rect width="720" height="760" fill="#f5f7fa"/>',
             '<g font-family="sans-serif" font-size="15" fill="#182638">',
             '<text x="40" y="32">Four-zone UAV course · 34 × 34 × 6 m · ENU</text>',
             '<rect x="54" y="54" width="612" height="612" fill="white" stroke="#182638"/>',
             '<rect x="288" y="288" width="144" height="144" fill="#e9f4ec" stroke="#5a9c71"/>',
             '<text x="305" y="354">Launch / survey</text>']
    for zone in layout['zones']:
        lo_x,lo_y,hi_x,hi_y = zone['bounds_xy']
        items.append(f'<rect x="{360+18*lo_x}" y="{360-18*hi_y}" width="{18*(hi_x-lo_x)}" height="{18*(hi_y-lo_y)}" fill="none" stroke="#c1cbd7" stroke-dasharray="5 5"/>')
        items.append(f'<text x="{366+18*lo_x}" y="{380-18*hi_y}">{zone["id"]}</text>')
        for x,y in zone.get('columns_xy',zone.get('trees_xy',[])):
            color = '#56854d' if zone['id']=='trees' else '#c47d36'
            items.append(f'<circle cx="{360+18*x}" cy="{360-18*y}" r="{15 if zone["id"]=="trees" else 6}" fill="{color}"/>')
        if zone['id'] == 'rectangle':
            x,y,_ = zone['center']; w = zone['opening_m'][0]
            items.append(f'<path d="M {360+18*x} {360-18*(y-w/2)} V {360-18*(y+w/2)}" stroke="#287fa7" stroke-width="6" stroke-dasharray="4 3"/>')
        elif zone['id'] == 'ring':
            x,y,_ = zone['center']; r = zone['inner_radius_m']
            items.append(f'<path d="M {360+18*(x-r)} {360-18*y} H {360+18*(x+r)}" stroke="#d46445" stroke-width="6" stroke-dasharray="4 3"/>')
    pts = [[0,0,2.5],*waypoints(layout,'all'),[0,0,2.5]]
    items.append('<polyline points="'+' '.join(point(p) for p in pts)+'" fill="none" stroke="#4c71ba" stroke-width="2"/>')
    for index,p in enumerate(pts[1:-1],1):
        x,y = map(float,point(p).split(','))
        items.append(f'<circle cx="{x}" cy="{y}" r="4" fill="#264f9c"/><text x="{x+6}" y="{y-6}" font-size="12">{index}</text>')
    items += ['<text x="40" y="706">Reference route only; live EGO paths can differ. Gate crossings are audited.</text>',
              '<text x="40" y="734">Rectangle opening 5 × 3 m · Ring inner diameter 5.2 m · Flight height 2.5 m</text>', '</g></svg>']
    return '\n'.join(items)+'\n'


def add_models(world, layout, *, textured=False):
    for obstacle in obstacles(layout):
        model = ET.SubElement(world, 'model', name=obstacle['name'])
        ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, obstacle['center']+obstacle['rpy']))
        link = ET.SubElement(model, 'link', name='structure')
        for kind in ('collision', 'visual'):
            item = ET.SubElement(link, kind, name=kind)
            geometry = ET.SubElement(item, 'geometry'); shape = ET.SubElement(geometry, obstacle['shape'])
            if obstacle['shape'] == 'box':
                ET.SubElement(shape, 'size').text = ' '.join(map(str, obstacle['size']))
            else:
                ET.SubElement(shape, 'radius').text = str(obstacle['size'][0]/2)
                if obstacle['shape'] == 'cylinder':
                    ET.SubElement(shape, 'length').text = str(obstacle['size'][2])
            if kind == 'visual':
                material = ET.SubElement(item, 'material')
                for tag in ('ambient', 'diffuse'):
                    ET.SubElement(material, tag).text = ' '.join(map(str, obstacle['color']))+' 1'
                if textured:
                    metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
                    ET.SubElement(metal, 'albedo_map').text = 'model://warehouse_textures/materials/textures/course.png'
                    ET.SubElement(metal, 'metalness').text = '0'
                    ET.SubElement(metal, 'roughness').text = '1'


def add_course(world, directory):
    """Inject only structures into the owned PX4 world; preserve its systems/rig."""
    from .vio_warehouse_scene import multiscale_texture
    layout = read_layout(); check = static_check(layout)
    textures = directory/'warehouse_textures/materials/textures'
    textures.mkdir(parents=True)
    for name in ('course', 'floor'):
        multiscale_texture(textures/(name+'.png'), random.Random(layout['seed']+(name=='floor')), width=1024)
    add_models(world, layout, textured=True)
    floor = ET.SubElement(world, 'model', name='course_textured_floor')
    ET.SubElement(floor, 'static').text = 'true'
    visual = ET.SubElement(ET.SubElement(floor, 'link', name='paint'), 'visual', name='floor')
    ET.SubElement(visual, 'pose').text = '0 0 .001 0 0 0'
    side = 2*layout['half_extent_m']
    ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = f'{side} {side} .002'
    material = ET.SubElement(visual, 'material'); metal = ET.SubElement(ET.SubElement(material, 'pbr'), 'metal')
    ET.SubElement(metal, 'albedo_map').text = 'model://warehouse_textures/materials/textures/floor.png'
    ET.SubElement(metal, 'metalness').text = '0'; ET.SubElement(metal, 'roughness').text = '1'
    receipt = dict(schema=1, layout=layout, layout_sha256=hashlib.sha256(LAYOUT.read_bytes()).hexdigest(),
                   clearance=check['protected'], reference_routes=check['routes'],
                   texture_sha256={str(p.relative_to(directory)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(textures.glob('*.png'))})
    # Same transport receipt name; scene_id identifies this separate geometry.
    (directory/'warehouse-layout.json').write_text(json.dumps(receipt, indent=2)+'\n')


def generated_world(layout):
    tree = ET.ElementTree(ET.Element('sdf', version='1.9'))
    world = ET.SubElement(tree.getroot(), 'world', name='uav_obstacle_course')
    physics = ET.SubElement(world, 'physics', name='fast_step', type='ignored')
    ET.SubElement(physics, 'max_step_size').text = '.002'
    ET.SubElement(physics, 'real_time_factor').text = '1.0'
    for system in ('Physics', 'UserCommands', 'SceneBroadcaster', 'Sensors', 'Imu'):
        slug = {'UserCommands':'user-commands', 'SceneBroadcaster':'scene-broadcaster'}.get(system, system.lower())
        plugin = ET.SubElement(world, 'plugin', filename=f'gz-sim-{slug}-system', name=f'gz::sim::systems::{system}')
        if system == 'Sensors': ET.SubElement(plugin, 'render_engine').text = 'ogre2'
    scene = ET.SubElement(world, 'scene'); ET.SubElement(scene, 'ambient').text = '.6 .6 .6 1'
    light = ET.SubElement(world, 'light', type='directional', name='sun')
    ET.SubElement(light, 'direction').text = '-.3 .2 -.9'; ET.SubElement(light, 'diffuse').text = '.9 .9 .9 1'
    ground = ET.SubElement(world, 'model', name='ground_plane'); ET.SubElement(ground, 'static').text = 'true'
    link = ET.SubElement(ground, 'link', name='ground')
    for kind in ('collision', 'visual'):
        plane = ET.SubElement(ET.SubElement(ET.SubElement(link, kind, name=kind), 'geometry'), 'plane')
        ET.SubElement(plane, 'normal').text = '0 0 1'; ET.SubElement(plane, 'size').text = f'{2*layout["half_extent_m"]+2} {2*layout["half_extent_m"]+2}'
    add_models(world, layout)
    for name, center, color in [('start_pad', [0,0,.01], '.1 .7 .2 1'),
                                ('goal_pad', [0,0,.015], '.1 .7 .2 1')]:
        # Render-only markings never form a raised collision in the landing area.
        model = ET.SubElement(world, 'model', name=name); ET.SubElement(model, 'static').text = 'true'
        ET.SubElement(model, 'pose').text = ' '.join(map(str, center))+' 0 0 0'
        visual = ET.SubElement(ET.SubElement(model, 'link', name='paint'), 'visual', name='paint')
        ET.SubElement(ET.SubElement(ET.SubElement(visual, 'geometry'), 'box'), 'size').text = '1.4 1.4 .002'
        ET.SubElement(ET.SubElement(visual, 'material'), 'diffuse').text = color
    include = ET.SubElement(world, 'include'); ET.SubElement(include, 'uri').text = 'model://uav_quad_mid360'
    ET.SubElement(include, 'name').text = 'uav_quad'; ET.SubElement(include, 'pose').text = '0 0 .11 0 0 0'
    ET.indent(tree)
    return ET.tostring(tree.getroot(), encoding='unicode', xml_declaration=True)+'\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--write', action='store_true', help='Regenerate the versioned world and five missions')
    args = parser.parse_args(); layout = read_layout(); check = static_check(layout)
    outputs = {WORLD:generated_world(layout),
               ROOT/'docs/simulation/images/obstacle-course.svg':plan_svg(layout),
               **{mission_path(task):json.dumps(recipe(layout, task), indent=2)+'\n' for task in TASKS[1:]}}
    for path, content in outputs.items():
        if args.write:
            path.parent.mkdir(parents=True, exist_ok=True); path.write_text(content)
        elif not path.is_file() or path.read_text() != content: raise ValueError('Regenerate course assets: '+str(path))
    print(json.dumps(check, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
