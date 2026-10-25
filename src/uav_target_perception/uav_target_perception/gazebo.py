"""Pinned OpenRobotics CC0 person assets and opt-in static Gazebo postures."""
import argparse
import copy
import hashlib
import json
import math
from pathlib import Path
import shutil
import tempfile
import urllib.request
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

MODEL_URL = 'https://fuel.gazebosim.org/1.0/OpenRobotics/models/Standing%20person'
ARCHIVE_URL = MODEL_URL + '/3/Standing%20person.zip'
ARCHIVE_SHA256 = '385c4c518662f75b6c719849a4f47100bac7617058407c7d6db41bf802fc5c5e'
ATTRIBUTION = dict(owner='OpenRobotics', author='Marina Kollmitz', version=3,
                   license='CC0-1.0', source=MODEL_URL,
                   license_url='https://creativecommons.org/publicdomain/zero/1.0/')
SITTING_SHA256 = '611ef9ddadfba16be25963f2b79967fec6493c638ca77d0652f18fbbc5202fb3'
SCENES = ('none', 'standing', 'sitting', 'lying', 'postures')


def asset_spec(kind):
    if kind == 'standing':
        return dict(**ATTRIBUTION, sha256=ARCHIVE_SHA256, directory='standing_person')
    if kind == 'sitting':
        return dict(owner='OpenRobotics', author='Ian Chen; Cole Biesemeyer', version=8,
                    license='CC0-1.0', license_url=ATTRIBUTION['license_url'],
                    source='https://fuel.gazebosim.org/1.0/OpenRobotics/models/Rescue%20Randy%20Sitting',
                    sha256=SITTING_SHA256, directory='rescue_randy_sitting')
    raise ValueError('Unknown person asset kind')


def scene_assets(cache_root, scene):
    if scene not in SCENES:
        raise ValueError('Unknown person scene')
    kinds = () if scene == 'none' else ('standing', 'sitting') if scene == 'postures' else ('sitting',) if scene == 'sitting' else ('standing',)
    paths = {}
    for kind in kinds:
        spec = asset_spec(kind)
        path = Path(cache_root)/spec['directory']/f"v{spec['version']}"
        paths[kind] = verify_asset(path, kind)
    return paths


def verify_asset(directory, kind='standing'):
    directory = Path(directory).expanduser().resolve()
    spec = asset_spec(kind)
    archive = directory.parent/f"v{spec['version']}.zip"
    if not archive.is_file() or hashlib.sha256(archive.read_bytes()).hexdigest() != spec['sha256']:
        raise ValueError('Missing or modified person archive; run gazebo prepare first')
    with zipfile.ZipFile(archive) as bundle:
        for entry in bundle.infolist():
            target = (directory/entry.filename).resolve()
            if not target.is_relative_to(directory) or (entry.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError('Unsafe standing-person archive entry')
            if not entry.is_dir() and (not target.is_file() or
                    hashlib.sha256(target.read_bytes()).digest() != hashlib.sha256(bundle.read(entry)).digest()):
                raise ValueError('Missing or modified person asset: '+entry.filename)
    return directory


def prepare_asset(destination, source=None, kind='standing'):
    """Explicit preparation only; launch never downloads or repairs assets."""
    destination = Path(destination).expanduser().resolve()
    if destination.exists():
        return verify_asset(destination, kind)
    spec = asset_spec(kind)
    archive_name = f"v{spec['version']}.zip"
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.standing-', dir=destination.parent) as temporary:
        temporary = Path(temporary)
        archive = temporary/archive_name
        if source:
            shutil.copyfile(source, archive)
        else:
            model_name = spec['source'].rsplit('/', 1)[1]
            url = spec['source']+f"/{spec['version']}/{model_name}.zip"
            with urllib.request.urlopen(url, timeout=60) as response, archive.open('wb') as out:
                shutil.copyfileobj(response, out)
        if hashlib.sha256(archive.read_bytes()).hexdigest() != spec['sha256']:
            raise ValueError('Standing-person archive SHA-256 mismatch')
        with zipfile.ZipFile(archive) as bundle:
            root = temporary/'v3'
            root.mkdir()
            for entry in bundle.infolist():
                target = (root/entry.filename).resolve()
                if not target.is_relative_to(root) or (entry.external_attr >> 16) & 0o170000 == 0o120000:
                    raise ValueError('Unsafe standing-person archive entry')
            bundle.extractall(root)
        pinned = destination.parent/archive_name
        if pinned.exists() and hashlib.sha256(pinned.read_bytes()).hexdigest() != spec['sha256']:
            raise ValueError('Refusing to replace a different standing-person archive')
        if not pinned.exists():
            with pinned.open('xb') as out, archive.open('rb') as src:
                shutil.copyfileobj(src, out)
        root.rename(destination)
    (destination.parent/'attribution.json').write_text(json.dumps(spec, indent=2)+'\n')
    return verify_asset(destination, kind)


def person_element(directory, pose=(3., 3.5, 0., 0., 0., -math.pi/4), posture='standing'):
    kind = 'sitting' if posture == 'sitting' else 'standing'
    directory = verify_asset(directory, kind)
    if posture not in ('standing', 'sitting', 'lying'):
        raise ValueError('Unknown person posture')
    if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
        raise ValueError('Expected a finite six-component person pose')
    model = copy.deepcopy(ET.parse(directory/'model.sdf').getroot().find('model'))
    model.set('name', 'person_target' if posture == 'standing' else 'person_'+posture)
    static = model.find('static')
    if static is None:
        static = ET.SubElement(model, 'static')
    static.text = 'true'
    pose = list(pose)
    if posture == 'lying':
        # A rigid supine variant of the standing mesh, not an articulated model.
        pose[3] -= math.pi/2
        local = mesh_triangles(directory, [0., 0., 0., *pose[3:]])
        pose[2] -= float(local[:, :, 2].min())
    ET.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    # The sitting asset includes a SubT proximity detector and thermal plugin.
    # Keep only static visual/collision geometry, without downloads or sensors.
    for parent in model.iter():
        for child in list(parent):
            if child.tag in ('include', 'plugin'):
                parent.remove(child)
    for uri in model.findall('.//uri'):
        relative = uri.text.split('/', 3)[-1] if uri.text.startswith('model://') else uri.text
        target = (directory/relative).resolve()
        if not target.is_relative_to(directory) or not target.exists():
            raise ValueError('Unexpected external person resource: '+uri.text)
        uri.text = str(target)
    return model


def pose_matrix(pose):
    x, y, z, roll, pitch, yaw = pose
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    result[:3, 3] = [x, y, z]
    return result


def mesh_triangles(directory, pose, kind='standing'):
    """Independent truth from the pinned static visual mesh, including SDF pose.

    Units, static instance transforms and link/visual poses are applied.
    Only static Z-up Collada meshes are supported; no animation is inferred.
    """
    directory = verify_asset(directory) if kind == 'standing' else verify_asset(directory, kind)
    sdf = ET.parse(directory/'model.sdf').getroot()
    link = sdf.find('model/link')
    visual = link.find('visual')
    uri = visual.findtext('geometry/mesh/uri')
    relative = uri.split('/', 3)[-1] if uri.startswith('model://') else uri
    root = ET.parse(directory/relative).getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    unit = float(root.find('c:asset/c:unit', ns).get('meter'))
    if (not math.isfinite(unit) or unit <= 0 or root.findtext('c:asset/c:up_axis', namespaces=ns) != 'Z_UP'):
        raise ValueError('Unsupported mesh units or axes')
    instances = []
    def visit(node, matrix):
        element = node.find('c:matrix', ns)
        if element is not None:
            matrix = matrix @ np.fromstring(element.text, sep=' ').reshape(4, 4)
        for instance in node.findall('c:instance_geometry', ns):
            instances.append((instance.get('url')[1:], matrix.copy()))
        for child in node.findall('c:node', ns):
            visit(child, matrix)
    scene = root.find('c:library_visual_scenes/c:visual_scene', ns)
    for node in scene.findall('c:node', ns):
        visit(node, np.eye(4))
    triangles = []
    for geometry in root.findall('c:library_geometries/c:geometry', ns):
        transforms = [matrix for name, matrix in instances if name == geometry.get('id')]
        if not transforms:
            continue
        mesh = geometry.find('c:mesh', ns)
        source = mesh.find('c:vertices/c:input', ns).get('source')[1:]
        positions = mesh.find(f"c:source[@id='{source}']", ns)
        stride = int(positions.find('c:technique_common/c:accessor', ns).get('stride'))
        if stride != 3:
            raise ValueError('Unexpected vertex stride')
        vertices = np.fromstring(positions.findtext('c:float_array', namespaces=ns), sep=' ').reshape(-1, 3)
        local_triangles = []
        for polygon in mesh.findall('c:polylist', ns)+mesh.findall('c:triangles', ns):
            inputs = polygon.findall('c:input', ns)
            width = max(int(i.get('offset')) for i in inputs)+1
            offset = next(int(i.get('offset')) for i in inputs if i.get('semantic') == 'VERTEX')
            indices = np.fromstring(polygon.findtext('c:p', namespaces=ns), sep=' ', dtype=int).reshape(-1, width)[:, offset]
            counts = (np.full(len(indices)//3, 3) if polygon.tag.endswith('triangles') else
                      np.fromstring(polygon.findtext('c:vcount', namespaces=ns), sep=' ', dtype=int))
            cursor = 0
            for count in counts:
                face = indices[cursor:cursor+count]
                local_triangles.extend(vertices[[face[0], face[i], face[i+1]]] for i in range(1, count-1))
                cursor += count
        for matrix in transforms:
            transformed = (np.asarray(local_triangles) @ matrix[:3, :3].T+matrix[:3, 3])*unit
            triangles.extend(transformed)
    if not triangles:
        raise ValueError('No truth triangles')
    link_pose = [float(v) for v in link.findtext('pose', '0 0 0 0 0 0').split()]
    visual_pose = [float(v) for v in visual.findtext('pose', '0 0 0 0 0 0').split()]
    scale = np.fromstring(visual.findtext('geometry/mesh/scale', '1 1 1'), sep=' ')
    transform = pose_matrix(pose) @ pose_matrix(link_pose) @ pose_matrix(visual_pose)
    return (np.asarray(triangles)*scale) @ transform[:3, :3].T + transform[:3, 3]


def surface_distance(point, triangles):
    """Euclidean nearest triangle distance; independent of depth back-projection."""
    a, b, c = np.asarray(triangles).transpose(1, 0, 2)
    point = np.asarray(point)
    ab, ac, ap = b-a, c-a, point-a
    normal = np.cross(ab, ac)
    squared = np.einsum('ij,ij->i', normal, normal)
    denominator = np.maximum(squared, 1e-24)
    projection = point-normal*(np.einsum('ij,ij->i', ap, normal)/denominator)[:, None]
    v = np.einsum('ij,ij->i', np.cross(projection-a, ac), normal)/denominator
    w = np.einsum('ij,ij->i', np.cross(ab, projection-a), normal)/denominator
    inside = (v >= 0) & (w >= 0) & (v+w <= 1) & (squared > 1e-24)
    distances = np.where(inside, np.linalg.norm(point-projection, axis=1), np.inf)
    for start, end in ((a, b), (b, c), (c, a)):
        edge = end-start
        t = np.clip(np.einsum('ij,ij->i', point-start, edge)/np.maximum(np.einsum('ij,ij->i', edge, edge), 1e-24), 0, 1)
        distances = np.minimum(distances, np.linalg.norm(point-(start+t[:, None]*edge), axis=1))
    return float(distances.min())


def add_person(world_path, directory, protected_volume=None):
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    if world.find("model[@name='person_target']") is not None:
        raise ValueError('Person target already exists')
    model = person_element(directory)
    pose = list(map(float, model.findtext('pose').split()))
    triangles = mesh_triangles(directory, pose)
    lower, upper = triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))
    if protected_volume is not None:
        lo, hi = map(np.asarray, protected_volume)
        if not np.any((upper < lo) | (lower > hi)):
            raise ValueError('Person intersects the protected launch/survey volume')
    world.append(model)
    tree.write(world_path, encoding='utf-8', xml_declaration=True)
    return dict(**ATTRIBUTION, archive_sha256=ARCHIVE_SHA256,
                model_name='person_target', pose_world=pose, bounds_min=lower.tolist(), bounds_max=upper.tolist(),
                protected_volume_clear=protected_volume is not None,
                static=True, scope='rendered mesh and ideal depth; not hardware or dynamic-person acceptance')


def add_person_scene(world_path, cache_root, scene, protected_volume=None):
    paths = scene_assets(cache_root, scene)
    if scene == 'none':
        return dict(scene=scene, models=[], static=True, protected_volume_clear=protected_volume is not None)
    if scene == 'standing':
        return add_person(world_path, paths['standing'], protected_volume)
    placements = dict(standing=(3., 3.5, 0., 0., 0., -math.pi/4),
                      sitting=(-3., 3.5, 0., 0., 0., math.pi/4),
                      lying=(-.8, 4.6, 0., 0., 0., -math.pi/2))
    postures = ('standing', 'sitting', 'lying') if scene == 'postures' else (scene,)
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    records = []
    for posture in postures:
        kind = 'sitting' if posture == 'sitting' else 'standing'
        model = person_element(paths[kind], placements[posture], posture)
        if world.find(f"model[@name='{model.get('name')}']") is not None:
            raise ValueError('Person target already exists')
        pose = list(map(float, model.findtext('pose').split()))
        triangles = mesh_triangles(paths[kind], pose, kind)
        lower, upper = triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))
        if protected_volume is not None:
            lo, hi = map(np.asarray, protected_volume)
            if not np.any((upper < lo) | (lower > hi)):
                raise ValueError('Person intersects the protected launch/survey volume')
        world.append(model)
        records.append(dict(posture=posture, model_name=model.get('name'), asset_kind=kind,
                            pose_world=pose, bounds_min=lower.tolist(), bounds_max=upper.tolist(),
                            source=asset_spec(kind), rigid_lying_variant=posture == 'lying'))
    tree.write(world_path, encoding='utf-8', xml_declaration=True)
    return dict(scene=scene, models=records, static=True,
                protected_volume_clear=protected_volume is not None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['prepare', 'check'])
    parser.add_argument('directory', type=Path)
    parser.add_argument('--source', type=Path, help='Offline ZIP matching the selected kind/version')
    parser.add_argument('--kind', choices=['standing', 'sitting'], default='standing')
    args = parser.parse_args()
    directory = prepare_asset(args.directory, args.source, args.kind) if args.operation == 'prepare' else verify_asset(args.directory, args.kind)
    print(json.dumps(dict(path=str(directory), **asset_spec(args.kind))))


if __name__ == '__main__':
    main()
