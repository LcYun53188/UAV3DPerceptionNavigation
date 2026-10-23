"""Pinned OpenRobotics CC0 standing-person asset and opt-in Gazebo placement."""
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


def verify_asset(directory):
    directory = Path(directory).expanduser().resolve()
    archive = directory.parent/'v3.zip'
    if not archive.is_file() or hashlib.sha256(archive.read_bytes()).hexdigest() != ARCHIVE_SHA256:
        raise ValueError('Missing or modified standing-person archive; run gazebo prepare first')
    with zipfile.ZipFile(archive) as bundle:
        for entry in bundle.infolist():
            target = (directory/entry.filename).resolve()
            if not target.is_relative_to(directory) or (entry.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError('Unsafe standing-person archive entry')
            if not entry.is_dir() and (not target.is_file() or
                    hashlib.sha256(target.read_bytes()).digest() != hashlib.sha256(bundle.read(entry)).digest()):
                raise ValueError('Missing or modified standing-person asset: '+entry.filename)
    return directory


def prepare_asset(destination, source=None):
    """Explicit preparation only; launch never downloads or repairs assets."""
    destination = Path(destination).expanduser().resolve()
    if destination.exists():
        return verify_asset(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='.standing-', dir=destination.parent) as temporary:
        temporary = Path(temporary)
        archive = temporary/'v3.zip'
        if source:
            shutil.copyfile(source, archive)
        else:
            with urllib.request.urlopen(ARCHIVE_URL, timeout=60) as response, archive.open('wb') as out:
                shutil.copyfileobj(response, out)
        if hashlib.sha256(archive.read_bytes()).hexdigest() != ARCHIVE_SHA256:
            raise ValueError('Standing-person archive SHA-256 mismatch')
        with zipfile.ZipFile(archive) as bundle:
            root = temporary/'v3'
            root.mkdir()
            for entry in bundle.infolist():
                target = (root/entry.filename).resolve()
                if not target.is_relative_to(root) or (entry.external_attr >> 16) & 0o170000 == 0o120000:
                    raise ValueError('Unsafe standing-person archive entry')
            bundle.extractall(root)
        pinned = destination.parent/'v3.zip'
        if pinned.exists() and hashlib.sha256(pinned.read_bytes()).hexdigest() != ARCHIVE_SHA256:
            raise ValueError('Refusing to replace a different standing-person archive')
        if not pinned.exists():
            with pinned.open('xb') as out, archive.open('rb') as src:
                shutil.copyfileobj(src, out)
        root.rename(destination)
    (destination.parent/'attribution.json').write_text(json.dumps(ATTRIBUTION, indent=2)+'\n')
    return verify_asset(destination)


def person_element(directory, pose=(3., 3.5, 0., 0., 0., -math.pi/4)):
    directory = verify_asset(directory)
    if len(pose) != 6 or not all(math.isfinite(x) for x in pose):
        raise ValueError('Expected a finite six-component person pose')
    model = copy.deepcopy(ET.parse(directory/'model.sdf').getroot().find('model'))
    model.set('name', 'person_target')
    ET.SubElement(model, 'static').text = 'true'
    ET.SubElement(model, 'pose').text = ' '.join(map(str, pose))
    for uri in model.findall('.//mesh/uri'):
        if uri.text != 'model://person_standing/meshes/standing.dae':
            raise ValueError('Unexpected mesh in pinned standing-person model')
        uri.text = str(directory/'meshes/standing.dae')
    return model


def pose_matrix(pose):
    x, y, z, roll, pitch, yaw = pose
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                       [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    result[:3, 3] = [x, y, z]
    return result


def mesh_triangles(directory, pose):
    """Independent truth from the pinned static visual mesh, including SDF pose.

    This parser deliberately accepts only this asset's identity mesh instances,
    metre/Z-up convention and polygon lists; no skin animation is inferred.
    """
    directory = verify_asset(directory)
    root = ET.parse(directory/'meshes/standing.dae').getroot()
    ns = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
    if (root.find('c:asset/c:unit', ns).get('meter') != '1'
            or root.findtext('c:asset/c:up_axis', namespaces=ns) != 'Z_UP'):
        raise ValueError('Unsupported mesh units or axes')
    instances = set()
    def visit(node, matrix):
        element = node.find('c:matrix', ns)
        if element is not None:
            matrix = matrix @ np.fromstring(element.text, sep=' ').reshape(4, 4)
        for instance in node.findall('c:instance_geometry', ns):
            if not np.allclose(matrix, np.eye(4), atol=1e-8):
                raise ValueError('Unexpected mesh instance transform')
            instances.add(instance.get('url')[1:])
        for child in node.findall('c:node', ns):
            visit(child, matrix)
    scene = root.find('c:library_visual_scenes/c:visual_scene', ns)
    for node in scene.findall('c:node', ns):
        visit(node, np.eye(4))
    triangles = []
    for geometry in root.findall('c:library_geometries/c:geometry', ns):
        if geometry.get('id') not in instances:
            continue
        mesh = geometry.find('c:mesh', ns)
        source = mesh.find('c:vertices/c:input', ns).get('source')[1:]
        positions = mesh.find(f"c:source[@id='{source}']", ns)
        stride = int(positions.find('c:technique_common/c:accessor', ns).get('stride'))
        if stride != 3:
            raise ValueError('Unexpected vertex stride')
        vertices = np.fromstring(positions.findtext('c:float_array', namespaces=ns), sep=' ').reshape(-1, 3)
        for polygon in mesh.findall('c:polylist', ns):
            inputs = polygon.findall('c:input', ns)
            width = max(int(i.get('offset')) for i in inputs)+1
            offset = next(int(i.get('offset')) for i in inputs if i.get('semantic') == 'VERTEX')
            indices = np.fromstring(polygon.findtext('c:p', namespaces=ns), sep=' ', dtype=int).reshape(-1, width)[:, offset]
            counts = np.fromstring(polygon.findtext('c:vcount', namespaces=ns), sep=' ', dtype=int)
            cursor = 0
            for count in counts:
                face = indices[cursor:cursor+count]
                triangles.extend(vertices[[face[0], face[i], face[i+1]]] for i in range(1, count-1))
                cursor += count
    if not triangles:
        raise ValueError('No truth triangles')
    visual_pose = [float(v) for v in ET.parse(directory/'model.sdf').getroot().findtext('model/link/visual/pose').split()]
    transform = pose_matrix(pose) @ pose_matrix(visual_pose)
    return np.asarray(triangles) @ transform[:3, :3].T + transform[:3, 3]


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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=['prepare', 'check'])
    parser.add_argument('directory', type=Path)
    parser.add_argument('--source', type=Path, help='Offline pinned v3.zip')
    args = parser.parse_args()
    directory = prepare_asset(args.directory, args.source) if args.operation == 'prepare' else verify_asset(args.directory)
    print(json.dumps(dict(directory=str(directory), archive_sha256=ARCHIVE_SHA256, **ATTRIBUTION)))


if __name__ == '__main__':
    main()
