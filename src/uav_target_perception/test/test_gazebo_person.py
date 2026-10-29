import hashlib
from pathlib import Path
import zipfile
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from uav_target_perception import gazebo


@pytest.mark.parametrize('point,expected', [([.25, .25, .3], .3), ([1, 1, 0], np.sqrt(.5)),
                                           ([-1, 0, 0], 1), ([0, 0, 0], 0)])
def test_independent_surface_distance_includes_faces_edges_and_vertices(point, expected):
    triangles = np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 0.]]])
    assert gazebo.surface_distance(point, triangles) == pytest.approx(expected)


def test_truth_transform_rotates_and_translates_in_sdf_order():
    matrix = gazebo.pose_matrix([2, 3, 4, 0, 0, np.pi/2])
    assert np.allclose(matrix @ [1, 0, 0, 1], [2, 4, 4, 1])


def test_asset_preparation_checks_archive_and_rejects_modified_files(tmp_path, monkeypatch):
    archive = tmp_path/'source.zip'
    with zipfile.ZipFile(archive, 'w') as out:
        out.writestr('model.sdf', '<sdf/>')
        out.writestr('meshes/standing.dae', 'pinned mesh')
    monkeypatch.setattr(gazebo, 'ARCHIVE_SHA256', hashlib.sha256(archive.read_bytes()).hexdigest())
    target = tmp_path/'cache/v3'
    assert gazebo.prepare_asset(target, archive) == target
    (target/'meshes/standing.dae').write_text('modified')
    with pytest.raises(ValueError, match='modified'):
        gazebo.verify_asset(target)
    with pytest.raises(ValueError, match='modified'):
        gazebo.prepare_asset(target, archive)


def test_asset_preparation_rejects_archive_path_escape(tmp_path, monkeypatch):
    archive = tmp_path/'unsafe.zip'
    with zipfile.ZipFile(archive, 'w') as out:
        out.writestr('../escaped.txt', 'invalid')
    monkeypatch.setattr(gazebo, 'ARCHIVE_SHA256', hashlib.sha256(archive.read_bytes()).hexdigest())
    with pytest.raises(ValueError, match='Unsafe'):
        gazebo.prepare_asset(tmp_path/'cache/v3', archive)
    assert not (tmp_path/'cache/escaped.txt').exists()


def test_default_person_scene_and_explicit_detection_rejection():
    from simulation import profiles
    from simulation.manual import runtime_command
    command = profiles.command(dict(schema=1, profile='full'))
    assert command[command.index('--person-scene')+1] == 'standing'
    with pytest.raises(ValueError):
        profiles.command(dict(schema=1, profile='full', options={'people': False, 'person_scene': 'standing'}))
    command = profiles.command(dict(schema=1, profile='full', options={'people': True, 'person_scene': 'standing'}))
    assert command[command.index('--person-scene')+1] == 'standing'
    identity = dict(device='/dev/input/js0', input_profile='pocket-rc-v2')
    with pytest.raises(ValueError):
        runtime_command(identity, 3600, person_scene='standing')
    command = runtime_command(identity, 3600, program='full', person_model=Path('model.onnx'), person_scene='standing')
    assert '--person-scene' in command


def test_posture_choices_and_rotation_inference_are_forwarded():
    from simulation import profiles
    from simulation.manual import runtime_command
    identity = dict(device='/dev/input/js0', input_profile='pocket-rc-v2')
    for scene in ['sitting', 'lying', 'postures']:
        command = profiles.command(dict(schema=1, profile='full', options={
            'people': True, 'person_scene': scene, 'person_rotations': True}))
        assert command[command.index('--person-scene')+1] == scene
        assert '--person-rotations' in command
        assert '--person-rotations' in runtime_command(identity, 3600, program='full',
            person_model=Path('model.onnx'), person_scene=scene, person_rotations=True)
    with pytest.raises(ValueError):
        profiles.command(dict(schema=1, profile='full', options={'people': False, 'person_rotations': True}))


def test_sitting_model_has_no_external_sensor_dependencies(tmp_path, monkeypatch):
    (tmp_path/'meshes').mkdir()
    (tmp_path/'meshes/sitting.dae').write_text('mesh')
    sdf = '<sdf><model><static>true</static><include><uri>https://example.invalid/sensor</uri></include><link><visual><geometry><mesh><uri>meshes/sitting.dae</uri></mesh></geometry><plugin name="thermal"/></visual></link></model></sdf>'
    (tmp_path/'model.sdf').write_text(sdf)
    monkeypatch.setattr(gazebo, 'verify_asset', lambda *_: tmp_path)
    model = gazebo.person_element(tmp_path, [0, 0, 0, 0, 0, 0], 'sitting')
    assert not model.findall('.//plugin') and not model.findall('.//include')
    assert len(model.findall('static')) == 1
    assert (tmp_path/'model.sdf').read_text() == sdf


def test_scene_rejects_person_inside_protected_volume_without_writing(tmp_path, monkeypatch):
    path = tmp_path/'world.sdf'
    path.write_text('<sdf><world name="fixture"/></sdf>')
    original = path.read_bytes()
    model = ET.fromstring('<model name="person_target"><pose>0 0 0 0 0 0</pose></model>')
    monkeypatch.setattr(gazebo, 'person_element', lambda _: model)
    monkeypatch.setattr(gazebo, 'mesh_triangles', lambda *_: np.array([[[0., 0., 0.], [1., 0., 0.], [0., 1., 1.]]]))
    with pytest.raises(ValueError, match='protected'):
        gazebo.add_person(path, tmp_path, ([-1, -1, -1], [2, 2, 2]))
    assert path.read_bytes() == original


def test_mesh_truth_applies_visual_and_world_poses(tmp_path, monkeypatch):
    (tmp_path/'meshes').mkdir()
    (tmp_path/'model.sdf').write_text('<sdf><model><link><visual><pose>0 0 .02 0 0 0</pose><geometry><mesh><uri>meshes/standing.dae</uri></mesh></geometry></visual></link></model></sdf>')
    (tmp_path/'meshes/standing.dae').write_text('''
    <COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema">
      <asset><unit meter="1"/><up_axis>Z_UP</up_axis></asset>
      <library_visual_scenes><visual_scene><node><instance_geometry url="#quad"/></node></visual_scene></library_visual_scenes>
      <library_geometries><geometry id="quad"><mesh>
        <source id="positions"><float_array>0 0 0 1 0 0 1 1 0 0 1 0</float_array>
          <technique_common><accessor stride="3"/></technique_common></source>
        <vertices><input semantic="POSITION" source="#positions"/></vertices>
        <polylist><input semantic="VERTEX" offset="0"/><vcount>4</vcount><p>0 1 2 3</p></polylist>
      </mesh></geometry></library_geometries>
    </COLLADA>''')
    monkeypatch.setattr(gazebo, 'verify_asset', lambda _: tmp_path)
    triangles = gazebo.mesh_triangles(tmp_path, [3, 0, 0, 0, 0, np.pi/2])
    assert triangles.shape == (2, 3, 3)
    assert np.allclose(triangles.min(axis=(0, 1)), [2, 0, .02])
    assert np.allclose(triangles.max(axis=(0, 1)), [3, 1, .02])
    assert gazebo.surface_distance([2.5, .5, .12], triangles) == pytest.approx(.1)
