"""Check rendered geometry, inflated clearance and bounded mission targets."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parent))
from prepare_vio_sensor_assets import assets
from vio_warehouse_scene import LAYOUT, box_clearance


def test_inflated_flight_volume_excludes_every_actual_collision(tmp_path):
    world=tmp_path/'world.sdf';world.write_text('<sdf version="1.9"><world name="default"/></sdf>')
    assets(tmp_path/'a',world,scene='warehouse',resolution=(480,300))
    receipt=json.loads((tmp_path/'a/warehouse-layout.json').read_text())
    layout=receipt['layout'];assert receipt['clearance']['passed']
    generated=ET.parse(tmp_path/'a/default.sdf')
    for obstacle in layout['obstacles']:
        model=generated.find(f'.//model[@name="{obstacle["name"]}"]')
        assert model.findtext('pose').split()[:3]==list(map(str,obstacle['center']))
        assert list(map(float,model.findtext('.//collision/geometry/box/size').split()))==obstacle['size']
    # Door opening is 2.4 m wide and 3 m high; not in the first flight box.
    assert generated.find('.//model[@name="door_lintel"]') is not None
    assert len(generated.findall('.//collision'))==len(layout['obstacles'])
    assert len(receipt['texture_sha256'])==len(layout['obstacles'])
    for name,digest in receipt['texture_sha256'].items():
        raw=(tmp_path/'a'/name).read_bytes()
        assert raw[:8]==b'\x89PNG\r\n\x1a\n' and hashlib.sha256(raw).hexdigest()==digest
    recipe=json.loads((LAYOUT.parents[1]/'missions/warehouse_vio_sequence.json').read_text())
    for step in recipe['steps']:
        point=step.get('offset_enu',[0,0,step.get('height_m',0)])
        assert all(lo<=v<=hi for lo,v,hi in zip(layout['flight_box_min'],point,layout['flight_box_max']))


def test_obstacle_touching_or_inside_envelope_is_rejected():
    layout=json.loads(LAYOUT.read_text())
    for center in ([0,0,1],[4.1,0,1]):
        invalid=deepcopy(layout)
        invalid['obstacles'].append(dict(name='unsafe',center=center,size=[.2,.2,1]))
        with pytest.raises(ValueError,match='inflated flight volume'):box_clearance(invalid)


def test_warehouse_generation_deterministic_without_upstream_changes(tmp_path):
    world=tmp_path/'world.sdf';raw='<sdf version="1.9"><world name="default"/></sdf>';world.write_text(raw)
    assets(tmp_path/'a',world,scene='warehouse');assets(tmp_path/'b',world,scene='warehouse')
    for name in ('default.sdf','warehouse-layout.json','frames.json'):
        assert (tmp_path/'a'/name).read_bytes()==(tmp_path/'b'/name).read_bytes()
    assert world.read_text()==raw


def test_floor_texture_does_not_change_any_obstacle_or_camera(tmp_path):
    world=tmp_path/'world.sdf';world.write_text('<sdf version="1.9"><world name="default"/></sdf>')
    assets(tmp_path/'a',world,scene='warehouse')
    profile,_=assets(tmp_path/'b',world,scene='warehouse',warehouse_floor_texture=True)
    assert profile['warehouse_floor_texture']
    a=ET.parse(tmp_path/'a/default.sdf');b=ET.parse(tmp_path/'b/default.sdf')
    for obstacle in json.loads(LAYOUT.read_text())['obstacles']:
        query=f'.//model[@name="{obstacle["name"]}"]'
        assert ET.tostring(a.find(query))==ET.tostring(b.find(query))
    assert (tmp_path/'a/x500_vio_ref/model.sdf').read_bytes()==(tmp_path/'b/x500_vio_ref/model.sdf').read_bytes()
    floor=b.find('.//model[@name="warehouse_textured_floor"]')
    assert floor is not None and floor.find('.//collision') is None
    assert floor.findtext('.//albedo_map').endswith('/floor.png')
