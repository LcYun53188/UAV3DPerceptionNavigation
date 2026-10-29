"""Course clearance, hollow collisions, directed crossing and launch ownership."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest

from simulation import cli, profiles
from simulation.manual import runtime_command
from simulation.vio.runtime.obstacle_course import (
    LAYOUT, WORLD, ROOT, TASKS, clearance, generated_world, obstacles, read_layout,
    mission_path, recipe, segment_points, static_check, validate_region, waypoints,
)
from simulation.vio.analysis.assess_obstacle_course import directed_crossing, evaluate, assess, save_pocket_result
from prepare_vio_sensor_assets import assets
from pocket_operator_session import operator_environment, task_parameters


def trace(layout, task):
    path = [[0.,0.,2.5],*waypoints(layout,task),[0.,0.,2.5]]
    points = [path[0]]
    for a,b in zip(path,path[1:]): points += segment_points(a,b,spacing=.04)[1:]
    return [dict(position=p,mono=i*.1,source_sim_s=i*.08,model='x500_vio_ref_7') for i,p in enumerate(points)]


def test_course_and_recipes_match_validated_source_geometry():
    layout = read_layout(); checked = static_check(layout)
    assert checked['passed'] and len(layout['zones']) == 4
    assert WORLD.read_text() == generated_world(layout)
    for task in TASKS[1:]:
        assert json.loads(mission_path(task).read_text()) == recipe(layout,task)
        assert checked['routes'][task]['steps'] <= 20
    assert checked['routes']['all']['steps'] == 20
    validate_region(layout, json.loads((ROOT/'simulation/safe_regions/obstacle_course.json').read_text()))


def test_ring_is_hollow_and_tree_parts_have_real_collisions():
    layout = read_layout(); world = ET.fromstring(generated_world(layout)); solids = obstacles(layout)
    ring = [o for o in solids if o['name'].startswith('ring_')]
    assert len([o for o in ring if o['name'].startswith('ring_segment_')]) == 32
    assert min(clearance([8.,8.,2.5],o) for o in ring) == pytest.approx(2.6)
    assert min(clearance([10.75,8.,2.5],o) for o in ring) == 0
    for model in world.findall('.//model'):
        if model.get('name').startswith(('ring_', 'tree_', 'gate_', 'column_')):
            assert len(model.findall('.//collision')) == len(model.findall('.//visual')) == 1
    assert len(world.findall('.//model')) > 50


def test_braking_envelope_and_central_launch_cannot_be_weakened():
    layout = read_layout(); bad = deepcopy(layout)
    bad['zones'][1]['opening_m'][0] = 3.0
    with pytest.raises(ValueError, match='braking/voxel'): static_check(bad)
    bad = deepcopy(layout); bad['zones'][0]['columns_xy'].append([0.,0.])
    with pytest.raises(ValueError, match='protected'): static_check(bad)
    config = json.loads((ROOT/'simulation/safe_regions/obstacle_course.json').read_text())
    config['braking_margin_m'] = .1
    with pytest.raises(ValueError, match='margin mismatch'): validate_region(layout,config)


@pytest.mark.parametrize('task', TASKS[1:])
def test_continuous_reference_path_passes_geometry_only(task):
    layout = read_layout(); result = evaluate(layout,task,trace(layout,task))
    assert result['passed'], result
    assert 'mission_completed' not in result['checks']


def test_gate_rejects_reverse_wrong_height_and_outside_opening():
    layout = read_layout(); rectangle,ring = layout['zones'][1:3]
    assert directed_crossing([7.,-8.,2.5],[9.,-8.,2.5],rectangle,.8)
    assert not directed_crossing([9.,-8.,2.5],[7.,-8.,2.5],rectangle,.8)
    assert not directed_crossing([7.,-8.,4.],[9.,-8.,4.],rectangle,.8)
    assert not directed_crossing([7.,-4.,2.5],[9.,-4.,2.5],rectangle,.8)
    assert not directed_crossing([10.,7.,2.5],[10.,9.,2.5],ring,.8)


@pytest.mark.parametrize('fault', ['missing', 'out_of_order', 'gap', 'duplicate_time', 'teleport', 'collision'])
def test_incomplete_or_discontinuous_or_colliding_run_cannot_pass(fault):
    layout = read_layout(); rows = trace(layout,'rectangle')
    if fault == 'missing': rows = rows[:20]
    elif fault == 'out_of_order': rows.reverse()
    elif fault == 'gap':
        for r in rows[10:]: r['source_sim_s'] += 1.; r['mono'] += 1.
    elif fault == 'duplicate_time': rows[10]['source_sim_s'] = rows[9]['source_sim_s']
    elif fault == 'teleport': rows[10]['position'] = [0,0,2.5]
    else: rows[10]['position'] = [8.,-5.38,2.5]
    assert not evaluate(layout,'rectangle',rows)['passed']


def test_vio_assets_replace_geometry_without_changing_sensor_rig(tmp_path):
    upstream = tmp_path/'upstream.sdf'
    upstream.write_text('<sdf version="1.9"><world name="default"/></sdf>')
    for task in ('none','all'):
        assets(tmp_path/task,upstream,scene='warehouse',warehouse_floor_texture=True,
               warehouse_variant='avoidance',course=task)
    for name in ('x500_vio_ref/model.sdf','frames.json'):
        assert (tmp_path/'none'/name).read_bytes() == (tmp_path/'all'/name).read_bytes()
    root = ET.parse(tmp_path/'all/default.sdf')
    assert root.findtext('.//world/gravity') == '0 0 -9.81'
    floor = root.find('.//model[@name="course_textured_floor"]/link/visual')
    assert floor.findtext('material/ambient') == '1 1 1 1'
    assert floor.findtext('material/diffuse') == '1 1 1 1'
    assert root.find('.//model[@name="avoidance_crate"]') is None
    assert root.find('.//model[@name="ring_segment_00"]/link/collision') is not None
    receipt = json.loads((tmp_path/'all/warehouse-layout.json').read_text())
    assert receipt['layout_sha256'] == hashlib.sha256(LAYOUT.read_bytes()).hexdigest()
    for name,digest in receipt['texture_sha256'].items():
        assert hashlib.sha256((tmp_path/'all'/name).read_bytes()).hexdigest() == digest


def test_full_course_selection_stays_opt_in_and_keeps_physical_input(capsys):
    config = profiles.validate_config(dict(schema=1,profile='full',options=dict(course='all')))
    command = profiles.command(config)
    assert command[command.index('--course')+1] == 'all'
    assert command[command.index('--input-source')+1] == 'physical'
    cli.main(['run','full','--course','ring','--dry-run'])
    assert '--course ring' in capsys.readouterr().out
    identity = dict(device='/dev/input/js0',input_profile='pocket-rc-v2')
    run = runtime_command(identity,3600,program='full',course='all')
    assert '--pocket-session' in run and '--vio-primary' in run and '--course' in run
    assert '--course' not in runtime_command(identity,3600,program='full')
    with pytest.raises(ValueError): runtime_command(identity,3600,program='manual',course='all')
    with pytest.raises(ValueError): profiles.validate_config(dict(schema=1,profile='full',options=dict(course='all',people=True,person_scene='standing')))


def test_course_operator_environment_requires_matching_recipe_and_region(tmp_path):
    (tmp_path/'flight-assets.json').write_text('{}')
    expected = dict(UAV_REQUIRE_VIO='1',UAV_LOCALIZATION_PROFILE='DEFAULT',UAV_VIO_FUSION_PROFILE='aligned_pose_mtf_primary_sim_v1',
        UAV_VIO_CALIBRATION_ID='calibration',UAV_ENABLE_FUSION_SWITCH='0',UAV_FLIGHT_REGION_PROFILE='obstacle_course',
        UAV_ENABLE_EGO_NAV='1',UAV_NAVIGATION_BACKEND='EGO',UAV_EXPERIMENTAL_VIO_CHAIN='1',UAV_EXPECT_OBSERVATION='1',
        UAV_FLIGHT_COORDINATOR_INSTANCE='run',UAV_FLIGHT_MODEL='x500_vio_ref_7',
        UAV_FLIGHT_ASSET_RECEIPT=str(tmp_path/'flight-assets.json'),UAV_FLIGHT_EVIDENCE=str(tmp_path),
        UAV_FLIGHT_ADMISSION=str(tmp_path/'flight-admission.json'),UAV_FLIGHT_MISSION_FILE=str(mission_path('all')),
        UAV_FLIGHT_ASSET_SHA256=hashlib.sha256(b'{}').hexdigest())
    manifest = dict(localization_profile='VIO_FLOW',calibration_id='calibration',run_id='run',vio_primary=True,
                    sdk_schur64=True,obstacle_course='all',operator_environment=expected)
    assert operator_environment(tmp_path,manifest) == expected
    manifest['obstacle_course'] = 'ring'
    with pytest.raises(ValueError,match='ownership mismatch'): operator_environment(tmp_path,manifest)


def test_course_task_rejects_starting_from_a_shifted_launch(monkeypatch):
    monkeypatch.setenv('UAV_FLIGHT_MISSION_FILE',str(mission_path('all')))
    node = SimpleNamespace(position=lambda:(2.,0.,0.))
    with pytest.raises(ValueError,match='CENTRAL_LAUNCH'):
        task_parameters(node,dict(localization_profile='VIO_FLOW',obstacle_course='all'))


def test_success_flag_alone_cannot_qualify_a_course(tmp_path):
    (tmp_path/'assets').mkdir()
    layout = read_layout(); manifest = dict(obstacle_course='rectangle',run_id='run',model='x500_vio_ref_7')
    rows = trace(layout,'rectangle'); rows = rows[:20]
    (tmp_path/'manifest.json').write_text(json.dumps(manifest))
    (tmp_path/'assets/default.sdf').write_text('<sdf/>')
    (tmp_path/'assets/warehouse-layout.json').write_text(json.dumps(dict(layout=layout)))
    (tmp_path/'flight-assets.json').write_text(json.dumps(dict(files={name:hashlib.sha256((tmp_path/name).read_bytes()).hexdigest()
        for name in ('assets/default.sdf','assets/warehouse-layout.json')})))
    (tmp_path/'course-truth.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
    (tmp_path/'course-flight.json').write_text(json.dumps(dict(run_id='run',start_mono=0.,end_mono=rows[-1]['mono'],
        home_truth=[0,0,0],exit_code=0,result=['SUCCEEDED','LANDED_AND_DISARMED',True],landed=True,arming=1,navigation_backend='EGO')))
    result = assess(tmp_path)
    assert result['checks']['mission_completed'] and not result['passed']


def test_airborne_cancel_does_not_replay_geometry_or_block_the_gateway(tmp_path):
    node = SimpleNamespace(samples=dict(vehicle_land_detected=SimpleNamespace(landed=False),
                                       vehicle_status=SimpleNamespace(arming_state=2)))
    result = save_pocket_result(tmp_path,node,{},130)
    assert not result['passed'] and result['reason'] == 'COURSE_AUDIT_REQUIRES_LANDED_DISARMED'
    assert not (tmp_path/'course-flight.json').exists()
