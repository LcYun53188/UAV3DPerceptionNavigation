from pathlib import Path
from types import SimpleNamespace as NS
import subprocess
import sys
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from support.paths import enable_imports
enable_imports()
from simulation import cli, menu, profiles
from simulation.manual import runtime_command
from vio_chain_mapping import export_grounded_color_map, launch_mapping


def test_full_defaults_keep_person_perception_disabled():
    command = profiles.command(dict(schema=1, profile='full'))
    assert '--people' not in command and '--person-model' not in command


def test_cli_explicit_people_flag_is_forwarded_without_adding_a_third_program(monkeypatch):
    launch = Mock()
    monkeypatch.setattr(menu, 'launch', launch)
    assert cli.main(['run', 'full', '--people', '--person-model', 'model with spaces.onnx']) == 0
    command = profiles.command(launch.call_args.args[0])
    assert '--people' in command
    assert command[command.index('--person-model')+1] == str(ROOT/'model with spaces.onnx')
    assert set(profiles.BY_KEY) == {'manual', 'full'}


def test_person_perception_cannot_be_added_to_flow_only_manual():
    identity = dict(device='/dev/input/js0', input_profile='pocket-rc-v2')
    with pytest.raises(ValueError, match='full'):
        runtime_command(identity, 3600, program='manual', person_model=Path('model.onnx'))
    command = runtime_command(identity, 3600, program='full', person_model=Path('model.onnx'))
    assert '--people' in command and '--pocket-session' in command


def test_vio_mapping_adds_owned_rgb_and_person_processes_only_when_requested(tmp_path):
    launch = Mock()
    launch_mapping(launch, {}, tmp_path, 'test', depth_only=True)
    assert [call.args[0] for call in launch.call_args_list] == ['depth_bridge']
    launch.reset_mock()
    launch_mapping(launch, {}, tmp_path, 'test', depth_only=True, person_model=Path('/tmp/model.onnx'))
    commands = {call.args[0]: list(map(str, call.args[1])) for call in launch.call_args_list}
    assert set(commands) == {'depth_bridge', 'person_rgb_bridge', 'person_mapper'}
    assert 'context_source:=planning_context' in commands['person_mapper']
    assert '/px4_reference/rgbd/image:=/px4_rgb/image' in commands['person_rgb_bridge']
    assert not any('use_color:=true' in part for command in commands.values() for part in command)


def test_person_child_selects_new_messages_without_replacing_mission_overlay(tmp_path):
    launch = Mock()
    launch_mapping(launch, {}, tmp_path, 'test', depth_only=True, person_model=Path('/tmp/model.onnx'))
    command = next(call.args[1] for call in launch.call_args_list if call.args[0] == 'person_mapper')
    boundary = command.index('ros2')
    probe = ('from uav_nav_interfaces.msg import PersonObservationArray; '
             'from rclpy.type_support import check_for_type_support; '
             'check_for_type_support(PersonObservationArray); print(PersonObservationArray.__name__)')
    result = subprocess.run([*map(str, command[:boundary]), str(ROOT/'.venv/bin/python'), '-c', probe],
                            capture_output=True, text=True, check=True)
    assert result.stdout.strip() == 'PersonObservationArray'


def test_color_mapping_is_explicit_and_uses_gated_rgb_with_one_shared_bridge(tmp_path):
    (tmp_path/'assets').mkdir()
    (tmp_path/'assets/default.sdf').write_text('<sdf/>')
    launch = Mock()
    launch_mapping(launch, {}, tmp_path, 'test', graph_only=True, color_map=True)
    commands = {call.args[0]: list(map(str, call.args[1])) for call in launch.call_args_list}
    assert 'use_color:=true' in commands['nvblox']
    assert 'camera_0/color/image:=/uav/mapping/color' in commands['nvblox']
    assert '/rgbd_camera/image:=/px4_rgb/image' in commands['map_session']
    assert 'person_rgb_bridge' not in commands
    launch.reset_mock()
    launch_mapping(launch, {}, tmp_path, 'test', depth_only=True,
                   color_map=True, person_model=Path('/tmp/model.onnx'))
    names = [call.args[0] for call in launch.call_args_list]
    assert names.count('person_rgb_bridge') == 1


def test_color_cli_does_not_enable_people_or_add_a_program():
    command = profiles.command(dict(schema=1, profile='full', options=dict(color_map=True)))
    assert '--color-map' in command and '--people' not in command
    identity = dict(device='/dev/input/js0', input_profile='pocket-rc-v2')
    assert '--color-map' in runtime_command(identity, 3600, program='full', color_map=True)
    with pytest.raises(ValueError):
        runtime_command(identity, 3600, program='manual', color_map=True)


def test_automated_export_uses_guarded_map_session_and_returns_actual_file_receipt(tmp_path):
    target = tmp_path/'color.ply'
    future = Mock()
    future.done.return_value = True
    future.result.return_value = NS(success=True)
    def write(request):
        Path(request.file_path).write_text('ply\nformat ascii 1.0\nend_header\n')
        return future
    client = Mock()
    client.call_async.side_effect = write
    node = Mock()
    node.create_client.return_value = client
    receipt = export_grounded_color_map(node, target)
    assert node.create_client.call_args.args[1] == '/uav/map/export_mesh'
    assert receipt['guard'] == 'fresh_unique_px4_landed_disarmed'
    assert receipt['bytes'] == target.stat().st_size and len(receipt['sha256']) == 64
    node.destroy_client.assert_called_once_with(client)
    with pytest.raises(ValueError):
        export_grounded_color_map(node, target)


def test_export_rejection_cannot_be_reported_as_a_saved_map(tmp_path):
    node = Mock()
    future = node.create_client.return_value.call_async.return_value
    future.done.return_value = True
    future.result.return_value = NS(success=False)
    with pytest.raises(RuntimeError, match='rejected'):
        export_grounded_color_map(node, tmp_path/'new.ply')
