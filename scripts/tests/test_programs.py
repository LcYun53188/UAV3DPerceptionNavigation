"""Public programs, input ownership and second-terminal session isolation."""
import json
import os
from pathlib import Path
from unittest.mock import Mock

import pytest

from simulation import cli, control, menu, profiles, session
from simulation.manual import runtime_command
from pocket_operator_session import manual_command_allowed
from pocket_input import configure_qgc_input


def test_only_two_programs_default_to_physical_hid(capsys):
    assert set(profiles.BY_KEY) == {'manual', 'full'}
    cli.main(['list'])
    assert len(capsys.readouterr().out.splitlines()) == 2
    for key in profiles.BY_KEY:
        config = profiles.validate_config(dict(schema=1, profile=key))
        assert config['options']['input_source'] == 'physical'
        assert '--input-source physical' in ' '.join(profiles.command(config))


def test_full_default_people_can_be_disabled_and_course_has_no_warehouse_person():
    default = profiles.validate_config(dict(schema=1, profile='full'))
    assert default['options']['people'] and default['options']['person_scene'] == 'standing'
    disabled = profiles.validate_config(dict(schema=1, profile='full', options=dict(people=False)))
    assert disabled['options']['person_scene'] == 'none'
    assert '--people' not in profiles.command(disabled)
    course = profiles.validate_config(dict(schema=1, profile='full', options=dict(course='all')))
    assert course['options']['person_scene'] == 'none'
    assert '--person-scene' not in profiles.command(course)
    assert profiles.validate_config(default) == default


def test_menu_configuration_error_returns_to_menu(monkeypatch, capsys):
    answers = iter(['l', '/missing/config.json', '0'])
    monkeypatch.setattr('builtins.input', lambda _: next(answers))
    assert menu.interactive(dry_run=True) == 0
    assert '无法启动' in capsys.readouterr().err


@pytest.mark.parametrize('key', ['manual', 'full'])
def test_menu_and_saved_config_preview_without_starting(key, tmp_path, monkeypatch, capsys):
    launch = Mock()
    monkeypatch.setattr(menu, 'launch', launch)
    filename = tmp_path / 'config with spaces.json'
    cli.main(['run', key, '--input-source', 'virtual', '--save', str(filename), '--dry-run'])
    cli.main(['run', '--config', str(filename), '--dry-run'])
    assert '--input-source virtual' in capsys.readouterr().out
    launch.assert_not_called()


@pytest.mark.parametrize('old', ['algorithm', 'advanced', 'start', 'px4-flow-local', 'px4-vio-pocket', 'compat'])
def test_retired_routes_cannot_start_a_simulator(old):
    with pytest.raises(ValueError, match='旧仿真入口已移除'):
        cli.main([old])


@pytest.mark.parametrize('options', [dict(input_source='auto'), dict(session_duration=0),
                                     dict(session_duration=True), dict(vio=False)])
def test_invalid_or_retired_options_rejected(options):
    with pytest.raises(ValueError):
        profiles.validate_config(dict(schema=1, profile='manual', options=options))


def test_manual_is_flow_only_and_full_preserves_task_control():
    identity = dict(device='/dev/input/js0', input_profile='pocket-rc-v2')
    manual = runtime_command(identity, 3600)
    full = runtime_command(identity, 3600, program='full')
    assert any(item.endswith('/flow/run_px4_mtf01.py') for item in manual)
    assert '--manual-only' in manual and '--usb-joystick' in manual
    assert '--flight' not in manual and not any('sdk-' in item for item in manual)
    assert any(item.endswith('/vio/runtime/run_px4_vio_sensors.py') for item in full)
    assert '--manual-only' not in full and '--pocket-session' in full
    assert '--sdk-schur64' in full and '--vio-primary' in full
    assert not manual_command_allowed('PREPARE') and not manual_command_allowed('RUN')


@pytest.fixture
def owned(tmp_path, monkeypatch):
    monkeypatch.setattr(session, 'CACHE', tmp_path)
    monkeypatch.setattr(session, 'STATE', tmp_path/'current.json')
    with session.own_session('full') as environment:
        monkeypatch.setenv('UAV_SIM_OPERATOR_ID', environment['UAV_SIM_OPERATOR_ID'])
        folder = tmp_path/'runtime'
        folder.mkdir()
        monkeypatch.setattr(session, 'ROOT', tmp_path)
        # Runtime paths must be in ROOT/.cache/simulation.
        folder = tmp_path/'.cache/simulation/runtime'
        folder.mkdir(parents=True)
        manifest = dict(domain=78, partition='owned-test')
        (folder/'manifest.json').write_text(json.dumps(manifest))
        session.register_runtime(folder, manifest)
        yield folder, session.read()


def test_owner_lock_and_process_identity_prevent_session_reuse(owned):
    folder, state = owned
    assert session.runtime(state) == folder
    with pytest.raises(RuntimeError, match='已有仿真'):
        with session.own_session('manual'):
            pass
    state['identity'] = ['other boot', 'reused pid']
    assert not session.active(state)


def test_stop_requests_grounded_close_without_killing_processes(owned):
    folder, state = owned
    assert control.main(['stop']) == 0
    assert (folder/'close-ui').exists()
    assert session.active(state)


def test_map_command_rejects_wrong_program_or_unadmitted_map(owned, tmp_path):
    folder, state = owned
    with pytest.raises(RuntimeError, match='尚未准入'):
        control.main(['save', str(tmp_path/'output')])
    state['program'] = 'manual'
    session.write(state)
    with pytest.raises(ValueError, match='没有地图'):
        control.main(['save', str(tmp_path/'output')])
    assert not (tmp_path/'output').exists()


def test_map_rpc_inherits_current_domain_partition_and_transport(owned, tmp_path, monkeypatch):
    folder, state = owned
    (folder/'map-admission.json').write_text('{}')
    run = Mock(return_value=Mock(returncode=0))
    monkeypatch.setattr(control.subprocess, 'run', run)
    output = tmp_path/'map with spaces'
    assert control.main(['save', str(output)]) == 0
    args = run.call_args.args[0]
    env = run.call_args.kwargs['env']
    assert args[-2:] == ['save', str(output)]
    assert env['ROS_DOMAIN_ID'] == '78' and env['GZ_PARTITION'] == state['partition']
    assert env['UAV_SIM_OPERATOR_ID'] == state['id']
    assert env['FASTDDS_DEFAULT_PROFILES_FILE'] == env['FASTRTPS_DEFAULT_PROFILES_FILE']


@pytest.mark.parametrize('age,landed,armed,expected', [(0, True, 1, True), (3, True, 1, False),
    (0, False, 1, False), (0, True, 2, False), (-1, True, 1, False)])
def test_flow_close_requires_fresh_ground_telemetry(tmp_path, age, landed, armed, expected):
    (tmp_path/'pocket-live.json').write_text(json.dumps(
        dict(system_ns=int((10-age)*1e9), landed=landed, arming=armed, ground_state_fresh=True)))
    assert session.operator_grounded(tmp_path, now_ns=10*10**9) is expected


def test_flow_close_rejects_a_new_heartbeat_with_stale_px4_samples(tmp_path):
    (tmp_path/'pocket-live.json').write_text(json.dumps(dict(
        system_ns=10*10**9, landed=True, arming=1, ground_state_fresh=False)))
    assert not session.operator_grounded(tmp_path, now_ns=10*10**9)


@pytest.mark.parametrize('name', ['OpenTX Radiomaster Pocket Joystick', 'Pocket Virtual UAV AETR'])
def test_qgc_uses_selected_device_and_preserves_calibration(tmp_path, name):
    path = tmp_path/'QGroundControl.ini'
    path.write_text('[JoystickManager]\nactiveJoystickName=wrong device\n[Joystick]\nCalibrated=true\n')
    configure_qgc_input(path, {'name': name})
    assert 'activeJoystickName='+name in path.read_text()
    assert 'Calibrated=true' in path.read_text()
