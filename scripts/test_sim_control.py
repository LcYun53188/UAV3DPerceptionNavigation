"""CLI policy tests; no Gazebo, ROS services or signals to real processes."""
import importlib.util
import json
from pathlib import Path
from unittest.mock import Mock

import pytest

spec = importlib.util.spec_from_file_location('sim_control', Path(__file__).with_name('sim_control.py'))
control = importlib.util.module_from_spec(spec)
spec.loader.exec_module(control)


@pytest.mark.parametrize('layout,extent', [('lab', '5.0'), ('expanded', '10.5')])
def test_world_and_extent_match(layout, extent):
    args = control.parser().parse_args(['start', '--layout', layout, '--view', 'none'])
    cmd = control.launch_command(args)
    assert f'map_extent:={extent}' in cmd
    assert any(word.endswith(f'uav_ego_{layout}.sdf') for word in cmd)
    assert 'gui:=false' in cmd and 'launch_rviz:=false' in cmd


def test_pid_reuse_does_not_send_signal(monkeypatch):
    monkeypatch.setattr(control, 'identity', lambda pid: ['new-boot', '456'])
    kill = Mock()
    monkeypatch.setattr(control.os, 'kill', kill)
    control.stop(dict(pid=123, identity=['old-boot', '123']))
    kill.assert_not_called()


def test_saved_environment_wins_over_new_terminal(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '0')
    monkeypatch.setenv('GZ_PARTITION', 'wrong')
    env = control.environment(dict(domain=68, partition='saved'))
    assert env['ROS_DOMAIN_ID'] == '68' and env['GZ_PARTITION'] == 'saved'


@pytest.mark.parametrize('command,mode', [('init', 'localization'), ('explore', 'localization'), ('load', 'mapping')])
def test_wrong_mode_cannot_mutate_simulation(tmp_path, monkeypatch, command, mode):
    monkeypatch.setattr(control, 'CACHE', tmp_path)
    session = dict(mode=mode)
    monkeypatch.setattr(control, 'read_session', lambda *a: session)
    run = Mock()
    monkeypatch.setattr(control, 'run', run)
    monkeypatch.setattr(control.sys, 'argv', ['sim', command, *(['map'] if command == 'load' else [])])
    with pytest.raises(RuntimeError):
        control.main()
    run.assert_not_called()


def test_stale_state_requires_start(tmp_path, monkeypatch):
    state = tmp_path / 'session.json'
    state.write_text(json.dumps(dict(pid=123, identity=['boot', '1'])))
    monkeypatch.setattr(control, 'STATE', state)
    monkeypatch.setattr(control, 'identity', lambda pid: None)
    with pytest.raises(RuntimeError, match='没有运行'):
        control.read_session()


def test_operation_lock_rejects_overlap(tmp_path):
    with control.lock(tmp_path / 'lock'):
        with pytest.raises(RuntimeError, match='已有仿真或控制操作'):
            with control.lock(tmp_path / 'lock'):
                pass


@pytest.mark.parametrize('value', ['nan', 'inf', '-1', '0'])
def test_invalid_goal_height_rejected(value):
    with pytest.raises(SystemExit):
        control.parser().parse_args(['start', '--goal-height', value])
