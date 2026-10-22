"""Residual selection, active-session exclusion and real bounded process cleanup."""
from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
from unittest.mock import Mock

import pytest

from simulation import cleanup


def process(root, pid=100, *, ppid=1, exe='MicroXRCEAgent', args=None, partition='uav_test'):
    return cleanup.Process(pid, ppid, pid, pid, '12345', tuple(args or [str(root/'.deps'/exe)]),
                           str(root), '/usr/bin/'+exe, {'GZ_PARTITION': partition})


def test_orphan_simulator_selected_but_other_workspace_and_personal_qgc_are_not(tmp_path):
    a = process(tmp_path)
    b = replace(process(tmp_path, 101), cwd='/elsewhere')
    c = process(tmp_path, 102, exe='QGroundControl')
    selected, _ = cleanup.select({p.pid:p for p in (a,b,c)}, tmp_path, self_pid=999)
    assert set(selected) == {100}


def test_active_supervisor_protects_separate_groups_and_reparented_components(tmp_path):
    supervisor = process(tmp_path, 90, exe='python3', args=[sys.executable,
        str(tmp_path/'scripts/simulation/manual.py')])
    a = process(tmp_path, ppid=90)
    b = process(tmp_path, 101)  # A component reparented to the init/reaper process.
    selected, skipped = cleanup.select({p.pid:p for p in (supervisor,a,b)}, tmp_path, self_pid=999)
    assert not selected and set(skipped) == {90, 100, 101}


def test_terminal_with_old_partition_does_not_protect_orphans(tmp_path):
    parent = process(tmp_path, 90, exe='bash')
    caller = process(tmp_path, 91, ppid=90, exe='python3', args=[sys.executable,
        str(tmp_path/'scripts/simulation/cleanup.py')])
    orphan = process(tmp_path)
    selected, _ = cleanup.select({p.pid:p for p in (parent,caller,orphan)}, tmp_path, self_pid=91)
    assert set(selected) == {100}


def test_ide_command_strings_do_not_count_as_executed_simulation_scripts(tmp_path):
    command = process(tmp_path, exe='bash', args=['bash','-c',
        'python3 '+str(tmp_path/'scripts/simulation/manual.py')])
    selected, _ = cleanup.select({100:command}, tmp_path, self_pid=999)
    assert not selected


def test_installed_python_ros_node_and_isolated_qgc_are_selected(tmp_path):
    node = process(tmp_path, exe='python3', args=[sys.executable,
        str(tmp_path/'.deps/mission-install/uav_nav_sim/lib/uav_nav_sim/map_session')])
    qgc = process(tmp_path, 101, exe='QGroundControl')
    qgc.env['XDG_CONFIG_HOME'] = str(tmp_path/'.cache/simulation/vio-sensors/owned/qgc-config')
    selected, _ = cleanup.select({100:node,101:qgc}, tmp_path, self_pid=999)
    assert set(selected) == {100,101}


def test_gazebo_ruby_launcher_is_detected_without_matching_shell_strings(tmp_path):
    gazebo = process(tmp_path, exe='ruby3.2', args=['/usr/bin/ruby','/usr/bin/gz','sim','-r','-s'])
    selected, _ = cleanup.select({100:gazebo}, tmp_path, self_pid=999)
    assert set(selected) == {100}


def test_recorded_algorithm_owner_uses_birth_identity_and_not_just_pid(tmp_path):
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    state = tmp_path/'.cache/sim/session.json'
    state.parent.mkdir(parents=True)
    p = process(tmp_path)
    state.write_text(json.dumps(dict(pid=100,identity=[boot,p.birth])))
    assert not cleanup.select({100:p}, tmp_path, self_pid=999)[0]
    state.write_text(json.dumps(dict(pid=100,identity=[boot,'old birth'])))
    assert set(cleanup.select({100:p}, tmp_path, self_pid=999)[0]) == {100}


def test_preview_never_sends_a_signal(tmp_path, monkeypatch, capsys):
    p = process(tmp_path)
    monkeypatch.setattr(cleanup, 'scan', lambda: {100:p})
    monkeypatch.setattr(cleanup, 'select', lambda _: ({100:'orphan'},{}))
    terminate = Mock()
    monkeypatch.setattr(cleanup, 'terminate', terminate)
    assert cleanup.main([]) == 0
    terminate.assert_not_called()
    assert '--apply' in capsys.readouterr().out


@pytest.mark.skipif(not hasattr(os,'pidfd_open'), reason='Linux pidfd required')
def test_pid_reuse_check_and_kill_escalation_only_affect_owned_test_child(tmp_path):
    child = subprocess.Popen([sys.executable,'-c',
        'import signal,time; signal.signal(signal.SIGINT,signal.SIG_IGN); '
        'signal.signal(signal.SIGTERM,signal.SIG_IGN); print("ready",flush=True); time.sleep(30)'],
        cwd=tmp_path, stdout=subprocess.PIPE, text=True, start_new_session=True)
    try:
        assert child.stdout.readline().strip() == 'ready'
        p = cleanup.inspect_process(child.pid)
        assert p is not None
        assert not cleanup.send(replace(p,birth='reused pid'), signal.SIGKILL)
        assert child.poll() is None
        pending, errors = cleanup.terminate({p.pid:p}, timeout=.1)
        assert not pending and not errors
        assert child.wait(timeout=2) == -signal.SIGKILL
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=2)
        child.stdout.close()
