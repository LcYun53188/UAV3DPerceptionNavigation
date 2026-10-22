#!/usr/bin/env python3
"""查找当前工作区的残留仿真进程；默认预览，--apply 分阶段清理。"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import shlex
import signal
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation.backends import ROOT

SUPERVISORS = {'manual.py', 'run_px4_mtf01.py', 'run_px4_vio_sensors.py',
               'run_px4_sitl_smoke.py', 'run_fusion_switch_sitl.py', 'sim_control.py'}
NODES = {'px4', 'MicroXRCEAgent', 'rviz2', 'component_container', 'component_container_mt',
         'nvblox_node', 'ego_nvblox_planner', 'parameter_bridge', 'static_transform_publisher',
         'mission_runner', 'map_session', 'planning_sources', 'planning_context'}
ENV_KEYS = {'GZ_PARTITION', 'UAV_WORKSPACE', 'UAV_FLIGHT_EVIDENCE', 'XDG_CONFIG_HOME'}


@dataclass(frozen=True)
class Process:
    pid: int
    ppid: int
    pgid: int
    sid: int
    birth: str
    argv: tuple[str, ...]
    cwd: str
    exe: str
    env: dict[str, str]


def inspect_process(pid, proc=Path('/proc')):
    """Ignore zombies, foreign UIDs and processes which exit during inspection."""
    folder = proc/str(pid)
    try:
        if folder.stat().st_uid != os.getuid():
            return None
        fields = (folder/'stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        argv = tuple(s.decode(errors='replace') for s in (folder/'cmdline').read_bytes().split(b'\0') if s)
        env = {}
        for entry in (folder/'environ').read_bytes().split(b'\0'):
            key, sep, value = entry.partition(b'=')
            if sep and key.decode(errors='replace') in ENV_KEYS:
                env[key.decode()] = value.decode(errors='replace')
        return Process(int(pid), int(fields[1]), int(fields[2]), int(fields[3]), fields[19],
                       argv, str((folder/'cwd').readlink()), str((folder/'exe').readlink()), env)
    except (OSError, ValueError, IndexError):
        return None


def scan():
    return {p.pid: p for entry in Path('/proc').iterdir() if entry.name.isdecimal()
            if (p := inspect_process(int(entry.name))) is not None}


def under(path, root):
    return bool(path) and Path(path).is_absolute() and Path(path).resolve().is_relative_to(root.resolve())


def script_path(process, root):
    # Inspect the executed script, never arbitrary strings inside `bash -c` or
    # `python -c`, which could belong to this tool, a terminal, or the IDE.
    args = process.argv
    if len(args) < 2 or not Path(args[0]).name.startswith('python') or args[1].startswith('-'):
        return None
    path = Path(args[1])
    path = path if path.is_absolute() else Path(process.cwd)/path
    path = path.resolve()
    if path.is_relative_to(root/'scripts/simulation') or path.is_relative_to(root/'scripts/compat'):
        return path
    return None


def records(root):
    """Session manifests identify owned partitions without trusting old PIDs."""
    partitions = set()
    for path in (root/'.cache/simulation').glob('*/*/manifest.json'):
        try:
            value = json.loads(path.read_text())
            partition = value.get('partition')
            if isinstance(partition, str) and partition.startswith('uav_'):
                partitions.add(partition)
        except (OSError, ValueError, AttributeError):
            continue
    return partitions


def select(processes, root=ROOT, self_pid=None):
    """Return residual candidates and protected processes with ownership reasons."""
    partitions = records(root)
    protected = set()
    pid = os.getpid() if self_pid is None else self_pid
    while pid in processes and pid not in protected:
        protected.add(pid)
        pid = processes[pid].ppid
    for p in processes.values():
        script = script_path(p, root)
        if script and script.name in SUPERVISORS:
            if script.name != 'sim_control.py' or 'start' in p.argv:
                protected.add(p.pid)
    # Protect recorded launchers too (the old algorithm launcher is ros2 launch).
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    for name in ('.cache/simulation/operator/current.json', '.cache/sim/session.json'):
        try:
            state = json.loads((root/name).read_text())
            p = processes.get(state['pid'])
            if p and not state.get('closed') and state.get('identity') == [boot, p.birth]:
                protected.add(p.pid)
        except (OSError, ValueError, KeyError, TypeError):
            continue
    # Components start their own process groups; walk ancestry rather than
    # assuming that they remain in the launcher's process group.
    while True:
        descendants = {p.pid for p in processes.values() if p.ppid in protected}
        # Ancestors of this cleanup command are not supervision roots.
        roots = protected - ancestors(processes, os.getpid() if self_pid is None else self_pid)
        descendants = {p.pid for p in processes.values() if p.ppid in roots}
        if descendants <= protected:
            break
        protected |= descendants
    protected_partitions = {processes[pid].env.get('GZ_PARTITION') for pid in protected
                            if pid in processes}
    selected, skipped = {}, {}
    for p in processes.values():
        script = script_path(p, root)
        partition = p.env.get('GZ_PARTITION')
        workspace = under(p.cwd, root) or p.env.get('UAV_WORKSPACE') == str(root)
        isolated_qgc = under(p.env.get('XDG_CONFIG_HOME'), root/'.cache/simulation')
        executable = Path(p.exe).name
        qgc = 'qgroundcontrol' in executable.casefold() or (
            p.argv and Path(p.argv[0]).name.casefold().startswith('qgroundcontrol'))
        known = (script is not None and script.name not in {
                    'cleanup.py', 'cli.py', 'menu.py', 'control.py', 'control_ros.py'})
        known |= executable in NODES or executable.startswith('gz-sim')
        known |= bool(p.argv and Path(p.argv[0]).name == 'gz' and 'sim' in p.argv[:3])
        known |= bool(qgc and isolated_qgc)
        owned = workspace and ((partition in partitions) or
                 (partition or '').startswith('uav_') or isolated_qgc or
                 any(under(arg, root/'.cache/simulation') for arg in p.argv if arg.startswith('/')))
        # A personal QGC instance without an isolated session config is excluded.
        if not known or not owned or qgc and not isolated_qgc:
            continue
        if p.pid in protected or partition and partition in protected_partitions:
            skipped[p.pid] = '运行中会话，保留'
        else:
            selected[p.pid] = '工作区仿真进程，监督器已退出'
    return selected, skipped


def ancestors(processes, pid):
    result = set()
    while pid in processes and pid not in result:
        result.add(pid)
        pid = processes[pid].ppid
    return result


def send(process, sig):
    """pidfd binds the signal to this process even if its PID is later reused."""
    try:
        fd = os.pidfd_open(process.pid)
    except ProcessLookupError:
        return False
    try:
        current = inspect_process(process.pid)
        if current is None or current.birth != process.birth:
            return False
        signal.pidfd_send_signal(fd, sig)
        return True
    except ProcessLookupError:
        return False
    finally:
        os.close(fd)


def survivors(targets):
    return {pid: p for pid, p in targets.items()
            if (current := inspect_process(pid)) is not None and current.birth == p.birth}


def terminate(targets, timeout=5.):
    errors = []
    pending = dict(targets)
    for sig, wait in ((signal.SIGINT, timeout), (signal.SIGTERM, timeout), (signal.SIGKILL, 1.)):
        for p in pending.values():
            try:
                send(p, sig)
            except OSError as exc:
                errors.append(f'PID {p.pid}: {exc}')
        deadline = time.monotonic()+wait
        while pending and time.monotonic() < deadline:
            pending = survivors(pending)
            if pending:
                time.sleep(.1)
        if not pending:
            break
    return pending, errors


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--apply', action='store_true', help='执行清理；默认仅显示清单')
    parser.add_argument('--timeout', type=float, default=5., help='INT／TERM 各阶段等待秒数，默认 5（0.1–20）')
    args = parser.parse_args(argv)
    if not math.isfinite(args.timeout) or not .1 <= args.timeout <= 20:
        parser.error('--timeout 必须在 0.1–20 秒之间')
    processes = scan()
    selected, skipped = select(processes)
    for title, entries in (('残留进程', selected), ('跳过的运行中进程', skipped)):
        print(f'{title}：{len(entries)}')
        for pid, reason in sorted(entries.items()):
            p = processes[pid]
            print(f'  PID={pid} PPID={p.ppid} {reason}\n    {shlex.join(p.argv)}')
    if not selected:
        print('没有可清理的残留仿真进程。')
        return 0
    if not args.apply:
        print('仅预览；执行清理：./scripts/sim.sh cleanup --apply')
        return 0
    if not hasattr(os, 'pidfd_open') or not hasattr(signal, 'pidfd_send_signal'):
        raise RuntimeError('清理需要支持 pidfd 的 Linux／Python，未发送信号')
    pending, errors = terminate({pid: processes[pid] for pid in selected}, args.timeout)
    for error in errors:
        print(error, file=sys.stderr)
    # Rescan so newly exposed or reparented components are reported as well.
    remaining, _ = select(scan())
    print(f'清理后残留：{len(remaining)}；本次原进程仍存活：{len(pending)}')
    return 1 if pending or remaining or errors else 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, RuntimeError) as exc:
        sys.exit(f'清理失败：{exc}')
