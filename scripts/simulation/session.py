"""Bind terminal operations to one owned simulation and its process lifetime."""
from contextlib import contextmanager
import fcntl
import json
import os
from pathlib import Path
import time
import uuid

from simulation.backends import ROOT

CACHE = ROOT / '.cache/simulation/operator'
STATE = CACHE / 'current.json'


def identity(pid):
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return [Path('/proc/sys/kernel/random/boot_id').read_text().strip(), fields[19]]
    except (OSError, ValueError):
        return None


def active(state):
    return bool(state and state.get('identity') and not state.get('closed')
                and identity(state['pid']) == state['identity'])


def read(required=True):
    state = json.loads(STATE.read_text()) if STATE.exists() else None
    if required and not active(state):
        raise RuntimeError('没有运行中的仿真；先执行 sim.sh run manual 或 sim.sh run full')
    return state


def write(state):
    pending = STATE.with_suffix('.pending.json')
    pending.write_text(json.dumps(state, ensure_ascii=False, indent=2) + '\n')
    os.replace(pending, STATE)


@contextmanager
def own_session(program):
    CACHE.mkdir(parents=True, exist_ok=True)
    with (CACHE / 'owner.lock').open('a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('已有仿真正在运行；用 sim.sh status 查看，落地上锁后 sim.sh stop') from None
        if active(read(False)):
            raise RuntimeError('已有运行中的仿真会话')
        token = str(uuid.uuid4())
        write(dict(schema=1, id=token, program=program, pid=os.getpid(),
                   identity=identity(os.getpid()), started_ns=time.time_ns(), closed=False))
        try:
            yield dict(os.environ, UAV_SIM_OPERATOR_ID=token)
        finally:
            state = read(False)
            if state and state.get('id') == token:
                state.update(closed=True, closed_ns=time.time_ns())
                write(state)


def register_runtime(folder, manifest):
    """Called by the runner; diagnostic invocations have no public-session token."""
    token = os.environ.get('UAV_SIM_OPERATOR_ID')
    if not token:
        return
    state = read()
    if state['id'] != token:
        raise RuntimeError('Simulation operator session changed during startup')
    folder = Path(folder).resolve()
    if not folder.is_relative_to(ROOT / '.cache/simulation'):
        raise ValueError('Runtime directory must belong to this workspace')
    state.update(runtime=str(folder), domain=manifest['domain'], partition=manifest['partition'],
                 runner_pid=os.getpid(), runner_identity=identity(os.getpid()))
    write(state)
    print(f'当前会话：{folder}\n另一终端：./scripts/sim.sh status；./scripts/sim.sh stop', flush=True)


def runtime(state):
    if (not active(state) or not state.get('runner_identity')
            or identity(state['runner_pid']) != state['runner_identity']):
        raise RuntimeError('仿真尚未启动完成或运行器已退出；用 sim.sh status 查看')
    folder = Path(state['runtime']).resolve()
    if not folder.is_relative_to(ROOT / '.cache/simulation'):
        raise ValueError('会话目录不属于当前工作区')
    manifest = json.loads((folder / 'manifest.json').read_text())
    if manifest['domain'] != state['domain'] or manifest['partition'] != state['partition']:
        raise RuntimeError('会话通信域与运行记录不一致')
    return folder


def operator_grounded(folder, now_ns=None):
    """Fresh operator telemetry is required before an ordinary flow shutdown."""
    path = Path(folder) / 'pocket-live.json'
    if not path.is_file():
        return False
    try:
        live = json.loads(path.read_text())
        age = ((time.time_ns() if now_ns is None else now_ns) - live['system_ns']) / 1e9
        return (0 <= age <= 2 and live.get('ground_state_fresh') is True
                and live.get('landed') is True and live.get('arming') == 1)
    except (OSError, ValueError, KeyError):
        return False


def report_rejected_arm(folder, last_seen):
    """Surface a real switch rejection once; this function never sends commands."""
    try:
        live = json.loads((Path(folder)/'pocket-live.json').read_text())
        request = live.get('last_operator_request') or {}
        if request.get('command') != 'ARM' or request.get('mono') == last_seen:
            return last_seen
        if request.get('accepted') is False:
            reason = request.get('reason')
            throttle = (request.get('manual_control') or {}).get('throttle')
            hint = {
                'ARM_THROTTLE_NOT_LOW': '解锁要求 ≤ -0.8，最低油门应接近 -1',
                'TASK_PREPARATION_REQUIRED': '任务尚未准备通过；SB 任务档下重新切入 SC 起飞准备档，确认 TASK_PREPARED 后再操作 SA',
                'SOURCE_PREFLIGHT_PENDING': '定位源预检尚未通过；等待准入后重新操作准备档与 SA',
            }.get(reason)
            detail = f'（{hint}）' if hint else ''
            print(f'SA 解锁被拒绝：{reason}；PX4 油门={throttle}{detail}。', flush=True)
        return request.get('mono')
    except (OSError, ValueError):
        return last_seen
