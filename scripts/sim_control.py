#!/usr/bin/env python3
"""Manage one Gazebo EGO/nvblox session in this workspace (Linux)."""
import argparse
from contextlib import contextmanager
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
CACHE = ROOT / '.cache/sim'
STATE = CACHE / 'session.json'
WRAPPER = ROOT / 'scripts/with_venv.sh'


def identity(pid):
    """Boot ID + process birth time protect stop against PID reuse."""
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        if fields[0] == 'Z':
            return None
        return [Path('/proc/sys/kernel/random/boot_id').read_text().strip(), fields[19]]
    except (FileNotFoundError, ProcessLookupError):
        return None


def alive(session):
    return bool(session and session.get('identity') and
                identity(session['pid']) == session['identity'])


def read_session(required=True):
    session = json.loads(STATE.read_text()) if STATE.exists() else None
    if required and not alive(session):
        raise RuntimeError('没有运行中的受管仿真。先执行 scripts/sim.sh start；手动 launch 不由此脚本接管。')
    return session


@contextmanager
def lock(path):
    with open(path, 'a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError(f'已有仿真或控制操作正在运行：{path}') from None
        yield handle


def environment(session):
    return {**os.environ, 'ROS_DOMAIN_ID': str(session['domain']),
            'GZ_PARTITION': session['partition']}


def run(session, arguments, timeout=None):
    subprocess.run([str(WRAPPER), *arguments], cwd=ROOT, env=environment(session),
                   check=True, timeout=timeout)


def positive(value):
    number = float(value)
    if not math.isfinite(number) or number <= 0:
        raise argparse.ArgumentTypeError('必须是有限正数')
    return number


def parser():
    p = argparse.ArgumentParser(description=__doc__)
    sub = p.add_subparsers(dest='command', required=True)
    start = sub.add_parser('start', help='启动导航仿真；默认前台等待，Ctrl+C 退出')
    start.add_argument('--layout', choices=['lab', 'expanded'], default='expanded')
    start.add_argument('--mode', choices=['mapping', 'localization'], default='mapping')
    start.add_argument('--view', choices=['both', 'rviz', 'none'], default='both')
    start.add_argument('--domain', type=int, choices=range(233), default=68, metavar='0..232')
    start.add_argument('--partition', default='uav_ego_lab')
    start.add_argument('--goal-height', type=positive, default=1.2)
    start.add_argument('--background', action='store_true', help='后台运行；用 stop 退出')
    for name, help_text in [('stop', '退出整套受管仿真'), ('status', '进程、导航和地图状态'),
                            ('init', '离线放置相机，初始化起点；仅 mapping'),
                            ('cancel', '取消目标与自动探索，仿真继续运行'),
                            ('explore', '启动自主边界探索；无需手动目标，仅 mapping')]:
        sub.add_parser(name, help=help_text)
    sub.add_parser('doctor', help='S0 离线依赖审计；不判定 topic/TF/clock 就绪')
    logs = sub.add_parser('logs', help='查看最近日志')
    logs.add_argument('--follow', action='store_true')
    for name in ['survey', 'save', 'load']:
        item = sub.add_parser(name, help={'survey': '完整离线扫描并保存', 'save': '保存当前地图',
                                         'load': '加载静态地图；仅 localization'}[name])
        item.add_argument('directory', type=lambda value: Path(value).expanduser().resolve())
    mesh = sub.add_parser('export-mesh', help='导出带地图顶点颜色的三维 PLY 模型')
    mesh.add_argument('filename', type=lambda value: Path(value).expanduser().resolve())
    goal = sub.add_parser('goal', help='发布 map 坐标系 XYZ 目标（米）')
    goal.add_argument('xyz', type=float, nargs=3)
    return p


def launch_command(args):
    world = ROOT / f'src/uav_bringup/gazebo/worlds/uav_ego_{args.layout}.sdf'
    return [str(WRAPPER), 'ros2', 'launch', 'uav_bringup', 'uav_ego_nvblox.launch.py',
            f'mode:={args.mode}', f'world:={world}',
            f'map_extent:={10.5 if args.layout == "expanded" else 5.0}',
            f'gui:={str(args.view == "both").lower()}',
            f'launch_rviz:={str(args.view != "none").lower()}',
            f'goal_height:={args.goal_height}']


def unmanaged_launches():
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            argv = (entry / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        if b'launch' in argv and b'uav_bringup' in argv and any(
                name in argv for name in [b'uav_ego_nvblox.launch.py', b'gazebo_harmonic_nav.launch.py']):
            return entry.name
    return None


def start(args):
    process = None
    with lock(CACHE / 'control.lock'), lock(Path(f'/tmp/uav_nav_gazebo_harmonic_{os.getuid()}.lock')) as sim_lock:
        if alive(read_session(False)):
            raise RuntimeError('本工作区仿真已运行；先 stop 再切换模式或场景。')
        existing = unmanaged_launches()
        if existing:
            raise RuntimeError(f'检测到已有 UAV launch（PID {existing}），请先在原终端退出。')
        if not (ROOT / 'install_uav/setup.bash').is_file():
            raise RuntimeError('缺少 install_uav；请先执行 scripts/build_algorithm_sim.sh。')
        session = dict(layout=args.layout, mode=args.mode, domain=args.domain,
                       partition=args.partition, view=args.view)
        logfile = CACHE / f'launch-{time.time_ns()}.log'
        with logfile.open('w') as output:
            process = subprocess.Popen(launch_command(args), cwd=ROOT, env=environment(session),
                                       stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                       start_new_session=True, pass_fds=(sim_lock.fileno(),))
        session.update(pid=process.pid, identity=identity(process.pid), log=str(logfile))
        try:
            STATE.write_text(json.dumps(session, indent=2) + '\n')
            # The launch process inherits the global lock, including in background mode.
            time.sleep(1)
        except (OSError, KeyboardInterrupt):
            stop(session)
            raise
        if process.poll() is not None:
            raise RuntimeError(f'启动失败，查看 {logfile}')
    print(f'已启动 PID={process.pid}，layout={args.layout}，mode={args.mode}，ROS_DOMAIN_ID={args.domain}', flush=True)
    print(f'日志：{logfile}\n另一个终端运行 scripts/sim.sh status；建图模式执行 init，加载模式执行 load。', flush=True)
    if not args.background:
        print('此终端保持运行；Ctrl+C 退出整套仿真。日志用 scripts/sim.sh logs --follow 查看。', flush=True)
        try:
            code = process.wait()
            if code:
                raise RuntimeError(f'launch 已退出（{code}），查看 {logfile}')
        except KeyboardInterrupt:
            stop(session)


def stop(session):
    if not alive(session):
        print('受管仿真已停止。')
        return
    os.kill(session['pid'], signal.SIGINT)
    deadline = time.monotonic() + 30
    while alive(session) and time.monotonic() < deadline:
        time.sleep(.2)
    if alive(session):
        raise RuntimeError(f'launch 尚未退出，保留会话记录；检查日志：{session["log"]}')
    print('受管 launch 已退出。日志和地图保留。')


def main():
    args = parser().parse_args()
    CACHE.mkdir(parents=True, exist_ok=True)
    if args.command == 'start':
        start(args)
        return
    if args.command == 'doctor':
        subprocess.run([str(WRAPPER), 'python', 'scripts/check_sim_environment.py'],
                       cwd=ROOT, check=True)
        return
    session = read_session(False)
    if args.command == 'stop':
        with lock(CACHE / 'control.lock'):
            stop(read_session(False))
        return
    if args.command == 'logs':
        if session is None:
            raise RuntimeError('尚无受管仿真日志。')
        subprocess.run(['tail', '-n', '80', *(['-f'] if args.follow else []), session['log']], check=True)
        return
    if args.command == 'status':
        print(json.dumps(session, indent=2, ensure_ascii=False), flush=True)
        if not alive(session):
            print('状态：未运行（记录若存在，为上次会话）。')
            return
        run(session, ['python', 'scripts/sim_ros_control.py', 'status'], timeout=15)
        return
    session = read_session()
    with lock(CACHE / 'operation.lock'):
        if args.command in ['init', 'survey', 'save', 'explore'] and session['mode'] != 'mapping':
            raise RuntimeError('该操作要求 mapping；先 stop，再 start --mode mapping。')
        if args.command == 'load' and session['mode'] != 'localization':
            raise RuntimeError('加载要求 localization；先 stop，再 start --mode localization。')
        if args.command in ['init', 'survey']:
            extra = ['--local-only'] if args.command == 'init' else ['--output', str(args.directory)]
            run(session, ['python', 'scripts/survey_gazebo_map.py', '--layout', session['layout'], *extra])
        elif args.command == 'export-mesh':
            if args.filename.suffix != '.ply' or args.filename.exists():
                raise RuntimeError('请使用尚不存在的 .ply 文件路径。')
            run(session, ['python', 'scripts/sim_ros_control.py', 'cancel'], timeout=25)
            run(session, ['python', 'scripts/sim_ros_control.py', 'export-mesh',
                          '--output', str(args.filename)], timeout=100)
        elif args.command in ['save', 'load']:
            if args.command == 'save' and args.directory.exists():
                raise RuntimeError('保存目录已存在；请使用新目录。')
            if args.command == 'load' and not all((args.directory / name).is_file() for name in ['manifest.json', 'static_map.nvblx']):
                raise RuntimeError('地图目录缺少 manifest.json 或 static_map.nvblx。')
            run(session, ['python', 'scripts/sim_ros_control.py', 'cancel'], timeout=25)
            run(session, ['ros2', 'run', 'uav_nav_sim', 'map_bundle', args.command, str(args.directory)], timeout=100)
            if args.command == 'load':
                run(session, ['python', 'scripts/sim_ros_control.py', 'ready'], timeout=35)
        else:
            extra = [str(v) for v in args.xyz] if args.command == 'goal' else []
            run(session, ['python', 'scripts/sim_ros_control.py', args.command, *extra], timeout=25)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('\n操作已中断。用 status 检查状态，必要时 cancel 或 stop。', file=sys.stderr)
        sys.exit(130)
    except (RuntimeError, OSError, ValueError, subprocess.SubprocessError) as exc:
        print(f'错误：{exc}', file=sys.stderr)
        sys.exit(1)
