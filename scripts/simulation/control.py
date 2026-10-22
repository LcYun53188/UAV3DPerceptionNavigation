"""Operate the current public simulation from a second terminal."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import subprocess

from simulation.backends import ROOT, ENVIRONMENT
from simulation import session


def main(argv=None):
    parser = argparse.ArgumentParser(prog='./scripts/sim.sh')
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('status', help='显示当前程序、定位和遥控器状态')
    sub.add_parser('stop', help='请求落地上锁后关闭；不发送解锁、起飞或降落指令')
    logs = sub.add_parser('logs', help='查看当前运行日志')
    logs.add_argument('--follow', action='store_true')
    for action in ('save', 'export-mesh'):
        item = sub.add_parser(action, help='落地上锁后保存地图包／导出 PLY')
        item.add_argument('output', type=lambda value: Path(value).expanduser().resolve())
    args = parser.parse_args(argv)
    state = session.read(required=args.action != 'status')
    if args.action == 'status':
        report = dict(state or {}, active=session.active(state))
        if state and state.get('runtime'):
            folder = Path(state['runtime'])
            for name in ('live-session', 'pocket-live', 'live-heartbeat', 'result'):
                path = folder / (name + '.json')
                if path.is_file():
                    report[name] = json.loads(path.read_text())
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0
    folder = session.runtime(state)
    if args.action == 'stop':
        (folder / 'close-ui').touch()
        print('已请求关闭；运行器在确认落地上锁后清理。飞行中请先用遥控器降落。')
        return 0
    if args.action == 'logs':
        paths = sorted(folder.glob('*.log'))
        if not paths:
            raise RuntimeError('日志尚未生成；请稍后重试')
        return subprocess.run(['tail', '-n', '30', *(['-f'] if args.follow else []),
                               *map(str, paths)], check=False).returncode
    if state['program'] != 'full':
        raise ValueError('纯光流手动程序没有地图；地图操作需要 sim.sh run full')
    if not (folder / 'map-admission.json').is_file():
        raise RuntimeError('完整项目的定位／地图尚未准入；用 status 和 logs 检查')
    if args.output.exists():
        raise ValueError('输出已存在，请选择新路径：' + str(args.output))
    if args.action == 'export-mesh' and args.output.suffix.casefold() != '.ply':
        raise ValueError('模型导出路径需要 .ply 后缀')
    with (session.CACHE / 'operation.lock').open('a') as handle:
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('已有地图操作正在执行') from None
        env = dict(os.environ, ROS_DOMAIN_ID=str(state['domain']), ROS_LOCALHOST_ONLY='1',
                   GZ_PARTITION=state['partition'], UAV_SIM_OPERATOR_ID=state['id'],
                   FASTRTPS_DEFAULT_PROFILES_FILE=str(ROOT/'src/uav_bringup/config/fastdds_map_service.xml'),
                   FASTDDS_DEFAULT_PROFILES_FILE=str(ROOT/'src/uav_bringup/config/fastdds_map_service.xml'))
        return subprocess.run([str(ENVIRONMENT), 'vio', 'python',
            str(ROOT/'scripts/simulation/control_ros.py'), args.action, str(args.output)],
            cwd=ROOT, env=env, check=False).returncode
