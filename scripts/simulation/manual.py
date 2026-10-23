#!/usr/bin/env python3
"""Shared HID/panel ownership for the manual flow and full project programs."""
from __future__ import annotations

import argparse
import configparser
from contextlib import contextmanager
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from support.paths import enable_imports
enable_imports()
from simulation.backends import ROOT, command as backend_command
from pocket_input import preflight_device, select_device


def runtime_command(identity, duration, qgc_config=None, program='manual', person_model=None, color_map=False, person_scene='none'):
    if program not in ('manual', 'full'):
        raise ValueError('Unknown simulation program: ' + program)
    args = (['--manual-only'] if program == 'manual' else ['--sdk-schur64', '--vio-primary'])
    args += ['--device', identity['device'],
            '--input-profile', identity['input_profile'], '--session-duration', str(duration)]
    if qgc_config:
        args += ['--qgc-config-from', str(qgc_config)]
    if person_model is not None:
        if program != 'full':
            raise ValueError('Person mapping requires the full VIO/map program')
        args += ['--people', '--person-model', str(person_model)]
    if color_map:
        if program != 'full':
            raise ValueError('Color mapping requires the full VIO/map program')
        args += ['--color-map']
    if person_scene != 'none':
        if program != 'full' or person_model is None or person_scene != 'standing':
            raise ValueError('Standing-person scene requires full person perception')
        args += ['--person-scene', person_scene]
    return backend_command('flow-pocket' if program == 'manual' else 'vio-pocket', args)


def stop_panel(panel):
    if panel.poll() is None:
        panel.terminate()
        try:
            panel.wait(timeout=5)
        except subprocess.TimeoutExpired:
            panel.kill()
            panel.wait(timeout=5)


@contextmanager
def input_device(source, device, folder):
    identity = select_device(source, device)
    panel, logfile = None, None
    try:
        if identity is None:
            # Only a missing virtual input can arrive here. A physical-device
            # error propagates before any GUI or simulation process is started.
            folder.mkdir(parents=True, exist_ok=True)
            path = folder / 'pocket-panel.log'
            logfile = path.open('w')
            panel = subprocess.Popen([sys.executable, str(ROOT / 'scripts/simulation/pocket/pocket_panel.py'), '--uinput'],
                                     cwd=ROOT, stdout=logfile, stderr=subprocess.STDOUT, start_new_session=True)
            print(f'需要虚拟 Pocket，正在启动虚拟面板；日志：{path}', flush=True)
            until = time.monotonic() + 12
            while time.monotonic() < until:
                if panel.poll() is not None:
                    raise ValueError(f'虚拟面板启动失败（exit={panel.returncode}），请查看 {path}')
                for line in path.read_text(errors='replace').splitlines():
                    try:
                        event = json.loads(line)
                    except ValueError:
                        continue
                    if isinstance(event, dict) and event.get('device') and event.get('profile') == 'uav-aetr-v1':
                        identity = dict(preflight_device(event['device'], 'uav-aetr-v1'), input_source='virtual')
                        break
                if identity:
                    break
                time.sleep(.05)
            if identity is None:
                raise ValueError(f'虚拟摇杆创建超时，检查 /dev/uinput 权限与面板提示；日志：{path}')
        folder.mkdir(parents=True, exist_ok=True)
        (folder / 'input.json').write_text(json.dumps(dict(identity, created_panel=panel is not None), ensure_ascii=False, indent=2) + '\n')
        yield identity
    finally:
        if panel is not None:
            stop_panel(panel)
        if logfile is not None:
            logfile.close()


def supervise(arguments, environment=None):
    process = subprocess.Popen(arguments, cwd=ROOT, start_new_session=True, env=environment)
    previous = {}

    def request_stop(_signum, _frame):
        if process.poll() is None:
            # Let the original supervisor run its landed/disarmed cleanup.
            os.killpg(process.pid, signal.SIGINT)

    try:
        for sig in (signal.SIGINT, signal.SIGTERM):
            previous[sig] = signal.signal(sig, request_stop)
        return process.wait()
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)
        if process.poll() is None:
            request_stop(signal.SIGINT, None)
            process.wait()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--program', choices=('manual', 'full'), default='manual')
    parser.add_argument('--input-source', choices=('physical', 'virtual'), default='physical')
    parser.add_argument('--device', default='auto')
    parser.add_argument('--session-duration', type=int, default=3600)
    parser.add_argument('--qgc-config-from', type=Path)
    parser.add_argument('--people', action='store_true')
    parser.add_argument('--person-model', type=Path, default=ROOT/'.cache/models/person/yolox_tiny.onnx')
    parser.add_argument('--person-scene', choices=['none', 'standing'], default='none')
    parser.add_argument('--color-map', action='store_true')
    args = parser.parse_args(argv)
    if args.person_scene != 'none' and not (args.people and args.program == 'full'):
        parser.error('--person-scene requires the full program and --people')
    if args.person_scene != 'none':
        subprocess.run(['env', '-u', 'PYTHONPATH', '-u', 'AMENT_PREFIX_PATH', '-u', 'COLCON_PREFIX_PATH',
                        '-u', 'CMAKE_PREFIX_PATH', '-u', 'LD_LIBRARY_PATH',
                        str(ROOT/'scripts/with_venv.sh'), 'python', '-m', 'uav_target_perception.gazebo',
                        'check', str(ROOT/'.cache/models/gazebo/standing_person/v3')], check=True)
    if not 60 <= args.session_duration <= 14400:
        parser.error('session-duration must be within [60,14400] seconds')
    if args.qgc_config_from and not args.qgc_config_from.is_file():
        parser.error('QGC calibration config does not exist')
    if args.color_map and args.program != 'full':
        parser.error('--color-map requires the full program')
    if args.people:
        if args.program != 'full':
            parser.error('--people requires the full program')
        # Fail before opening the panel or touching the running simulation.
        subprocess.run(['env', '-u', 'PYTHONPATH', '-u', 'AMENT_PREFIX_PATH', '-u', 'COLCON_PREFIX_PATH',
                        '-u', 'CMAKE_PREFIX_PATH', '-u', 'LD_LIBRARY_PATH',
                        str(ROOT/'scripts/with_venv.sh'), 'python', '-c',
                        'from uav_target_perception.node import PersonMapper; from uav_target_perception.detector import PersonDetector; import sys; PersonDetector(sys.argv[1])',
                        str(args.person_model.expanduser().resolve())], check=True)
    folder = ROOT / '.cache/simulation/operator' / str(uuid.uuid4())
    from simulation.session import own_session
    with own_session(args.program) as environment, input_device(args.input_source, args.device, folder) as identity:
        from qgc_calibration import cached, remember
        calibration = args.qgc_config_from or cached(identity)
        if calibration:
            print(f'使用已保存的 QGC 校准：{calibration}', flush=True)
        print(f'手动输入已选择：{identity["name"]}，{identity["axes"]}轴／{identity["buttons"]}按钮\n'
              f'设备：{identity["device"]}\n请确认 QGC 启用此摇杆并使用 Fly 页面；'
              '校准仅需首次完成，启动过程不自动解锁或起飞。', flush=True)
        try:
            return supervise(runtime_command(identity, args.session_duration, calibration, args.program,
                             args.person_model.expanduser().resolve() if args.people else None,args.color_map,args.person_scene), environment)
        finally:
            from simulation.session import read
            state = read(False)
            if state and state.get('id') == environment['UAV_SIM_OPERATOR_ID'] and state.get('runtime'):
                path = Path(state['runtime'])/'qgc-config/QGroundControl/QGroundControl.ini'
                try:
                    saved = remember(path, identity)
                    if saved:
                        print(f'已保存 QGC 校准，后续同设备会话自动复用：{saved}', flush=True)
                except (OSError, ValueError, configparser.Error) as exc:
                    print(f'QGC 校准保存失败：{exc}', file=sys.stderr)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (ValueError, OSError, RuntimeError) as exc:
        print(f'手动会话启动失败：{exc}', file=sys.stderr)
        raise SystemExit(1)
