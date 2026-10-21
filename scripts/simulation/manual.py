#!/usr/bin/env python3
"""Pure manual session: choose a physical Pocket or own a virtual panel fallback."""
from __future__ import annotations

import argparse
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
from simulation.backends import ROOT, command as backend_command
from pocket_input import preflight_device, select_device


def runtime_command(identity, duration, qgc_config=None):
    args = ['--sdk-schur64', '--vio-primary', '--manual-only', '--device', identity['device'],
            '--input-profile', identity['input_profile'], '--session-duration', str(duration)]
    if qgc_config:
        args += ['--qgc-config-from', str(qgc_config)]
    return backend_command('vio-pocket', args)


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
            panel = subprocess.Popen([sys.executable, str(ROOT / 'scripts/pocket_panel.py'), '--uinput'],
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


def supervise(arguments):
    process = subprocess.Popen(arguments, cwd=ROOT, start_new_session=True)
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
    parser.add_argument('--input-source', choices=('auto', 'physical', 'virtual'), default='auto')
    parser.add_argument('--device', default='auto')
    parser.add_argument('--session-duration', type=int, default=3600)
    parser.add_argument('--qgc-config-from', type=Path)
    args = parser.parse_args(argv)
    if not 60 <= args.session_duration <= 14400:
        parser.error('session-duration must be within [60,14400] seconds')
    if args.qgc_config_from and not args.qgc_config_from.is_file():
        parser.error('QGC calibration config does not exist')
    folder = ROOT / '.cache/simulation/operator' / str(uuid.uuid4())
    with input_device(args.input_source, args.device, folder) as identity:
        print(f'手动输入已选择：{identity["name"]}，{identity["axes"]}轴／{identity["buttons"]}按钮\n'
              f'设备：{identity["device"]}\n请在 QGC 校准并启用此摇杆；启动过程不自动解锁或起飞。', flush=True)
        return supervise(runtime_command(identity, args.session_duration, args.qgc_config_from))


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (ValueError, OSError) as exc:
        print(f'手动会话启动失败：{exc}', file=sys.stderr)
        raise SystemExit(1)
