#!/usr/bin/env python3
"""交互仿真菜单；配置与启动策略由 profiles 和 backends 统一维护。"""
from __future__ import annotations

import argparse
from pathlib import Path
import shlex
import sys

# Also support direct execution from another working directory.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from support.paths import enable_imports
enable_imports()
from simulation.profiles import (ROOT, SIM, PROFILES, BY_KEY, validate_config,
                                 command, path_arg, read_config, save_config)
from simulation.backends import execute


def launch(config):
    config = validate_config(config)
    o = config['options']
    if o.get('qgc_config') and not Path(path_arg(o['qgc_config'])).is_file():
        raise ValueError('QGC 校准文件不存在：' + path_arg(o['qgc_config']))
    execute(command(config))


def choose(label, choices, default):
    print('\n' + label)
    for index, (value, title) in enumerate(choices, 1):
        print(f'  {index}. {title}' + ('（默认）' if value == default else ''))
    while True:
        answer = input('输入编号，回车用默认值：').strip()
        if not answer:
            return default
        if answer.isdecimal() and 1 <= int(answer) <= len(choices):
            return choices[int(answer) - 1][0]
        print('请输入有效编号。')


def configure(profile, previous=None):
    print('\n' + profile.title + '\n' + profile.note)
    options = {}
    for item in profile.options:
        default = (previous or {}).get(item.key, item.default)
        if item.choices:
            options[item.key] = choose(item.label, item.choices, default)
            continue
        while True:
            answer = input(f'{item.label} [{default}]：').strip()
            try:
                if not answer:
                    value = default
                elif item.limits:
                    value = int(answer) if type(item.default) is int else float(answer)
                else:
                    value = answer
                options[item.key] = item.validate(value)
                break
            except ValueError as exc:
                print(f'输入无效：{exc}')
    return validate_config(dict(schema=1, profile=profile.key, options=options))


def preview(config):
    print('\n工作目录：' + str(ROOT))
    print('定位约束：全局无 GNSS（关闭卫星数据源与 GNSS 融合）。')
    print('启动命令：\n' + shlex.join(command(config)), flush=True)
    print('默认使用实体 HID；模拟遥控器须显式选择。请在 QGC 校准、启用摇杆并使用 Fly 页面。')
    print('启动保持未解锁，由操作员操纵；落地上锁后用另一终端 sim.sh stop 正常关闭。')
    if config['profile'] == 'full':
        print('另一终端：./scripts/sim.sh status；./scripts/sim.sh save .cache/maps/run1')
        print('地图保存和模型导出要求飞机落地且上锁。')
    else:
        print('纯手动会话拒绝 PREPARE／RUN 等自动任务意图。')


MAIN_ENTRIES = tuple((p.key, p.title) for p in PROFILES)


def show_tree():
    print('无人机仿真（全局无 GNSS）')
    for index, (_, title) in enumerate(MAIN_ENTRIES, 1):
        print(f'{index}. {title}')
    print('两种程序默认实体 HID，可选择模拟遥控器。新终端操作：status / save / export-mesh / logs / stop')


def interactive(dry_run=False):
    while True:
        print('\n无人机仿真：两个完整程序（全局无 GNSS，Ctrl+C 退出）')
        for index, (_, title) in enumerate(MAIN_ENTRIES, 1):
            print(f'  {index}. {title}')
        print('   l. 加载已保存的 JSON 配置\n   0. 退出')
        answer = input('请选择：').strip().lower()
        if answer in ('0', 'q', 'exit'):
            return 0
        try:
            if answer == 'l':
                config = read_config(Path(path_arg(input('配置文件路径：').strip())))
                print(BY_KEY[config['profile']].note)
            elif answer.isdecimal() and 1 <= int(answer) <= len(MAIN_ENTRIES):
                config = configure(BY_KEY[MAIN_ENTRIES[int(answer) - 1][0]])
            else:
                print('请输入有效编号或 l。')
                continue
            while True:
                preview(config)
                if dry_run:
                    print('仅预览：未启动进程。')
                    return 0
                action = input('\n1 启动 / 2 修改配置 / 3 保存配置 / 0 返回主菜单：').strip()
                if action == '0':
                    break
                if action == '1':
                    launch(config)
                elif action == '2':
                    config = configure(BY_KEY[config['profile']], config['options'])
                elif action == '3':
                    filename = input('保存路径（默认 .cache/sim/profiles/' + config['profile'] + '.json）：').strip()
                    try:
                        save_config(config, Path(path_arg(filename or '.cache/sim/profiles/' + config['profile'] + '.json')))
                    except OSError as exc:
                        print(f'保存失败：{exc}；请选择新文件名。')
                else:
                    print('请输入 0、1、2 或 3。')
        except (ValueError, OSError, RuntimeError) as exc:
            print(f'无法启动：{exc}', file=sys.stderr)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true', help='显示两个仿真程序后退出')
    parser.add_argument('--config', type=lambda p: Path(p).expanduser().resolve(), help='加载 JSON 配置直接启动')
    parser.add_argument('--dry-run', action='store_true', help='只预览命令，不启动仿真')
    args = parser.parse_args(argv)
    if args.list:
        show_tree()
        return 0
    if args.config:
        config = read_config(args.config)
        print(BY_KEY[config['profile']].note)
        preview(config)
        if not args.dry_run:
            launch(config)
        return 0
    if not sys.stdin.isatty() and not args.dry_run:
        parser.error('交互菜单需要终端；自动化使用 menu --config <文件>，预览可加 --dry-run')
    return interactive(args.dry_run)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except EOFError:
        print('\n输入结束，退出菜单。')
    except KeyboardInterrupt:
        print('\n已退出菜单。')
        sys.exit(130)
    except (ValueError, OSError) as exc:
        print(f'配置或启动失败：{exc}', file=sys.stderr)
        sys.exit(1)
