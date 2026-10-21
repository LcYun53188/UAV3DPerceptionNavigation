#!/usr/bin/env python3
"""统一仿真 CLI：配置启动、交互菜单、算法控制和高级运行器入口。"""
from __future__ import annotations

import argparse
from pathlib import Path
import shlex
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulation import backends, menu, profiles

PROG = './scripts/sim.sh'


def list_profiles():
    for profile in profiles.PROFILES:
        print(f'{profile.key:14} {profile.title}')


def config_path(value):
    return Path(value).expanduser().resolve()


def run_profile(arguments):
    # Find the selected profile before registering its particular options. This
    # keeps CLI defaults and menu defaults in the same registry.
    selector = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    selector.add_argument('--config', type=config_path)
    selected, remainder = selector.parse_known_args(arguments)
    saved = profiles.read_config(selected.config) if selected.config else None
    key = saved['profile'] if saved else next((word for word in remainder if word in profiles.BY_KEY), None)
    parser = argparse.ArgumentParser(prog=PROG + ' run', allow_abbrev=False, description='按注册配置启动仿真；先用 --dry-run 查看命令。')
    parser.add_argument('profile', nargs='?', choices=profiles.BY_KEY, help='仿真类型；也可由 --config 指定')
    parser.add_argument('--config', type=config_path, help='读取 JSON 配置，命令行选项覆盖同名配置')
    parser.add_argument('--save', type=config_path, help='保存最终配置，不覆盖已有文件')
    parser.add_argument('--dry-run', action='store_true', help='仅预览，不加载 ROS 或启动进程')
    if key:
        profile = profiles.BY_KEY[key]
        parser.description = profile.title + '\n' + profile.note
        for item in profile.options:
            name = '--' + item.key.replace('_', '-')
            kwargs = dict(default=argparse.SUPPRESS, help=item.label)
            if type(item.default) is bool:
                kwargs['action'] = argparse.BooleanOptionalAction
            else:
                kwargs['type'] = type(item.default)
                if item.choices:
                    kwargs['choices'] = [value for value, _ in item.choices]
            parser.add_argument(name, **kwargs)
    args = parser.parse_args(arguments)
    if key is None:
        parser.error('请指定 profile 或 --config；用 sim.sh list 查看类型')
    if args.profile and args.profile != key:
        parser.error('指定的 profile 与 JSON 配置不一致')
    options = dict(saved['options']) if saved else {}
    options.update({item.key: getattr(args, item.key) for item in profiles.BY_KEY[key].options
                    if hasattr(args, item.key)})
    config = profiles.validate_config(dict(schema=1, profile=key, options=options))
    if args.save:
        profiles.save_config(config, args.save)
    print(profiles.BY_KEY[key].note)
    menu.preview(config)
    if not args.dry_run:
        menu.launch(config)
    return 0


def advanced(arguments):
    parser = argparse.ArgumentParser(prog=PROG + ' advanced', allow_abbrev=False, description='直接向运行器传参；查看专项手册中的适用条件。')
    parser.add_argument('backend', choices=backends.BACKENDS)
    parser.add_argument('--dry-run', action='store_true', help='仅预览底层启动命令')
    args, forwarded = parser.parse_known_args(arguments)
    if forwarded[:1] == ['--']:
        forwarded = forwarded[1:]
    command = backends.command(args.backend, forwarded)
    if args.dry_run:
        print(shlex.join(command))
    else:
        backends.execute(command)
    return 0


def help_parser():
    return argparse.ArgumentParser(
        prog=PROG,
        description='无人机仿真统一入口；无参数打开交互菜单。',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''常用命令：
  menu [--dry-run]              交互选择与配置
  list                         查看全部仿真类型
  run <profile> [配置选项]      按配置启动，支持 --dry-run / --config / --save
  algorithm <动作> [参数]      算法会话：start / init / status / cancel / save / load / stop
  advanced <backend> [参数]    直接向底层运行器传参

示例：
  ./scripts/sim.sh run algorithm --layout lab --view rviz --dry-run
  ./scripts/sim.sh run vio-pocket --device /dev/input/js0
  ./scripts/sim.sh algorithm status

原有 start、px4-flight 等子命令与历史脚本继续兼容。
完整说明：docs/simulation/LAUNCHERS.md''')


def main(argv=None):
    args = list(sys.argv[1:] if argv is None else argv)
    if not args:
        return menu.main([])
    action, rest = args[0], args[1:]
    if action in ('-h', '--help'):
        help_parser().parse_args(rest)
        help_parser().print_help()
        return 0
    if action == 'menu':
        return menu.main(rest)
    if action == 'list':
        argparse.ArgumentParser(prog=PROG + ' list').parse_args(rest)
        list_profiles()
        return 0
    if action == 'run':
        return run_profile(rest)
    if action == 'advanced':
        return advanced(rest)
    if action == 'algorithm':
        backends.execute(backends.command('algorithm', rest))
        return 0
    if action == 'compat':
        if not rest or rest[0] not in backends.LEGACY_SCRIPTS:
            raise ValueError('未知历史脚本入口')
        key, prefix = backends.LEGACY_SCRIPTS[rest[0]]
        backends.execute(backends.command(key, [*prefix, *rest[1:]]))
        return 0
    if action in backends.LEGACY_COMMANDS:
        backends.execute(backends.command(backends.LEGACY_COMMANDS[action], rest))
        return 0
    if action in backends.ALGORITHM_COMMANDS:
        backends.execute(backends.command('algorithm', args))
        return 0
    help_parser().error('未知子命令：' + action)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except EOFError:
        print('\n输入结束，退出菜单。')
    except KeyboardInterrupt:
        print('\n已退出。')
        sys.exit(130)
    except (ValueError, OSError) as exc:
        print(f'配置或启动失败：{exc}', file=sys.stderr)
        sys.exit(1)
