#!/usr/bin/env python3
"""两个完整仿真程序及当前会话操作。"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from support.paths import enable_imports
enable_imports()
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


def help_parser():
    return argparse.ArgumentParser(
        prog=PROG,
        description='无人机仿真统一入口；无参数打开交互菜单。',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''程序：
  ./scripts/sim.sh run manual                光流／测距纯手动，无 VIO
  ./scripts/sim.sh run full                  VIO＋建图＋导航完整项目
  ./scripts/sim.sh run full --input-source virtual   使用模拟遥控器

新终端操作当前会话：
  ./scripts/sim.sh status
  ./scripts/sim.sh save .cache/maps/run1      保存地图包（落地上锁后）
  ./scripts/sim.sh export-mesh .cache/maps/run1.ply
  ./scripts/sim.sh logs [--follow]
  ./scripts/sim.sh stop                      请求落地上锁后关闭
  ./scripts/sim.sh cleanup [--apply]         预览／清理监督器退出后的残留进程

run 支持 --dry-run、--config、--save；list 只列出两个程序。
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
    if action == 'cleanup':
        from simulation.cleanup import main as cleanup
        return cleanup(rest)
    if action in ('status', 'save', 'export-mesh', 'logs', 'stop'):
        from simulation.control import main as control
        return control(args)
    if action in ('manual', 'full'):
        return run_profile(args)
    if (action in ('advanced', 'algorithm', 'compat') or action in backends.LEGACY_COMMANDS
            or action in backends.ALGORITHM_COMMANDS):
        raise ValueError('旧仿真入口已移除；请选择 sim.sh run manual 或 sim.sh run full。'
                         '当前会话使用 status / save / export-mesh / logs / stop。')
    help_parser().error('未知子命令：' + action)


if __name__ == '__main__':
    try:
        sys.exit(main())
    except EOFError:
        print('\n输入结束，退出菜单。')
    except KeyboardInterrupt:
        print('\n已退出。')
        sys.exit(130)
    except (ValueError, OSError, RuntimeError) as exc:
        print(f'配置或启动失败：{exc}', file=sys.stderr)
        sys.exit(1)
