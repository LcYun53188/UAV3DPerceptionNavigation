#!/usr/bin/env python3
"""交互仿真菜单；配置与启动策略由 profiles 和 backends 统一维护。"""
from __future__ import annotations

import argparse
import os
from pathlib import Path
import shlex
import sys

# Also support the historical scripts/simulation/menu.py symlink and direct execution.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from support.paths import enable_imports
enable_imports()
from simulation.profiles import (ROOT, SIM, PROFILES, BY_KEY, validate_config,
                                 command, path_arg, read_config, save_config, manual_unavailable)
from simulation.backends import execute
from pocket_input import preflight_device


def launch(config):
    config = validate_config(config)
    o = config['options']
    if o.get('qgc_config') and not Path(path_arg(o['qgc_config'])).is_file():
        raise ValueError('QGC 校准文件不存在：' + path_arg(o['qgc_config']))
    if config['profile'] in ('vio-pocket', 'flow-pocket'):
        identity = preflight_device(o['device'], o.get('input_profile', 'uav-aetr-v1'))
        print(f'摇杆检查通过：{identity["name"]}，{identity["axes"]}轴／{identity["buttons"]}按钮，{identity["device"]}', flush=True)
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


def configure(profile, previous=None, skip=()):
    print('\n' + profile.title + '\n' + profile.note)
    options = {}
    for item in profile.options:
        default = (previous or {}).get(item.key, item.default)
        if item.key in skip:
            options[item.key] = item.validate(default)
            continue
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
    if config['profile'] == 'algorithm':
        if config['options']['mode'] == 'mapping':
            print('另一终端：./scripts/sim.sh algorithm init；完成后在 RViz 选点或运行 sim.sh algorithm explore。')
        else:
            print('另一终端：./scripts/sim.sh algorithm load <地图目录>；地图必须匹配所选场景。')
        print('退出：Ctrl+C 或另一终端 ./scripts/sim.sh algorithm stop。')
    elif config['profile'] in ('vio-pocket', 'flow-pocket', 'manual'):
        print('会话运行时由所选 Pocket 输入控制；落地未解锁后，按对应手册关闭会话。')
        if config['profile'] == 'manual':
            print('纯手动会话：PREPARE／RUN 等自动任务意图被拒绝。')
            if config['options']['input_source'] == 'auto':
                print('自动选择：优先实体 Pocket；没有实体时复用虚拟设备，或自动打开虚拟面板。')
            elif config['options']['input_source'] == 'physical':
                print('实体 Pocket：选择 USB Joystick，在 QGC 完成校准并启用摇杆；使用 Fly 页面发送操纵。')
            else:
                print('电脑虚拟 Pocket：自动复用或创建虚拟设备与面板。')
    elif config['profile'] == 'autonomous-mapping':
        print('启动后自动初始化起点，再启动自主探索；全场覆盖以实际探索状态为准。')
        print('使用 Gazebo 真值定位与简化速度模型；Ctrl+C 或 sim.sh algorithm stop 退出。')
    elif config['profile'] == 'navigation':
        print('启动后自动加载地图或初始化起点，再在 RViz 使用 2D Goal Pose 选择任务点。')
        print('当前选择：RViz 单点导航；Ctrl+C 或 sim.sh algorithm stop 退出。')
    else:
        print('进程退出与清理由所选入口负责；终端保留前台运行。')


MAIN_ENTRIES = (
    ('manual', 'Gazebo＋RViz：纯手动飞行'),
    ('autonomous-mapping', 'Gazebo＋RViz：自动建图'),
    ('navigation', 'Gazebo＋RViz：任务点／行为树导航'),
    ('diagnostics', '传感器、定位与环境检查'),
    ('tools', '辅助工具与高级配置'),
)
ALGORITHM_SCENES = (('expanded', '扩展场景 20×20 m'), ('lab', '实验室 10×10 m'))


class ReturnToMain(Exception):
    pass


def stage(label, choices, default):
    print('\n' + label)
    for index, (value, title) in enumerate(choices, 1):
        print(f'  {index}. {title}' + ('（默认）' if value == default else ''))
    print('  0. 返回主菜单')
    while True:
        answer = input('输入编号，回车用默认值：').strip()
        if answer == '0':
            raise ReturnToMain()
        if not answer:
            return default
        if answer.isdecimal() and 1 <= int(answer) <= len(choices):
            return choices[int(answer) - 1][0]
        print('请输入有效编号。')


def show_tree():
    print('无人机仿真菜单：操作方式 → 功能配置 → 场景（全局无 GNSS）')
    print('1. 纯手动飞行（Gazebo＋RViz）\n'
          '   二级：VIO 开／关、建图开／关\n'
          '   三级：对应配置的兼容场景\n'
          '   当前可运行：VIO＋建图＋仓库；其他组合标明待接入\n'
          '2. 自动建图（Gazebo＋RViz）\n'
          '   二级：自动初始化 → 自主探索建图（仅一项）\n'
          '   三级：扩展场景／实验室\n'
          '3. 任务点／行为树导航（Gazebo＋RViz）\n'
          '   二级：不加载地图／加载现有地图\n'
          '   三级：扩展场景／实验室\n'
          '   随后：任务点与行为树配置；当前可运行 RViz 单点导航\n'
          '4. 传感器、定位与环境检查\n'
          '5. 辅助工具与高级配置\n'
          'l. 加载已保存的 JSON 配置\n0. 退出')


def configure_workflow(key, previous=None):
    previous = previous or {}
    if key == 'manual':
        vio, mapping = stage('二级菜单：选择定位与建图功能', (
            ((False, False), 'VIO 关闭／建图关闭［待接入 RViz］'),
            ((True, False), 'VIO 开启／建图关闭［待解耦］'),
            ((False, True), 'VIO 关闭／建图开启［待接入］'),
            ((True, True), 'VIO 开启／建图开启'),
        ), (previous.get('vio', True), previous.get('mapping', True)))
        scenes = (('warehouse-avoidance', '仓库静态障碍场景'),) if vio else (('px4-default', 'PX4 基础场景［RViz 待接入］'),)
        scene = stage('三级菜单：选择场景', scenes, previous.get('scene', scenes[0][0]) if previous.get('scene') in dict(scenes) else scenes[0][0])
        reason = manual_unavailable(vio, mapping, scene)
        if reason:
            print(reason + '；该组合不会启动仿真。')
            return None
        return configure(BY_KEY[key], dict(previous, vio=vio, mapping=mapping, scene=scene), skip=('vio', 'mapping', 'scene'))
    if key == 'autonomous-mapping':
        stage('二级菜单：自动建图方式', (('explore', '自动初始化 → 自主探索建图（算法仿真）'),), 'explore')
        layout = stage('三级菜单：选择场景', ALGORITHM_SCENES, previous.get('layout', 'expanded'))
        return validate_config(dict(schema=1, profile=key, options=dict(layout=layout)))
    if key == 'navigation':
        load_map = stage('二级菜单：是否加载现有地图', ((False, '不加载，在线建图导航'), (True, '加载现有地图导航')), previous.get('load_map', False))
        layout = stage('三级菜单：选择场景', ALGORITHM_SCENES, previous.get('layout', 'expanded'))
        source = stage('任务点与行为树配置', BY_KEY[key].options[-1].choices, previous.get('task_source', 'rviz'))
        if source != 'rviz':
            print('待接入：多航点交互选择／自定义行为树尚未连接到此流程，不会启动其他任务替代。')
            return None
        return configure(BY_KEY[key], dict(previous, load_map=load_map, layout=layout, task_source=source),
                         skip=('load_map', 'layout', 'task_source') + (() if load_map else ('map_bundle',)))
    if key == 'diagnostics':
        target = stage('二级菜单：选择检查类型', tuple((p, BY_KEY[p].title) for p in ('vio-audit', 'depth', 'sitl', 'doctor')), 'vio-audit')
    else:
        target = stage('二级菜单：选择工具', (('panel', '虚拟 Pocket 控制面板'), ('scene', 'Gazebo 场景与传感器查看'),
                       ('isaac', 'Isaac Sim 4.5 UI'), ('advanced', '原始配置列表（专项调试）')), 'panel')
        if target == 'advanced':
            target = stage('高级配置：选择底层入口', tuple((p.key, p.title) for p in PROFILES), 'algorithm')
    return configure(BY_KEY[target])


def interactive(dry_run=False):
    while True:
        print('\n无人机仿真总启动菜单（全局无 GNSS，Ctrl+C 退出）')
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
                config = configure_workflow(MAIN_ENTRIES[int(answer) - 1][0])
                if config is None:
                    if dry_run:
                        return 0
                    continue
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
                    if config['profile'] in ('manual', 'autonomous-mapping', 'navigation'):
                        updated = configure_workflow(config['profile'], config['options'])
                        if updated is None:
                            break
                        config = updated
                    else:
                        config = configure(BY_KEY[config['profile']], config['options'])
                elif action == '3':
                    filename = input('保存路径（默认 .cache/sim/profiles/' + config['profile'] + '.json）：').strip()
                    try:
                        save_config(config, Path(path_arg(filename or '.cache/sim/profiles/' + config['profile'] + '.json')))
                    except OSError as exc:
                        print(f'保存失败：{exc}；请选择新文件名。')
                else:
                    print('请输入 0、1、2 或 3。')
        except ReturnToMain:
            continue
        except (ValueError, OSError) as exc:
            print(f'无法启动：{exc}', file=sys.stderr)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--list', action='store_true', help='显示分层菜单结构后退出')
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
