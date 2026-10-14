#!/usr/bin/env python3
"""Generate workspace catalogs and check local documentation/script syntax without running tools."""
from __future__ import annotations

import argparse
import ast
from collections import defaultdict
import os
from pathlib import Path
import re
import subprocess
import sys
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[1]
PACKAGES = (
    'uav_bringup', 'uav_nav_sim', 'uav_ego_adapter', 'ego_planner_vendor',
    'uav_mission', 'uav_bt', 'uav_nav_interfaces', 'nav_guard', 'nav_safety',
    'nav_mapping', 'nav_px4_bridge', 'imu_fusion', 'oakd_perception', 'px4_comm_bridge',
)
DOC_GROUPS = {
    'setup': '安装与构建', 'simulation': '仿真运行与场景',
    'navigation': '导航与任务契约', 'flight': 'PX4、BT 与定位模式',
    'hardware': '硬件定位与通信', 'development': '开发设计与计划',
    'history': '历史资料',
}
SCRIPT_GROUPS = (
    '统一入口与环境', '构建', '安装与依赖准备', '运行与交互',
    '场景、标定与资产准备', '在线检查与故障回归',
    '离线分析、验收与归档', '共享实现与原生辅助源码', '包内与场景启动脚本',
)
DESCRIPTIONS = {
    'sim.sh': '算法仿真与 PX4 子命令统一分发入口；具体参数见对应手册。',
    'with_venv.sh': '进入工作区，加载 .venv、系统 ROS 2 与 install_uav。',
    'with_px4_sim.sh': '加载隔离的 PX4 消息与仿真环境。',
    'build_algorithm_sim.sh': '构建 EGO、nvblox、算法导航与显示插件。',
    'build_livox_sdk2.sh': '构建 Livox SDK2。',
    'build_navigation_actions.sh': '增量构建第一方导航 Action、Mission 和 BT。',
    'build_nvidia_3d_nav_deps.sh': '构建 NVIDIA 三维导航依赖。',
    'build_px4_flight.sh': '构建隔离的任务、BT 与 PX4 飞行后端。',
    'build_px4_sim.sh': '准备并构建锁定的 PX4 SITL、Agent 与消息包。',
    'build_px4_vio.sh': '调用独立 PX4 VIO 遥测构建器。',
    'build_uav_stack.sh': '使用 build_uav／install_uav 构建硬件感知与定位栈。',
    'build_vio_motion.sh': '构建独立 VIO 运动载台并记录构建回执。',
    'build_vio_node.sh': '构建当前默认 SDK16 ROS VIO 节点。',
    'build_vio_same_aircraft_motion.sh': '构建同机 VIO 运动插件并记录回执。',
    'apply_vendor_patches.sh': '调用 vendor 补丁应用／精确校验工具。',
    'install_vendor_lfs_assets.sh': '准备或校验 vendor LFS 资产。',
    'prepare_ego_vendor.sh': '准备锁定的 EGO 上游源码并应用受管补丁。',
    'install_cuda_13_2_stack.sh': '安装 CUDA 13.2；执行前阅读安装文档。',
    'install_vpi_4_stack.sh': '安装 VPI 4；执行前核对环境。',
    'install_nvidia_3d_nav_rosdeps.sh': '安装 NVIDIA ROS 依赖。',
    'nav_debug_lib.sh': '导航诊断脚本共用的环境与话题配置。',
    'nav_launch.env': '导航启动的公共环境变量默认值。',
    'record_nav_debug_bag.sh': '采集导航诊断 ROS bag。',
    'check_nav_stack.sh': '检查导航图与输入状态。',
    'measure_oakd_vio_performance.sh': '采集 OAK-D VIO 性能指标。',
    'run_mid360_fastlio_rviz.sh': '启动 MID360、FAST-LIO 与 RViz。',
    'run_rviz_nav.sh': '启动导航 RViz。',
    'run_vio_monitor.sh': '启动 VIO 监视器。',
    'run_px4_microxrce.sh': '启动 PX4 Micro XRCE-DDS Agent。',
    'run_px4_flight.sh': '在隔离环境中启动 PX4 任务运行器。',
    'run_px4_vio_sensors.sh': '在隔离环境中启动 VIO 传感器／实验任务运行器。',
    'run_px4_mtf01.py': '默认未解锁光流／测距审计；显式 --flight 支持独立光流飞行仿真。',
    'run_gazebo_harmonic_nav.sh': '启动 Gazebo 场景与传感器桥，保持单实例锁。',
    'run_uav_obstacle_course.sh': '选择障碍场地并调用 Gazebo 场景启动器。',
    'run_isaac_sim_45_ui.sh': '从已有 Isaac Sim 4.5 安装启动 UI。',
    'run_oakd_active_max.sh': '启动 OAK-D 主动双目强度配置。',
    'run_oakd_balance.sh': '启动 OAK-D 均衡深度配置。',
    'run_oakd_indoor.sh': '启动 OAK-D 室内深度配置。',
    'run_oakd_outdoor.sh': '启动 OAK-D 室外深度配置。',
    'gazebo_odometry_velocity.py': '由 Gazebo／VIO 位姿计算平滑三维机体系速度，只修改 twist。',
    'record_vio_same_aircraft_build.py': '记录同机运动插件源码与二进制回执。',
    'openvins_covariance_replay.cpp': 'OpenVINS 历史候选原生协方差回放诊断。',
    'openvins_pose_covariance.hpp': 'OpenVINS 历史候选位姿协方差辅助实现。',
    'openvins_ros_node.cpp': 'OpenVINS 历史候选 ROS 接口源码；当前路线暂停。',
    'vio_sdk_covariance_replay.cpp': '原生 SDK 位姿／协方差回放诊断。',
    'vio_sdk16_conditional_covariance.hpp': 'SDK16 条件位姿协方差导出辅助实现。',
    'vio_sdk16_covariance.hpp': 'SDK16 协方差导出共用实现。',
    'vio_sdk16_marginal_covariance.hpp': 'SDK16 边缘位姿协方差导出辅助实现。',
    'vio_sdk16_world_covariance.hpp': 'SDK16 世界坐标系协方差转换辅助实现。',
}


def script_files() -> list[Path]:
    files = [p for p in (ROOT / 'scripts').rglob('*')
             if p.is_file() and not p.is_symlink()
             and '__pycache__' not in p.parts and not p.name.startswith('test_')
             and p.suffix in {'.py', '.sh', '.cpp', '.hpp', '.env'}
             and p.name != 'nav_launch.local.env']
    files += list((ROOT / 'simulation/scripts').glob('*.sh'))
    for package in PACKAGES:
        directory = ROOT / 'src' / package / 'scripts'
        if directory.exists():
            files += [p for p in directory.rglob('*') if p.is_file()
                      and '__pycache__' not in p.parts and p.suffix in {'.py', '.sh'}]
    return sorted(set(files))


def description(path: Path) -> str:
    if path.name in DESCRIPTIONS:
        return DESCRIPTIONS[path.name]
    if path.suffix == '.py':
        tree = ast.parse(path.read_text())
        doc = ast.get_docstring(tree)
        if doc:
            return doc.splitlines()[0]
    return '随所属模块维护的辅助脚本；使用前查看源码。'


def script_group(path: Path) -> str:
    relative = path.relative_to(ROOT).as_posix()
    name = path.name
    if not relative.startswith('scripts/'):
        return SCRIPT_GROUPS[8]
    if name in {'sim.sh', 'sim_control.py', 'sim_ros_control.py',
                'with_venv.sh', 'with_px4_sim.sh', 'maintain_catalogs.py'}:
        return SCRIPT_GROUPS[0]
    if name.startswith('build_'):
        return SCRIPT_GROUPS[1]
    if '/setup/' in relative or name.startswith(('install_', 'vendor_')):
        return SCRIPT_GROUPS[2]
    if name.startswith(('run_', 'usb_', 'bt_', 'pocket_', 'select_',
                        'localization_selection_', 'oakd_vio_motion_ui', 'record_nav_')):
        return SCRIPT_GROUPS[3]
    if name.startswith('prepare_'):
        return SCRIPT_GROUPS[4]
    if name.startswith(('check_', 'audit_', 'px4_', 'inspect_', 'survey_', 'measure_')):
        return SCRIPT_GROUPS[5]
    if name.startswith(('analyze_', 'assess_', 'diagnose_', 'plot_', 'freeze_', 'record_')):
        return SCRIPT_GROUPS[6]
    return SCRIPT_GROUPS[7]


def markdown_link(path: Path, owner: Path) -> str:
    label = path.relative_to(ROOT).as_posix()
    return f'[{label}]({Path(os.path.relpath(path, owner.parent)).as_posix()})'


def script_catalog() -> str:
    owner = ROOT / 'scripts/README.md'
    grouped = defaultdict(list)
    for path in script_files():
        grouped[script_group(path)].append(path)
    lines = []
    for group in SCRIPT_GROUPS:
        lines += [f'## {group}', '', '| 文件 | 用途 |', '| --- | --- |']
        for path in grouped[group]:
            text = description(path).replace('|', '\\|').replace('\n', ' ')
            lines.append(f'| {markdown_link(path, owner)} | {text} |')
        lines.append('')
    lines += ['## 兼容入口', '', '旧路径为相对符号链接，与规范路径使用同一份实现。', '',
              '| 旧路径 | 规范路径 |', '| --- | --- |']
    for path in sorted((ROOT / 'scripts').glob('*.sh')):
        if path.is_symlink():
            lines.append(f'| {markdown_link(path, owner)} | {markdown_link(path.resolve(), owner)} |')
    return '\n'.join(lines) + '\n'


def heading_title(path: Path) -> str:
    for line in path.read_text().splitlines():
        if line.startswith('# '):
            return line[2:].strip().replace('|', '\\|')
    return path.stem


def package_documents(package: str) -> list[Path]:
    directory = ROOT / 'src' / package
    return sorted(p for p in directory.rglob('*.md')
                  if not any(part.startswith('.') for part in p.relative_to(directory).parts))


def document_catalog() -> str:
    owner = ROOT / 'docs/README.md'
    lines = []
    for directory, title in DOC_GROUPS.items():
        lines += [f'## {title}', '', '| 文档 | 主题 |', '| --- | --- |']
        for path in sorted((ROOT / 'docs' / directory).glob('*.md')):
            lines.append(f'| {markdown_link(path, owner)} | {heading_title(path)} |')
        lines.append('')
    lines += ['## 模块文档', '', '| 模块资料 | 主题 |', '| --- | --- |']
    for package in PACKAGES:
        for path in package_documents(package):
            lines.append(f'| {markdown_link(path, owner)} | {heading_title(path)} |')
    lines += ['', '## 仿真与依赖资源', '', '| 入口 | 内容 |', '| --- | --- |']
    for relative in ['simulation/README.md', 'simulation/px4/README.md',
                     'simulation/acceptance/README.md', 'simulation/assets/README.md',
                     'patches/vendor/README.md']:
        path = ROOT / relative
        lines.append(f'| {markdown_link(path, owner)} | {heading_title(path)} |')
    return '\n'.join(lines) + '\n'


def managed_text(path: Path, name: str, body: str) -> str:
    begin, end = f'<!-- BEGIN {name} CATALOG -->', f'<!-- END {name} CATALOG -->'
    text = path.read_text()
    if text.count(begin) != 1 or text.count(end) != 1:
        raise ValueError(f'{path.relative_to(ROOT)} must have exactly one catalog marker pair')
    prefix, rest = text.split(begin, 1)
    _, suffix = rest.split(end, 1)
    return prefix + begin + '\n\n' + body + '\n' + end + suffix


def prose(text: str) -> str:
    # Mask fenced code while preserving line numbers. Literal code paths are not hyperlinks.
    lines, fence = [], None
    for line in text.splitlines():
        match = re.match(r'^\s*(`{3,}|~{3,})', line)
        if match:
            if fence is None:
                fence = match.group(1)
            elif match.group(1)[0] == fence[0] and len(match.group(1)) >= len(fence):
                fence = None
            lines.append('')
        else:
            lines.append('' if fence else line)
    return '\n'.join(lines)


def anchors(path: Path) -> set[str]:
    result, counts = set(), defaultdict(int)
    for heading in re.findall(r'^\s{0,3}#{1,6}\s+(.+?)\s*#*$', prose(path.read_text()), re.M):
        heading = re.sub(r'!?\[([^]]+)\]\([^)]*\)', r'\1', heading)
        slug = re.sub(r'[^\w\- ]', '', heading.lower()).replace(' ', '-')
        count = counts[slug]
        result.add(f'{slug}-{count}' if count else slug)
        counts[slug] += 1
    result.update(re.findall(r'<a\s+(?:id|name)=["\']([^"\']+)', path.read_text()))
    return result


def documents() -> list[Path]:
    paths = [ROOT / 'README.md', ROOT / 'scripts/README.md']
    paths += [p for p in (ROOT / 'docs').rglob('*.md') if 'validation' not in p.parts]
    paths += list((ROOT / 'simulation').rglob('README.md'))
    paths += list((ROOT / 'patches').rglob('README.md'))
    for package in PACKAGES:
        paths += package_documents(package)
    return sorted(set(paths))


def check() -> int:
    errors, optional, links, anchor_cache = [], set(), 0, {}
    for path in documents():
        for number, line in enumerate(prose(path.read_text()).splitlines(), 1):
            for destination in re.findall(r'\]\(([^\s)]+)(?:\s+"[^"]*")?\)', line):
                if re.match(r'[A-Za-z][\w+.-]*:', destination) or destination.startswith('//'):
                    continue
                target, _, anchor = unquote(destination).partition('#')
                resolved = (path.parent / target).resolve() if target else path.resolve()
                links += 1
                if resolved.is_relative_to(ROOT / 'docs/validation'):
                    if not resolved.exists():
                        optional.add(destination)
                    continue
                if not resolved.exists():
                    errors.append(f'{path.relative_to(ROOT)}:{number}: missing {destination}')
                elif anchor and resolved.suffix == '.md':
                    if resolved not in anchor_cache:
                        anchor_cache[resolved] = anchors(resolved)
                    if anchor not in anchor_cache[resolved]:
                        errors.append(f'{path.relative_to(ROOT)}:{number}: missing anchor {destination}')
    covered = {ROOT / 'docs' / group / p.name for group in DOC_GROUPS
               for p in (ROOT / 'docs' / group).glob('*.md')}
    administrative = {ROOT / 'docs' / name for name in ['README.md', 'INDEX.md', 'MAINTENANCE.md']}
    for path in (ROOT / 'docs').rglob('*.md'):
        if 'validation' not in path.parts and path not in covered | administrative:
            errors.append(f'Uncataloged document: {path.relative_to(ROOT)}')
    syntax_files = script_files() + list((ROOT / 'scripts').glob('test_*.py'))
    for path in syntax_files:
        try:
            if path.suffix == '.py':
                compile(path.read_bytes(), str(path), 'exec')
            elif path.suffix == '.sh':
                result = subprocess.run(['bash', '-n', str(path)], capture_output=True, text=True)
                if result.returncode:
                    errors.append(f'{path.relative_to(ROOT)}: {result.stderr.strip()}')
        except (SyntaxError, UnicodeError) as error:
            errors.append(f'{path.relative_to(ROOT)}: {error}')
    for path in (ROOT / 'scripts').glob('*.sh'):
        if path.is_symlink() and (not path.exists() or not path.resolve().is_relative_to(ROOT / 'scripts')):
            errors.append(f'Broken or escaping compatibility link: {path.relative_to(ROOT)}')
    if (ROOT / '.git').exists():
        # Generic build/ ignore rules must not hide the canonical build scripts.
        paths = documents() + script_files()
        visibility = subprocess.run(
            ['git', 'check-ignore', '--no-index', '--stdin'], cwd=ROOT,
            input=''.join(str(p.relative_to(ROOT)) + '\n' for p in paths),
            capture_output=True, text=True,
        )
        if visibility.returncode not in {0, 1}:
            errors.append(f'Git visibility check failed: {visibility.stderr.strip()}')
        errors.extend(f'Cataloged file is ignored by Git: {p}'
                      for p in visibility.stdout.splitlines())
    for error in errors:
        print(error, file=sys.stderr)
    print(f'Checked {len(documents())} documents, {links} local links, '
          f'{len(syntax_files)} script/source files; '
          f'{len(optional)} unavailable optional evidence links; {len(errors)} errors.')
    return bool(errors)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument('--write', action='store_true', help='Update generated catalog regions only')
    mode.add_argument('--check', action='store_true', help='Check catalog freshness, local links and syntax')
    args = parser.parse_args()
    stale = []
    for path, name, body in [
        (ROOT / 'scripts/README.md', 'SCRIPT', script_catalog()),
        (ROOT / 'docs/README.md', 'DOCUMENT', document_catalog()),
    ]:
        desired = managed_text(path, name, body)
        if args.write:
            path.write_text(desired)
            print(f'Updated {path.relative_to(ROOT)}')
        elif path.read_text() != desired:
            stale.append(path.relative_to(ROOT))
    for path in stale:
        print(f'Stale catalog: {path}; run python3 scripts/maintain_catalogs.py --write', file=sys.stderr)
    return 0 if args.write else int(bool(check() or stale))


if __name__ == '__main__':
    raise SystemExit(main())
