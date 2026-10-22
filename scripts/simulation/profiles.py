"""仿真配置注册表：菜单、命令行和 JSON 配置共用的选项与参数规则。"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path

from .backends import ROOT, command as backend_command

SIM = str(ROOT / 'scripts/sim.sh')


@dataclass(frozen=True)
class Option:
    key: str
    label: str
    default: object
    choices: tuple = ()
    limits: tuple = ()

    def validate(self, value):
        if self.choices:
            if type(value) is not type(self.default) or value not in dict(self.choices):
                raise ValueError(f'{self.label}：请选择列出的选项')
        elif self.limits:
            if (type(value) not in (int, float) or not math.isfinite(value)
                    or not self.limits[0] <= value <= self.limits[1]
                    or (type(self.default) is int and type(value) is not int)):
                raise ValueError(f'{self.label}：范围 {self.limits[0]}–{self.limits[1]}，单位见提示')
        elif not isinstance(value, str) or any(c in value for c in ('\0', '\n', '\r')):
            raise ValueError(f'{self.label}：需要单行文本路径')
        if self.key == 'device' and value != 'auto' and (not value or not value.startswith('/dev/input/')):
            raise ValueError('摇杆设备需要 auto 或 /dev/input/ 下的绝对路径')
        return value


@dataclass(frozen=True)
class Profile:
    key: str
    title: str
    note: str
    options: tuple = ()


INPUT = Option('input_source', '遥控器输入', 'physical',
               (('physical', '实体 HID 遥控器'), ('virtual', '模拟遥控器（电脑面板）')))
COMMON_OPTIONS = (
    INPUT, Option('device', 'HID 设备路径（auto 自动识别）', 'auto'),
    Option('qgc_config', '继承 QGC 校准文件（留空则自动复用该设备校准）', ''),
    Option('session_duration', '会话时长（秒）', 3600, limits=(60, 14400)),
)
PROFILES = (
    Profile('manual', '纯手动飞行：光流／测距，无 VIO',
            'Gazebo＋PX4＋QGC，使用原生光流／测距定位；不启动 VIO、建图或自动任务。', COMMON_OPTIONS),
    Profile('full', '完整项目：VIO／建图／导航／行为树',
            'Gazebo＋PX4＋QGC＋RViz，运行 VIO、光流／测距、nvblox、EGO 和遥控器任务控制。', COMMON_OPTIONS),
)
BY_KEY = {profile.key: profile for profile in PROFILES}


def validate_config(config):
    if not isinstance(config, dict) or set(config) - {'schema', 'profile', 'options'}:
        raise ValueError('配置只接受 schema、profile、options 字段')
    if type(config.get('schema')) is not int or config['schema'] != 1:
        raise ValueError('配置 schema 必须为 1')
    key = config.get('profile')
    if not isinstance(key, str) or key not in BY_KEY:
        raise ValueError('未知仿真 profile；使用 menu --list 查看')
    profile = BY_KEY[key]
    options = config.get('options', {})
    if not isinstance(options, dict) or set(options) - {item.key for item in profile.options}:
        raise ValueError('options 包含未知配置项')
    options = {item.key: item.validate(options.get(item.key, item.default)) for item in profile.options}
    return dict(schema=1, profile=key, options=options)


def path_arg(value):
    path = Path(value).expanduser()
    return str((ROOT / path).resolve())


def command(config):
    config = validate_config(config)
    key, o = config['profile'], config['options']
    args = ['--input-source', o['input_source'], '--device', o['device'],
            '--session-duration', str(o['session_duration'])]
    if o['qgc_config']:
        args += ['--qgc-config-from', path_arg(o['qgc_config'])]
    return backend_command(key, args)


def read_config(path):
    return validate_config(json.loads(path.read_text()))


def save_config(config, path):
    config = validate_config(config)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('x') as handle:
        handle.write(json.dumps(config, ensure_ascii=False, indent=2, allow_nan=False) + '\n')
    print(f'已保存：{path}')
