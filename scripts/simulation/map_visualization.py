"""Startup-only map display choices shared by the CLI and VIO runner."""
from pathlib import Path
import math


MODES = (
    ('height', '高度'), ('rgb', '相机 RGB'), ('normals', '表面法线'),
    ('esdf', '障碍距离 ESDF 切片'), ('semantic', '人员关联区域／未分类'),
    ('weight', 'TSDF 融合权重'), ('age', '网格块更新年龄'),
    ('traversability', '球形机体净空切片'),
)
# key, label, default, inclusive limits
SCALES = (
    ('map_height_min', '高度颜色下限（m）', 0., (-100., 100.)),
    ('map_height_max', '高度颜色上限（m）', 4., (-100., 100.)),
    ('map_distance_max', 'ESDF 颜色上限（m）', 3., (.1, 20.)),
    ('map_slice_height', 'ESDF／可通行性切片高度（map 坐标 m）', 1., (-100., 100.)),
    ('map_clearance', '可通行性球形机体半径（m）', .8, (.05, 5.)),
    ('map_caution_margin', '可通行性黄色余量（m）', .3, (.01, 5.)),
    ('map_age_max', '更新时间颜色上限（s）', 30., (1., 3600.)),
    ('map_semantic_radius', '人员躯干表面关联半径（m）', .4, (.05, 2.)),
)


def add_arguments(parser):
    parser.add_argument('--map-color', choices=dict(MODES), default='height')
    for key, label, default, _ in SCALES:
        parser.add_argument('--' + key.replace('_', '-'), type=float, default=default, help=label)


def validate(options):
    if options['map_color'] not in dict(MODES):
        raise ValueError('未知地图着色模式')
    for key, label, _, (low, high) in SCALES:
        value = options[key]
        if type(value) not in (float, int) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f'{label}：范围 {low}–{high}')
    if options['map_height_min'] >= options['map_height_max']:
        raise ValueError('地图高度颜色下限必须小于上限')


def options_from(args):
    options = {key: getattr(args, key) for key in ['map_color', *(s[0] for s in SCALES)]}
    validate(options)
    return options


def arguments(options):
    validate(options)
    result = ['--map-color', options['map_color']]
    for key, _, _, _ in SCALES:
        result += ['--' + key.replace('_', '-'), str(options[key])]
    return result


def display_spec(mode):
    if mode == 'weight':
        return 'nvblox_rviz_plugin/NvbloxVoxelBlockLayer', '/uav/map/colored_voxels', None
    if mode in ('esdf', 'traversability'):
        return 'rviz_default_plugins/MarkerArray', '/uav/map/color_slice', None
    if mode in ('semantic', 'age'):
        return 'nvblox_rviz_plugin/NvbloxMesh', '/uav/map/colored_mesh', 'RGB'
    return 'nvblox_rviz_plugin/NvbloxMesh', '/nvblox_node/mesh', dict(height='Height', rgb='RGB', normals='Normals')[mode]


def write_rviz(source, destination, options, *, openvins=False):
    """Write a session-owned config, preserving the repository template."""
    import yaml
    validate(options)
    original = Path(source).read_text()
    if openvins:
        original = original.replace('/visual_slam/tracking/odometry', '/openvins/odometry').replace(
            '/visual_slam/tracking/vo_path', '/openvins/path')
    config = yaml.safe_load(original)
    displays = config['Visualization Manager']['Displays']
    mesh = next(d for d in displays if d.get('Name') == 'Live Depth Map')
    mode = options['map_color']
    cls, topic, color = display_spec(mode)
    mesh['Class'] = cls
    mesh['Name'] = 'Live Map: ' + dict(MODES)[mode]
    mesh['Topic']['Value'] = topic
    for key in ('Mesh Color', 'Min Height', 'Max Height', 'Cut Ceiling', 'Ceiling Height'):
        mesh.pop(key, None)
    if color:
        mesh.update({'Mesh Color': color, 'Min Height': options['map_height_min'],
                     'Max Height': options['map_height_max'], 'Cut Ceiling': True, 'Ceiling Height': 3.8})
    if cls == 'rviz_default_plugins/MarkerArray':
        mesh['Namespaces'] = {}
    if mode in ('esdf', 'semantic', 'weight', 'age', 'traversability'):
        displays.append({'Class': 'rviz_default_plugins/MarkerArray', 'Name': 'Map Color Legend',
                         'Enabled': True, 'Value': True, 'Topic': dict(mesh['Topic'], Value='/uav/map/color_legend')})
    Path(destination).write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
    return Path(destination)


def node_parameters(options):
    """Only derived displays need an extra read-only subscriber process."""
    mode = options['map_color']
    if mode not in ('esdf', 'semantic', 'weight', 'age', 'traversability'):
        return []
    result = ['-p', f'mode:={mode}']
    for key in ('map_distance_max', 'map_slice_height', 'map_clearance', 'map_caution_margin',
                'map_age_max', 'map_semantic_radius'):
        result += ['-p', f'{key.removeprefix("map_")}:={options[key]}']
    return result
