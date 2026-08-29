"""
Unified navigation stack launch file (W1-D1-D3: 自包含化)

启动完整的无人机导航栈：
  1. 感知层: OAK-D RGB-D 相机 + IMU (400Hz) 
  2. 定位层: VINS-Fusion VIO + dual EKF (robot_localization)
  3. 规划层: 局部规划器 (当前APF，后续替换为DWB)
  4. 安全层: 多源健康监视
  5. 控制层: PX4 offboard 桥接

用法:
  ros2 launch uav_bringup nav_stack.launch.py enable_gps:=false
  ros2 launch uav_bringup nav_stack.launch.py odometry_source:=vio obstacle_pointcloud_source:=mid360
  ros2 launch uav_bringup nav_stack.launch.py odometry_source:=both obstacle_pointcloud_source:=both
  ros2 launch uav_bringup nav_stack.launch.py odometry_source:=lio obstacle_pointcloud_source:=mid360
"""

import os

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


LAUNCH_DEFAULTS = {
    # Core stack switches
    'enable_gps': 'false',
    'launch_dwb': 'false',
    'enable_oakd_perception': 'true',
    'enable_imu_fusion': 'true',
    'enable_vins': 'true',
    'odometry_source': 'vio',

    # Optional MID360 / LIO switches
    'enable_mid360': 'false',
    'enable_lio': 'false',
    'obstacle_pointcloud_source': 'oakd',

    # Point cloud topics
    'oakd_pointcloud_topic': '/oakd/points_filtered',
    'mid360_pointcloud_topic': '/mid360/points',
    'mid360_custom_topic': '/livox/lidar',
    'mid360_frame_id': 'mid360_link',
    'combined_pointcloud_topic': '/perception/obstacle_points',
    'combined_pointcloud_frame': 'base_link',
    'combined_pointcloud_publish_rate': '10.0',
    'combined_pointcloud_max_age_sec': '0.5',

    # Optional driver package/launch names
    'mid360_driver_package': 'livox_ros_driver2',
    'mid360_driver_launch': 'launch_ROS2/msg_MID360_launch.py',
    'lio_package': 'fast_lio',
    'lio_executable': 'fastlio_mapping',
    'lio_config_file': 'mid360.yaml',
    'lio_odom_topic': '/lio/odometry',
    'lio_path_topic': '/lio/path',

    # MID360 mounting extrinsics on the UAV body.
    #
    # These launch arguments describe the rigid transform published as:
    #
    #   base_link -> mid360_link
    #
    # Meaning:
    #   - base_link is the UAV body frame / vehicle reference frame.
    #   - mid360_link is the MID360 point cloud frame used by /mid360/points.
    #   - x/y/z are the MID360 origin offset measured from base_link, in meters.
    #   - yaw/pitch/roll are the MID360 orientation relative to base_link, in radians.
    #
    # Coordinate convention follows ROS body ENU:
    #   - +X: forward
    #   - +Y: left
    #   - +Z: up
    #
    # Rotation convention follows tf2 static_transform_publisher argument order:
    #   x y z yaw pitch roll parent_frame child_frame
    #
    # Examples:
    #   - MID360 at the body center, same attitude:
    #       mid360_x:=0.0 mid360_y:=0.0 mid360_z:=0.0
    #       mid360_yaw:=0.0 mid360_pitch:=0.0 mid360_roll:=0.0
    #   - MID360 8 cm forward and 5 cm above the body origin:
    #       mid360_x:=0.08 mid360_y:=0.0 mid360_z:=0.05
    #
    # Do not confuse these values with FAST-LIO's extrinsic_T/extrinsic_R in
    # src/FAST_LIO_ROS2/config/mid360.yaml. That file configures the LiDAR-to-IMU
    # transform used internally by FAST-LIO. The values below configure the
    # whole MID360 sensor pose relative to the aircraft body.
    'mid360_x': '0.0',
    'mid360_y': '0.0',
    'mid360_z': '0.0',
    'mid360_yaw': '0.0',
    'mid360_pitch': '0.0',
    'mid360_roll': '0.0',
}


def _launch_arg(name, description=''):
    return DeclareLaunchArgument(
        name,
        default_value=LAUNCH_DEFAULTS[name],
        description=description,
    )


def _as_bool(value):
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def _selected_obstacle_topic(context):
    source = LaunchConfiguration('obstacle_pointcloud_source').perform(context).lower()
    oakd_topic = LaunchConfiguration('oakd_pointcloud_topic').perform(context)
    mid360_topic = LaunchConfiguration('mid360_pointcloud_topic').perform(context)
    combined_topic = LaunchConfiguration('combined_pointcloud_topic').perform(context)

    if source == 'oakd':
        return oakd_topic
    if source == 'mid360':
        return mid360_topic
    if source == 'both':
        return combined_topic
    return oakd_topic


def _source_expr(source_arg, *values):
    return PythonExpression([
        "'",
        LaunchConfiguration(source_arg),
        "'.lower() in ",
        repr(tuple(values)),
    ])


def _optional_launch(context, enabled, package_arg, launch_arg, label):
    if not enabled:
        return []

    package_name = LaunchConfiguration(package_arg).perform(context)
    launch_file = LaunchConfiguration(launch_arg).perform(context)
    if not package_name or not launch_file:
        return [LogInfo(msg=f'{label} enabled, but package or launch file is empty')]

    try:
        package_share = get_package_share_directory(package_name)
    except PackageNotFoundError:
        return [
            LogInfo(
                msg=(
                    f'{label} enabled, but package "{package_name}" was not found; '
                    'skipping optional launch'
                )
            )
        ]

    if os.path.dirname(launch_file):
        launch_path = os.path.join(package_share, launch_file)
    else:
        launch_path = os.path.join(package_share, 'launch', launch_file)
    if not os.path.exists(launch_path):
        return [
            LogInfo(
                msg=(
                    f'{label} enabled, but launch file "{launch_path}" was not found; '
                    'skipping optional launch'
                )
            )
        ]

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_path),
        )
    ]


def launch_setup(context, *args, **kwargs):
    config_file = PathJoinSubstitution([
        FindPackageShare('uav_bringup'),
        'config',
        'nav_stack.yaml',
    ])

    px4_comm_bridge_config = PathJoinSubstitution([
        FindPackageShare('px4_comm_bridge'),
        'config',
        'px4_comm_bridge.yaml',
    ])

    source = LaunchConfiguration('obstacle_pointcloud_source').perform(context).lower()
    selected_topic = _selected_obstacle_topic(context)
    odometry_source = LaunchConfiguration('odometry_source').perform(context).lower()
    enable_mid360 = (
        _as_bool(LaunchConfiguration('enable_mid360').perform(context))
        or source in ('mid360', 'both')
        or odometry_source in ('lio', 'both')
    )
    enable_lio = (
        _as_bool(LaunchConfiguration('enable_lio').perform(context))
        or odometry_source in ('lio', 'both')
    )
    launch_dwb = LaunchConfiguration('launch_dwb')

    nodes = []
    nodes.extend(
        _optional_launch(
            context,
            enable_mid360,
            'mid360_driver_package',
            'mid360_driver_launch',
            'MID360 driver',
        )
    )

    if enable_mid360 and source in ('mid360', 'both'):
        nodes.append(
            Node(
                package='nav_mapping',
                executable='livox_custom_to_pointcloud2',
                name='mid360_custom_to_pointcloud2',
                output='screen',
                parameters=[
                    {
                        'input_topic': LaunchConfiguration('mid360_custom_topic'),
                        'output_topic': LaunchConfiguration('mid360_pointcloud_topic'),
                        'frame_id': LaunchConfiguration('mid360_frame_id'),
                    }
                ],
            )
        )

    if enable_lio:
        lio_package = LaunchConfiguration('lio_package').perform(context)
        try:
            lio_share = get_package_share_directory(lio_package)
        except PackageNotFoundError:
            nodes.append(
                LogInfo(
                    msg=(
                        f'LIO enabled, but package "{lio_package}" was not found; '
                        'skipping LIO node'
                    )
                )
            )
        else:
            nodes.append(
                Node(
                    package=lio_package,
                    executable=LaunchConfiguration('lio_executable'),
                    name='fast_lio_mapping',
                    output='screen',
                    parameters=[
                        os.path.join(
                            lio_share,
                            'config',
                            LaunchConfiguration('lio_config_file').perform(context),
                        ),
                        {'use_sim_time': False},
                    ],
                    remappings=[
                        ('/Odometry', LaunchConfiguration('lio_odom_topic')),
                        ('/path', LaunchConfiguration('lio_path_topic')),
                    ],
                )
            )

    if source == 'both':
        nodes.append(
            Node(
                package='nav_mapping',
                executable='pointcloud_combiner',
                name='obstacle_pointcloud_combiner',
                output='screen',
                parameters=[
                    {
                        'primary_topic': LaunchConfiguration('oakd_pointcloud_topic'),
                        'secondary_topic': LaunchConfiguration('mid360_pointcloud_topic'),
                        'output_topic': LaunchConfiguration('combined_pointcloud_topic'),
                        'output_frame': LaunchConfiguration('combined_pointcloud_frame'),
                        'publish_rate': LaunchConfiguration(
                            'combined_pointcloud_publish_rate'
                        ),
                        'max_source_age_sec': LaunchConfiguration(
                            'combined_pointcloud_max_age_sec'
                        ),
                    }
                ],
            )
        )

    nodes.append(
        Node(
            package='nav_mapping',
            executable='local_map_builder',
            name='local_map_builder',
            output='screen',
            parameters=[config_file, {'pointcloud_topic': selected_topic}],
        )
    )

    nodes.append(
        Node(
            package='nav_planning',
            executable='se2_dwa_local_planner',
            name='se2_dwa_local_planner',
            output='screen',
            parameters=[config_file],
        )
    )

    nodes.append(
        Node(
            package='nav_planning',
            executable='dwb_bridge',
            name='dwb_bridge',
            output='screen',
            condition=IfCondition(launch_dwb),
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('nav_planning'),
                    'config',
                    'dwb_local_planner.yaml',
                ]),
            ],
            remappings=[
                ('/oakd/points', selected_topic),
                ('/tf', '/tf'),
                ('/tf_static', '/tf_static'),
                ('/odometry/filtered', '/odometry/filtered'),
                ('/nav/goal_pose', '/nav/goal_pose'),
                ('/nav/cmd_vel', '/nav/cmd_vel'),
            ],
        )
    )

    nodes.append(
        Node(
            package='nav_safety',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[config_file, {'pointcloud_topic': selected_topic}],
        )
    )

    nodes.append(
        Node(
            package='px4_comm_bridge',
            executable='px4_bridge_node',
            name='px4_comm_bridge',
            output='screen',
            parameters=[px4_comm_bridge_config],
        )
    )

    return nodes


def generate_launch_description():
    enable_gps = LaunchConfiguration('enable_gps')
    oakd_needed = _source_expr('obstacle_pointcloud_source', 'oakd', 'both')
    vio_needed = _source_expr('odometry_source', 'vio', 'both')
    enable_mid360 = LaunchConfiguration('enable_mid360')
    enable_lio = LaunchConfiguration('enable_lio')

    # ─────────────────────────────────────────────────────
    # ① OAK-D 感知层 (RGB-D + IMU 400Hz + Point Cloud 20Hz)
    # ─────────────────────────────────────────────────────
    oakd_perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('oakd_perception'),
                'launch',
                'oakd_unified.launch.py',
            ])
        ),
        launch_arguments={
            'imu_frequency': '400',
            'pointcloud_frequency': '20',
            'enable_passive_stereo': 'true',
        }.items(),
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_oakd_perception'),
            "'.lower() == 'true' or ",
            oakd_needed,
            " or ",
            vio_needed,
        ])),
    )

    # ─────────────────────────────────────────────────────
    # ② IMU 融合层 (OAK-D + external IMU)
    # ─────────────────────────────────────────────────────
    imu_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('imu_fusion'),
                'launch',
                'oakd_imu_fusion.launch.py',
            ])
        ),
        launch_arguments={
            'raw_topic': '/oakd/imu/raw',
            'fused_topic': '/oakd/imu/fused',
            'frame_id': 'oakd_imu_link',
            'imu_frequency': '400',
        }.items(),
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_imu_fusion'),
            "'.lower() == 'true' and (",
            oakd_needed,
            " or ",
            vio_needed,
            ")",
        ])),
    )

    # ─────────────────────────────────────────────────────
    # ③ VINS-Fusion 视觉惯性里程计 (VIO)
    # ─────────────────────────────────────────────────────
    vins_fusion_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('vins_fusion_ros2'),
                'launch',
                'oakd_vins.launch.py',
            ])
        ),
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('enable_vins'),
            "'.lower() == 'true' and ",
            vio_needed,
        ])),
    )

    # ─────────────────────────────────────────────────────
    # ④ 双层EKF融合 (robot_localization)
    # 说明：
    #   - EKF_odom: 融合 VINS + IMU → odom 帧 (局部坐标)
    #   - EKF_map: 融合 VINS + IMU + GPS (可选) → map 帧 (全局坐标)
    # ─────────────────────────────────────────────────────
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uav_bringup'),
                'launch',
                'ekf_launch.py',
            ])
        ),
        launch_arguments={
            'enable_gps': enable_gps,
            'enable_mid360': enable_mid360,
            'enable_lio': enable_lio,
            'odometry_source': LaunchConfiguration('odometry_source'),
            'lio_odom_topic': LaunchConfiguration('lio_odom_topic'),
            'mid360_x': LaunchConfiguration('mid360_x'),
            'mid360_y': LaunchConfiguration('mid360_y'),
            'mid360_z': LaunchConfiguration('mid360_z'),
            'mid360_yaw': LaunchConfiguration('mid360_yaw'),
            'mid360_pitch': LaunchConfiguration('mid360_pitch'),
            'mid360_roll': LaunchConfiguration('mid360_roll'),
        }.items(),
    )

    # ─────────────────────────────────────────────────────
    # ⑤ 规划+安全+控制层
    # ─────────────────────────────────────────────────────
    return LaunchDescription([
        # ─ 参数声明 ─
        DeclareLaunchArgument(
            'enable_gps',
            default_value=LAUNCH_DEFAULTS['enable_gps'],
            description='Enable GPS fusion (dual EKF + NavSat). Set false for GPS-denied.',
        ),
        DeclareLaunchArgument(
            'launch_dwb',
            default_value=LAUNCH_DEFAULTS['launch_dwb'],
            description='Launch DWB adapter. Requires Nav2 Python modules.',
        ),
        DeclareLaunchArgument(
            'enable_oakd_perception',
            default_value=LAUNCH_DEFAULTS['enable_oakd_perception'],
            description='Launch OAK-D perception pipeline.',
        ),
        DeclareLaunchArgument(
            'enable_imu_fusion',
            default_value=LAUNCH_DEFAULTS['enable_imu_fusion'],
            description='Launch OAK-D IMU pre-fusion/debug pipeline.',
        ),
        DeclareLaunchArgument(
            'enable_vins',
            default_value=LAUNCH_DEFAULTS['enable_vins'],
            description='Launch OAK-D stereo VINS-Fusion VIO pipeline.',
        ),
        DeclareLaunchArgument(
            'odometry_source',
            default_value=LAUNCH_DEFAULTS['odometry_source'],
            description='EKF odometry source: vio, lio, or both.',
        ),
        DeclareLaunchArgument(
            'enable_mid360',
            default_value=LAUNCH_DEFAULTS['enable_mid360'],
            description='Launch MID360 driver if the configured package is installed.',
        ),
        DeclareLaunchArgument(
            'enable_lio',
            default_value=LAUNCH_DEFAULTS['enable_lio'],
            description='Launch LIO and fuse /lio/odometry into EKF.',
        ),
        DeclareLaunchArgument(
            'obstacle_pointcloud_source',
            default_value=LAUNCH_DEFAULTS['obstacle_pointcloud_source'],
            description='Obstacle cloud source: oakd, mid360, or both.',
        ),
        DeclareLaunchArgument(
            'oakd_pointcloud_topic',
            default_value=LAUNCH_DEFAULTS['oakd_pointcloud_topic'],
            description='OAK-D filtered point cloud topic for obstacle mapping.',
        ),
        DeclareLaunchArgument(
            'mid360_pointcloud_topic',
            default_value=LAUNCH_DEFAULTS['mid360_pointcloud_topic'],
            description='MID360 point cloud topic for obstacle mapping.',
        ),
        DeclareLaunchArgument(
            'mid360_custom_topic',
            default_value=LAUNCH_DEFAULTS['mid360_custom_topic'],
            description='Livox CustomMsg topic from livox_ros_driver2.',
        ),
        DeclareLaunchArgument(
            'mid360_frame_id',
            default_value=LAUNCH_DEFAULTS['mid360_frame_id'],
            description='Frame assigned to converted MID360 PointCloud2.',
        ),
        DeclareLaunchArgument(
            'combined_pointcloud_topic',
            default_value=LAUNCH_DEFAULTS['combined_pointcloud_topic'],
            description='Output topic used when obstacle_pointcloud_source:=both.',
        ),
        DeclareLaunchArgument(
            'combined_pointcloud_frame',
            default_value=LAUNCH_DEFAULTS['combined_pointcloud_frame'],
            description='Frame for the combined obstacle point cloud.',
        ),
        DeclareLaunchArgument(
            'combined_pointcloud_publish_rate',
            default_value=LAUNCH_DEFAULTS['combined_pointcloud_publish_rate'],
            description='Combined obstacle cloud publish rate in Hz.',
        ),
        DeclareLaunchArgument(
            'combined_pointcloud_max_age_sec',
            default_value=LAUNCH_DEFAULTS['combined_pointcloud_max_age_sec'],
            description='Drop OAK-D/MID360 clouds older than this age in both mode.',
        ),
        DeclareLaunchArgument(
            'mid360_driver_package',
            default_value=LAUNCH_DEFAULTS['mid360_driver_package'],
            description='Optional MID360 driver package to include when enable_mid360:=true.',
        ),
        DeclareLaunchArgument(
            'mid360_driver_launch',
            default_value=LAUNCH_DEFAULTS['mid360_driver_launch'],
            description='Launch file path under the MID360 driver package share directory.',
        ),
        DeclareLaunchArgument(
            'lio_package',
            default_value=LAUNCH_DEFAULTS['lio_package'],
            description='Optional LIO package to include when enable_lio:=true.',
        ),
        DeclareLaunchArgument(
            'lio_executable',
            default_value=LAUNCH_DEFAULTS['lio_executable'],
        ),
        DeclareLaunchArgument(
            'lio_config_file',
            default_value=LAUNCH_DEFAULTS['lio_config_file'],
        ),
        DeclareLaunchArgument(
            'lio_odom_topic',
            default_value=LAUNCH_DEFAULTS['lio_odom_topic'],
            description='LIO odometry topic fused by EKF when enable_lio:=true.',
        ),
        DeclareLaunchArgument(
            'lio_path_topic',
            default_value=LAUNCH_DEFAULTS['lio_path_topic'],
        ),
        DeclareLaunchArgument('mid360_x', default_value=LAUNCH_DEFAULTS['mid360_x']),
        DeclareLaunchArgument('mid360_y', default_value=LAUNCH_DEFAULTS['mid360_y']),
        DeclareLaunchArgument('mid360_z', default_value=LAUNCH_DEFAULTS['mid360_z']),
        DeclareLaunchArgument(
            'mid360_yaw',
            default_value=LAUNCH_DEFAULTS['mid360_yaw'],
        ),
        DeclareLaunchArgument(
            'mid360_pitch',
            default_value=LAUNCH_DEFAULTS['mid360_pitch'],
        ),
        DeclareLaunchArgument(
            'mid360_roll',
            default_value=LAUNCH_DEFAULTS['mid360_roll'],
        ),

        # ─ 启动顺序（感知 → 定位 → 规划/安全/控制） ─
        oakd_perception_launch,
        imu_fusion_launch,
        vins_fusion_launch,
        ekf_launch,
        OpaqueFunction(function=launch_setup),
    ])
