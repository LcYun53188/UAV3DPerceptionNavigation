"""
Launch file for dual EKF sensor fusion with optional GPS.

Modes:
  enable_gps:=false (default):
    - EKF_odom: odom→base_link (selected odometry + IMU)
    - Static TF: map→odom (identity, map=odom)

  enable_gps:=true:
    - EKF_odom: odom→base_link (selected odometry + IMU)
    - EKF_map:  map→odom (selected odometry + IMU + GPS correction)
    - NavSat Transform: GPS WGS84→ENU

Both modes publish static transforms:
  - base_link → oakd_imu_link (camera mount extrinsic)
  - base_link → gps_link (GPS antenna offset)

Output topics:
  - /odometry/local  (from EKF_odom)
  - /odometry/global (from EKF_map, GPS mode only)

Optional:
  odometry_source:=vio|lio|both selects the odometry source fused by EKF.
  enable_mid360:=true publishes base_link→mid360_link static extrinsics.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    enable_gps = LaunchConfiguration('enable_gps').perform(context).lower() == 'true'
    enable_lio = LaunchConfiguration('enable_lio').perform(context).lower() == 'true'
    odometry_source = LaunchConfiguration('odometry_source').perform(context).lower()
    enable_mid360 = (
        LaunchConfiguration('enable_mid360').perform(context).lower() == 'true'
    )
    lio_odom_topic = LaunchConfiguration('lio_odom_topic').perform(context)

    if odometry_source not in ('vio', 'lio', 'both'):
        raise RuntimeError(
            f'Invalid odometry_source "{odometry_source}". Expected vio, lio, or both.'
        )

    config_file = PathJoinSubstitution([
        FindPackageShare('uav_bringup'),
        'config',
        'dual_ekf.yaml',
    ])

    nodes = []
    ekf_odom_parameters = [config_file]

    # dual_ekf.yaml defaults odom0 to /vio/odometry. Override it when LIO is
    # selected as the primary odometry source.
    if odometry_source == 'lio':
        ekf_odom_parameters.append({
            'odom0': lio_odom_topic,
            'odom0_config': [
                True, True, True,
                False, False, False,
                True, True, True,
                False, False, False,
                False, False, False,
            ],
            'odom0_queue_size': 10,
            'odom0_nodelay': True,
            'odom0_differential': False,
            'odom0_relative': False,
        })
    elif odometry_source == 'both' or enable_lio:
        ekf_odom_parameters.append({
            'odom1': lio_odom_topic,
            'odom1_config': [
                True, True, True,
                False, False, False,
                True, True, True,
                False, False, False,
                False, False, False,
            ],
            'odom1_queue_size': 10,
            'odom1_nodelay': True,
            'odom1_differential': False,
            'odom1_relative': False,
        })

    # ── 始终启动：EKF_odom (odom→base_link) ──
    nodes.append(Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=ekf_odom_parameters,
        remappings=[
            ('odometry/filtered', 'odometry/local'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ],
    ))

    if enable_gps:
        ekf_map_parameters = [config_file]
        if odometry_source == 'lio':
            ekf_map_parameters.append({
                'odom0': lio_odom_topic,
                'odom0_config': [
                    True, True, True,
                    False, False, False,
                    True, True, True,
                    False, False, False,
                    False, False, False,
                ],
                'odom0_queue_size': 10,
                'odom0_nodelay': True,
                'odom0_differential': False,
                'odom0_relative': False,
            })
        elif odometry_source == 'both' or enable_lio:
            ekf_map_parameters.append({
                'odom2': lio_odom_topic,
                'odom2_config': [
                    True, True, True,
                    False, False, False,
                    True, True, True,
                    False, False, False,
                    False, False, False,
                ],
                'odom2_queue_size': 10,
                'odom2_nodelay': True,
                'odom2_differential': False,
                'odom2_relative': False,
            })

        # ── GPS 模式：EKF_map (map→odom) ──
        nodes.append(Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=ekf_map_parameters,
            remappings=[
                ('odometry/filtered', 'odometry/global'),
            ],
        ))

        # ── GPS 模式：NavSat Transform ──
        nodes.append(Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('imu', '/px4/imu'),
                ('gps/fix', '/gps/fix'),
                ('odometry/filtered', 'odometry/global'),
            ],
        ))
    else:
        # ── 无 GPS 模式：静态 map→odom 恒等变换 ──
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_identity',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ))

    # ── 静态外参：base_link → oakd_imu_link ──
    #
    # 这组参数描述 OAK-D 作为一个整体安装在无人机机体上的位置和姿态，
    # 也就是“机体坐标系到 OAK-D IMU 坐标系”的静态 TF。
    #
    # TF 链路中的角色：
    #   - base_link：无人机机体坐标系，通常取飞控/机体中心作为原点。
    #   - oakd_imu_link：OAK-D 内置 IMU / 相机机身参考坐标系。
    #   - oakd_camera_optical_frame：OAK-D 相机光学坐标系，由
    #     oakd_perception/launch/oakd_unified.launch.py 继续从 oakd_imu_link 发布。
    #
    # 坐标约定：ROS ENU / body convention
    #   - +X：机头前方
    #   - +Y：机体左侧
    #   - +Z：机体上方
    #
    # static_transform_publisher 参数顺序：
    #   x y z yaw pitch roll parent_frame child_frame
    #
    # 参数单位：
    #   - x/y/z：米，表示 OAK-D IMU 原点相对 base_link 的平移。
    #   - yaw/pitch/roll：弧度，表示 OAK-D IMU 坐标系相对 base_link 的姿态。
    #
    # 当前默认值为全 0：
    #   - 表示 OAK-D IMU 原点与 base_link 重合。
    #   - 表示 OAK-D IMU 姿态与 base_link 完全一致。
    #
    # 实测安装后应替换这里的 6 个数。例如：
    #   - OAK-D 装在机体中心前方 10 cm：x = 0.10
    #   - OAK-D 装在机体中心右侧 4 cm：y = -0.04
    #   - OAK-D 装在机体中心上方 6 cm：z = 0.06
    #
    # 注意：
    #   - 这里配置的是“整台 OAK-D 相对飞机机体”的安装外参。
    #   - 不要把 VINS 配置文件里的 body_T_cam0/body_T_cam1 直接填到这里；
    #     body_T_cam* 描述的是 OAK-D 内部 IMU 到左右相机的外参。
    #   - 不要把 oakd_imu_link -> oakd_camera_optical_frame 的旋转直接填到这里；
    #     那是相机内部坐标轴到光学坐标轴的变换。
    #   - 如果这里不准，点云、VIO 里程计和避障障碍物会相对机体整体偏移或旋转。
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_oakd_imu_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'oakd_imu_link'],
    ))

    # ── 静态外参：base_link → gps_link ──
    # GPS 天线与飞控中心偏移（通常较小，可设为全零）
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_gps_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'gps_link'],
    ))

    if enable_mid360 or enable_lio or odometry_source in ('lio', 'both'):
        # ── 静态外参：base_link → mid360_link ──
        #
        # 这组参数描述 MID360 作为一个整体安装在无人机机体上的位置和姿态，
        # 也就是“机体坐标系到 MID360 点云坐标系”的静态 TF。
        #
        # 坐标系含义：
        #   - base_link：无人机机体坐标系，通常取飞控/机体中心作为原点。
        #   - mid360_link：MID360 点云坐标系，/mid360/points 使用这个 frame_id。
        #
        # 坐标约定：ROS ENU / body convention
        #   - +X：机头前方
        #   - +Y：机体左侧
        #   - +Z：机体上方
        #
        # 参数单位和顺序：
        #   - mid360_x/y/z：单位为米，表示 MID360 原点相对 base_link 的平移。
        #   - mid360_yaw/pitch/roll：单位为弧度，表示 MID360 相对 base_link 的姿态。
        #   - static_transform_publisher 的参数顺序为：
        #       x y z yaw pitch roll parent_frame child_frame
        #
        # 正负方向示例：
        #   - MID360 装在机体中心前方 8 cm：mid360_x:=0.08
        #   - MID360 装在机体中心右侧 3 cm：mid360_y:=-0.03
        #   - MID360 装在机体中心上方 5 cm：mid360_z:=0.05
        #
        # 注意：
        #   - 这里配置的是“传感器相对飞机机体”的安装外参。
        #   - FAST-LIO 配置文件里的 extrinsic_T/extrinsic_R 是“LiDAR 和 IMU 之间”
        #     的外参，含义不同，不应直接填到这里。
        #   - 如果这组安装外参不准，地图/避障点云会相对机体发生整体偏移或旋转，
        #     表现为障碍物位置不对、局部地图抖动、规划器绕障距离异常。
        nodes.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_mid360_link',
            arguments=[
                LaunchConfiguration('mid360_x'),
                LaunchConfiguration('mid360_y'),
                LaunchConfiguration('mid360_z'),
                LaunchConfiguration('mid360_yaw'),
                LaunchConfiguration('mid360_pitch'),
                LaunchConfiguration('mid360_roll'),
                'base_link',
                'mid360_link',
            ],
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_gps',
            default_value='false',
            description='Enable GPS fusion (dual EKF + NavSat). Set false for GPS-denied.',
        ),
        DeclareLaunchArgument(
            'enable_mid360',
            default_value='false',
            description='Publish base_link to mid360_link static TF.',
        ),
        DeclareLaunchArgument(
            'enable_lio',
            default_value='false',
            description='Start compatibility switch for LIO-related static TF.',
        ),
        DeclareLaunchArgument(
            'odometry_source',
            default_value='vio',
            description='EKF odometry source: vio, lio, or both.',
        ),
        DeclareLaunchArgument(
            'lio_odom_topic',
            default_value='/lio/odometry',
            description='LIO odometry topic.',
        ),
        DeclareLaunchArgument('mid360_x', default_value='0.0'),
        DeclareLaunchArgument('mid360_y', default_value='0.0'),
        DeclareLaunchArgument('mid360_z', default_value='0.0'),
        DeclareLaunchArgument('mid360_yaw', default_value='0.0'),
        DeclareLaunchArgument('mid360_pitch', default_value='0.0'),
        DeclareLaunchArgument('mid360_roll', default_value='0.0'),
        OpaqueFunction(function=launch_setup),
    ])
