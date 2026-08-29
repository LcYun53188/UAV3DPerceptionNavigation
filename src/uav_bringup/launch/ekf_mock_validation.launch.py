"""Minimal robot_localization validation without PX4, OAK-D, or VINS hardware."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_gps = LaunchConfiguration('enable_gps')
    rate_hz = LaunchConfiguration('rate_hz')
    motion_enabled = LaunchConfiguration('motion_enabled')

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uav_bringup'),
                'launch',
                'ekf_launch.py',
            ])
        ),
        launch_arguments={'enable_gps': enable_gps}.items(),
    )

    # ExecuteProcess is used here because this workspace's ament_python install
    # path places console scripts on PATH instead of the ROS libexec directory.
    fake_px4_sensors = ExecuteProcess(
        cmd=[
            'fake_px4_sensors',
            '--ros-args',
            '-r', '__node:=fake_px4_sensors',
            '-p', ['rate_hz:=', rate_hz],
            '-p', 'publish_gps:=true',
            '-p', 'publish_px4_odom:=false',
        ],
        output='screen',
    )

    fake_vio_odometry = ExecuteProcess(
        cmd=[
            'fake_vio_odometry',
            '--ros-args',
            '-r', '__node:=fake_vio_odometry',
            '-p', ['rate_hz:=', rate_hz],
            '-p', ['motion_enabled:=', motion_enabled],
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_gps',
            default_value='false',
            description='Enable the map EKF and navsat_transform with fake GPS.',
        ),
        DeclareLaunchArgument(
            'rate_hz',
            default_value='50.0',
            description='Publish rate for fake PX4 and VIO sensor inputs.',
        ),
        DeclareLaunchArgument(
            'motion_enabled',
            default_value='true',
            description='Publish deterministic VIO motion instead of a static pose.',
        ),
        ekf_launch,
        fake_px4_sensors,
        fake_vio_odometry,
    ])
