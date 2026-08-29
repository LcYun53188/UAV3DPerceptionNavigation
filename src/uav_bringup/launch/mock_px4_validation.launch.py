"""Validation launch for running the full navigation stack without PX4 firmware.

This launch starts the regular nav stack, fake PX4 sensor publishers, and a PX4
mock node that publishes /fmu/out/vehicle_status while consuming
/fmu/in/vehicle_command.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    enable_gps = LaunchConfiguration('enable_gps')

    nav_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('uav_bringup'),
                'launch',
                'nav_stack.launch.py',
            ])
        ),
        launch_arguments={'enable_gps': enable_gps}.items(),
    )

    px4_mock_node = Node(
        package='px4_comm_bridge',
        executable='px4_mock_node',
        name='px4_mock_node',
        output='screen',
    )

    fake_px4_sensors = Node(
        package='px4_comm_bridge',
        executable='fake_px4_sensors',
        name='fake_px4_sensors',
        output='screen',
        parameters=[{
            # Harmless in no-GPS mode; required when enable_gps:=true.
            'publish_gps': True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_gps',
            default_value='false',
            description='Enable GPS fusion (dual EKF + NavSat). Set false for GPS-denied.',
        ),
        nav_stack_launch,
        px4_mock_node,
        fake_px4_sensors,
    ])
