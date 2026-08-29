"""Compatibility launch file that forwards to the split packages."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav_mapping',
            executable='local_map_builder',
            name='local_map_builder',
            output='screen',
        ),
        Node(
            package='nav_planning',
            executable='local_planner',
            name='local_planner',
            output='screen',
        ),
        Node(
            package='px4_comm_bridge',
            executable='px4_bridge_node',
            name='px4_comm_bridge',
            output='screen',
        ),
        Node(
            package='nav_safety',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
        ),
    ])
