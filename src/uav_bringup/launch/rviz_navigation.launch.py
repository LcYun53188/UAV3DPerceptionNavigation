"""RViz planar goal selection with configurable map-frame goal height."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    common = {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('goal_height', default_value='1.2'),
        Node(package='uav_nav_sim', executable='rviz_goal', output='screen',
             parameters=[common, {'height': ParameterValue(
                 LaunchConfiguration('goal_height'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2', output='screen', parameters=[common],
             arguments=['-d', PathJoinSubstitution([
                 FindPackageShare('uav_bringup'), 'rviz', 'uav_navigation.rviz'])]),
    ])
