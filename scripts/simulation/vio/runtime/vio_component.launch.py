"""Owned single-thread SDK container, using the same parameters and source topics."""

# Support canonical direct execution and historical flat imports.
import sys as _sys
from pathlib import Path as _Path
_scripts = str(_Path(__file__).resolve().parents[3])
if _scripts not in _sys.path:
    _sys.path.insert(0, _scripts)
from support.paths import enable_imports as _enable_imports
_enable_imports()

import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def setup(context):
    parameters=json.loads(Path(LaunchConfiguration('params_file').perform(context)).read_text())['visual_slam']['ros__parameters']
    remappings=[('visual_slam/image_0','/vio/left/image'),('visual_slam/image_1','/vio/right/image'),
        ('visual_slam/camera_info_0','/vio/left/camera_info'),('visual_slam/camera_info_1','/vio/right/camera_info'),
        ('visual_slam/imu','/vio/imu')]
    if LaunchConfiguration('reset_proxy').perform(context)=='true':
        remappings.append(('visual_slam/reset','/visual_slam/internal/reset'))
    return [ComposableNodeContainer(name='vio_sdk_container',namespace='',
        package='rclcpp_components',executable='component_container',output='screen',
        composable_node_descriptions=[ComposableNode(package='isaac_ros_visual_slam',
            plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',name='visual_slam',
            parameters=[parameters],remappings=remappings)])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('params_file'),
        DeclareLaunchArgument('reset_proxy',default_value='false'),OpaqueFunction(function=setup)])
