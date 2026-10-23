"""Start optional person perception against an already running RGB-D/map graph."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = dict(model_path='', rgb_topic='/rgbd_camera/image',
                    depth_topic='/rgbd_camera/depth_image', camera_info_topic='/rgbd_camera/camera_info',
                    context_source='map_snapshot', context_topic='/uav/map/snapshot',
                    snapshot_path='', use_sim_time='true')
    parameters = {key: ParameterValue(LaunchConfiguration(key), value_type=str)
                  for key in defaults if key != 'use_sim_time'}
    parameters['use_sim_time'] = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        Node(package='uav_target_perception', executable='person_mapper', output='screen', parameters=[parameters]),
    ])
