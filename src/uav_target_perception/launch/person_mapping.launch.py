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
                    snapshot_path='', use_sim_time='true', retain_history='true',
                    history_radius_m='0.6', history_max_records='200', rotation_augmentation='false')
    types = dict(use_sim_time=bool, retain_history=bool, rotation_augmentation=bool,
                 history_radius_m=float, history_max_records=int)
    parameters = {key: ParameterValue(LaunchConfiguration(key), value_type=types.get(key, str))
                  for key in defaults}
    return LaunchDescription([
        *[DeclareLaunchArgument(key, default_value=value) for key, value in defaults.items()],
        Node(package='uav_target_perception', executable='person_mapper', output='screen', parameters=[parameters]),
    ])
