"""Gazebo-only EGO + nvblox pipeline. No hardware drivers or flight controller."""
from pathlib import Path
import hashlib
import math
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess, AppendEnvironmentVariable, RegisterEventHandler, EmitEvent, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('uav_bringup'))
    world = Path(LaunchConfiguration('world').perform(context)).resolve()
    world_name = ET.parse(world).getroot().find('world').get('name')
    model = share/'gazebo/models/uav_quad_mid360/model.sdf'
    config = share/'config/uav_ego_nvblox.yaml'
    map_profile = LaunchConfiguration('map_transport_profile').perform(context)
    map_environment = ({'FASTRTPS_DEFAULT_PROFILES_FILE': map_profile,
                        'FASTDDS_DEFAULT_PROFILES_FILE': map_profile} if map_profile else {})
    scene_id = hashlib.sha256(world.read_bytes()+model.read_bytes()+config.read_bytes()).hexdigest()
    extent = float(LaunchConfiguration('map_extent').perform(context))
    if not math.isfinite(extent) or extent <= 0:
        raise ValueError('map_extent must be a finite positive half-width in meters')
    if extent != 5.0:
        scene_id = hashlib.sha256(f'{scene_id}:extent={extent}'.encode()).hexdigest()
    build_id = hashlib.sha256((share/'config/algorithm_versions.json').read_bytes()).hexdigest()
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in ('mapping','localization'):
        raise ValueError('mode must be mapping or localization')
    gui = LaunchConfiguration('gui').perform(context).lower() == 'true'
    common = {'use_sim_time': True}
    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', *([] if gui else ['-s','--headless-rendering']), str(world)], output='screen')
    return [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(share/'gazebo/models')),
        gazebo,
        RegisterEventHandler(OnProcessExit(target_action=gazebo, on_exit=[EmitEvent(event=Shutdown(reason='Gazebo exited'))])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/gazebo_harmonic_nav.launch.py')),
          launch_arguments={'world':str(world), 'launch_gazebo':'false',
                            'launch_mid360':'false','launch_odometry_velocity_estimator':'false'}.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gazebo_pose_service_bridge',
             arguments=[f'/world/{world_name}/set_pose@ros_gz_interfaces/srv/SetEntityPose'], parameters=[common]),
        Node(package='uav_bringup', executable='gazebo_odometry_velocity.py',
             parameters=[common, {'output_topic':'/uav/localization/odometry'}]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='sim_map_odom',
             arguments=['--frame-id','map','--child-frame-id','odom'], parameters=[common]),
        Node(package='nvblox_ros', executable='nvblox_node', name='nvblox_node', output='screen',
             parameters=[str(config), common,
                         {'layer_visualization_exclusion_radius_m': -1.0,
                          'layer_visualization_exclusion_height_m': -1.0}],
             arguments=['--ros-args', '--log-level', 'warn'],
             remappings=[('camera_0/depth/image','/uav/mapping/depth'),
                         ('camera_0/depth/camera_info','/uav/mapping/camera_info'),
                         ('camera_0/color/image','/uav/mapping/color'),
                         ('camera_0/color/camera_info','/uav/mapping/color_camera_info')]),
        Node(package='uav_nav_sim', executable='map_session', output='screen',
             additional_env=map_environment,
             parameters=[common, {'mode':mode, 'scene_id':scene_id, 'build_id':build_id,
                         'aabb_min':[-extent, -extent, 0.0],
                         'aabb_size':[2*extent, 2*extent, 4.0]}]),
        Node(package='uav_nav_sim', executable='gazebo_executor', output='screen',
             parameters=[common, {'managed_goals': True, 'explore_unknown': mode == 'mapping',
                         'continuous_navigation': LaunchConfiguration('continuous_navigation').perform(context).lower() == 'true',
                         'moving_handover': LaunchConfiguration('moving_handover').perform(context).lower() == 'true',
                         'background_replan': LaunchConfiguration('background_replan').perform(context).lower() == 'true',
                         'reuse_inflight_observation': LaunchConfiguration('reuse_inflight_observation').perform(context).lower() == 'true',
                         'focused_observation': LaunchConfiguration('focused_observation').perform(context).lower() == 'true',
                         'exploration.step_radius': float(LaunchConfiguration('exploration_step').perform(context)),
                         'exploration.blocked_timeout': float(LaunchConfiguration('blocked_timeout').perform(context))}]),
        Node(package='uav_ego_adapter', executable='ego_nvblox_planner', output='screen',
             parameters=[str(config), common, {'managed_goals': True,
                         'continuous_navigation': LaunchConfiguration('continuous_navigation').perform(context).lower() == 'true'}],
             remappings=[('/uav/goal', '/uav/local_goal')]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share/'launch/rviz_navigation.launch.py')),
            launch_arguments={'goal_height': LaunchConfiguration('goal_height')}.items(),
            condition=IfCondition(LaunchConfiguration('launch_rviz'))),
    ]


def generate_launch_description():
    share = Path(get_package_share_directory('uav_bringup'))
    profile = []
    if not (os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE') or
            os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')):
        profile = [SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE',
                   str(share/'config/fastdds_large_maps.xml'))]
    return LaunchDescription([
        *profile,
        DeclareLaunchArgument('mode', default_value='mapping'),
        DeclareLaunchArgument('map_transport_profile', default_value=(
            '' if os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE') or os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE')
            else str(share/'config/fastdds_map_service.xml'))),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('launch_rviz', default_value=LaunchConfiguration('gui')),
        DeclareLaunchArgument('goal_height', default_value='1.2'),
        DeclareLaunchArgument('continuous_navigation', default_value='true'),
        DeclareLaunchArgument('moving_handover', default_value='true'),
        DeclareLaunchArgument('background_replan', default_value='true'),
        DeclareLaunchArgument('reuse_inflight_observation', default_value='true'),
        DeclareLaunchArgument('focused_observation', default_value='true'),
        # With the fixed downward camera, a 2 m horizon tends to select lower
        # viewpoints before the upper body volume at flight height is observed.
        DeclareLaunchArgument('exploration_step', default_value='3.0'),
        DeclareLaunchArgument('blocked_timeout', default_value='15.0'),
        DeclareLaunchArgument('map_extent', default_value='5.0'),
        DeclareLaunchArgument('world', default_value=str(share/'gazebo/worlds/uav_ego_lab.sdf')),
        OpaqueFunction(function=setup),
    ])
