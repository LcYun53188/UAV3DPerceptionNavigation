"""Gazebo Harmonic + ros_gz simulation for the UAV navigation stack."""

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    arena = LaunchConfiguration("arena")
    world = LaunchConfiguration("world")
    bridge_config = LaunchConfiguration("bridge_config")
    oakd_bridge_config = LaunchConfiguration("oakd_bridge_config")
    mid360_bridge_config = LaunchConfiguration("mid360_bridge_config")
    launch_bridge = LaunchConfiguration("launch_bridge")
    launch_oakd = LaunchConfiguration("launch_oakd")
    launch_mid360 = LaunchConfiguration("launch_mid360")

    gazebo_models_path = PathJoinSubstitution(
        [FindPackageShare("uav_bringup"), "gazebo", "models"]
    )
    register_gazebo_models = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH", gazebo_models_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={
            "gz_args": LaunchConfiguration("gz_args"),
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_gazebo")),
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="uav_gazebo_bridge",
        output="screen",
        parameters=[
            {
                "config_file": bridge_config,
                "use_sim_time": True,
            }
        ],
        condition=IfCondition(launch_bridge),
    )

    oakd_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="uav_gazebo_oakd_bridge",
        output="screen",
        parameters=[
            {
                "config_file": oakd_bridge_config,
                "use_sim_time": True,
            }
        ],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    mid360_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="uav_gazebo_mid360_bridge",
        output="screen",
        parameters=[
            {
                "config_file": mid360_bridge_config,
                "use_sim_time": True,
            }
        ],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_mid360)),
    )

    color_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="uav_gazebo_color_image_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["/rgbd_camera/image"],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    depth_image_bridge = Node(
        package="ros_gz_image",
        executable="image_bridge",
        name="uav_gazebo_depth_image_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["/rgbd_camera/depth_image"],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    gazebo_odometry_velocity = Node(
        package="uav_bringup",
        executable="gazebo_odometry_velocity.py",
        name="gazebo_odometry_velocity",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(
            AndSubstitution(
                launch_bridge,
                LaunchConfiguration("launch_odometry_velocity_estimator"),
            )
        ),
    )


    gazebo_camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_oakd_camera_static_tf",
        output="screen",
        arguments=[
            "--x",
            "0.18",
            "--y",
            "0.0",
            "--z",
            "0.16",
            "--yaw",
            "0.0",
            "--pitch",
            "0.31415926536",
            "--roll",
            "0.0",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "oakd_camera_link",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    gazebo_camera_optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_oakd_optical_static_tf",
        output="screen",
        arguments=[
            "--yaw",
            "-1.57079632679",
            "--pitch",
            "0.0",
            "--roll",
            "-1.57079632679",
            "--frame-id",
            "oakd_camera_link",
            "--child-frame-id",
            "oakd_camera_optical_frame",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    gazebo_oakd_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_oakd_imu_static_tf",
        output="screen",
        arguments=[
            "--frame-id",
            "oakd_camera_link",
            "--child-frame-id",
            "oakd_imu_link",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_oakd)),
    )

    gazebo_mid360_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="gazebo_mid360_static_tf",
        output="screen",
        arguments=[
            "--x",
            "0.113137085",
            "--y",
            "-0.113137085",
            "--z",
            "0.18",
            "--yaw",
            "0.7853981634",
            "--pitch",
            "0.0",
            "--roll",
            "0.5235987756",
            "--frame-id",
            "base_link",
            "--child-frame-id",
            "mid360_link",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(AndSubstitution(launch_bridge, launch_mid360)),
    )

    return LaunchDescription([
        DeclareLaunchArgument("arena", default_value="uav_harmonic_demo"),
        DeclareLaunchArgument("world", default_value=PathJoinSubstitution([
            FindPackageShare("uav_bringup"), "gazebo", "worlds",
            PythonExpression(["'", arena, ".sdf'"]),
        ])),
        DeclareLaunchArgument("gz_args", default_value=["-r -v 3 ", world]),
        DeclareLaunchArgument("launch_gazebo", default_value="true"),
        DeclareLaunchArgument("launch_bridge", default_value="true"),
        DeclareLaunchArgument("launch_oakd", default_value="true"),
        DeclareLaunchArgument("launch_mid360", default_value="false"),
        DeclareLaunchArgument("launch_odometry_velocity_estimator", default_value="true"),
        *[DeclareLaunchArgument(arg, default_value=PathJoinSubstitution([
            FindPackageShare("uav_bringup"), "gazebo", "config", filename,
        ])) for arg, filename in [
            ("bridge_config", "uav_gazebo_core_bridge.yaml"),
            ("oakd_bridge_config", "uav_gazebo_oakd_bridge.yaml"),
            ("mid360_bridge_config", "uav_gazebo_mid360_bridge.yaml"),
        ]],
        register_gazebo_models, gazebo, bridge, oakd_bridge, mid360_bridge,
        color_image_bridge, depth_image_bridge, gazebo_odometry_velocity,
        gazebo_camera_tf, gazebo_camera_optical_tf, gazebo_oakd_imu_tf, gazebo_mid360_tf,
    ])
