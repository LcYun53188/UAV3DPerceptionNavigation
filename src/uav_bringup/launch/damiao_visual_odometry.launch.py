"""Optional DM USB + visual-pose EKF; existing localization defaults are untouched."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def setup(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    if value('gyro_unit') not in ('rad_s', 'deg_s') or value('accel_unit') not in ('m_s2', 'g'):
        raise ValueError('Verify and set gyro_unit=rad_s|deg_s and accel_unit=m_s2|g')
    if value('start_oakd').lower() not in ('true', 'false'):
        raise ValueError('start_oakd must be true or false')
    start_oakd = value('start_oakd').lower() == 'true'
    return [
        Node(package='damiao_imu', executable='usb_driver', output='screen', parameters=[{
            'port': value('port'), 'gyro_unit': value('gyro_unit'),
            'accel_unit': value('accel_unit'), 'crc_mode': value('crc_mode'),
            'device_id': ParameterValue(LaunchConfiguration('device_id'), value_type=int),
            'gyro_variance': ParameterValue(LaunchConfiguration('gyro_variance'), value_type=float),
        }]),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='base_to_damiao_tf', arguments=[
                 '--x', value('imu_x'), '--y', value('imu_y'), '--z', value('imu_z'),
                 '--roll', value('imu_roll'), '--pitch', value('imu_pitch'),
                 '--yaw', value('imu_yaw'), '--frame-id', 'base_link',
                 '--child-frame-id', 'damiao_imu_link']),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('uav_bringup'), 'launch', 'oakd_vio.launch.py'])),
            launch_arguments={
                'launch_oakd': str(start_oakd).lower(),
                'launch_visual_slam': str(start_oakd).lower(),
                'publish_oakd_static_tf': str(start_oakd).lower(),
                'launch_robot_localization': 'true',
                'odom_guard_input_topic': value('visual_odom_topic'),
                'odom_guard_publish_rejected_as_hold': 'false',
                'filtered_odom_topic': '/uav/localization/odometry',
                'ekf_params_file': value('ekf_params_file'),
            }.items()),
    ]


def generate_launch_description():
    args = [DeclareLaunchArgument('gyro_unit', description='Verified wire unit: rad_s or deg_s'),
            DeclareLaunchArgument('accel_unit', description='Verified wire unit: m_s2 or g')]
    for name, default in dict(port='/dev/ttyACM0', device_id='1',
                              crc_mode='include_header', gyro_variance='0.0004',
                              start_oakd='true',
                              visual_odom_topic='/visual_slam/tracking/odometry',
                              imu_x='0.0', imu_y='0.0', imu_z='0.0',
                              imu_roll='0.0', imu_pitch='0.0', imu_yaw='0.0').items():
        args.append(DeclareLaunchArgument(name, default_value=default))
    args.append(DeclareLaunchArgument('ekf_params_file', default_value=PathJoinSubstitution([
        FindPackageShare('uav_bringup'), 'config', 'ekf_visual_damiao_3d.yaml'])))
    return LaunchDescription(args + [OpaqueFunction(function=setup)])
