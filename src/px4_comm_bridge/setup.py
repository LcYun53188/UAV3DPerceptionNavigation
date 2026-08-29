import os

from setuptools import setup

package_name = 'px4_comm_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/px4_comm_bridge.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='PX4 <-> ROS2 bridge for odometry/imu and control',
    license='BSD-3-Clause',
    entry_points={
        'console_scripts': [
            'px4_bridge_node = px4_comm_bridge.px4_bridge_node:main',
            'px4_mock_node = px4_comm_bridge.px4_mock_node:main',
            'fake_px4_sensors = px4_comm_bridge.fake_px4_sensors:main',
            'fake_vio_odometry = px4_comm_bridge.fake_vio_odometry:main',
        ],
    },
    options={
        'develop': {
            'script_dir': os.path.join('$base', 'lib', package_name),
        },
        'install': {
            'install_scripts': os.path.join('$base', 'lib', package_name),
        },
    },
)
