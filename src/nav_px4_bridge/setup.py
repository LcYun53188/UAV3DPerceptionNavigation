from setuptools import find_packages, setup

package_name = 'nav_px4_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='Bridge navigation commands to PX4 offboard control',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'px4_offboard_ctrl = nav_px4_bridge.px4_offboard_ctrl:main',
    ]},
)