from setuptools import find_packages, setup

package_name = 'nav_local'

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
    description='Local 2D/2.5D navigation prototype',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'local_map_builder = nav_local.local_map_builder:main',
        'local_planner = nav_local.local_planner:main',
        'px4_offboard_ctrl = nav_local.px4_offboard_ctrl:main',
        'safety_monitor = nav_local.safety_monitor:main',
    ]},
)
