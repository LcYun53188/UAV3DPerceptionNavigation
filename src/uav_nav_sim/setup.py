from setuptools import setup
setup(name='uav_nav_sim', version='0.1.0', packages=['uav_nav_sim'],
 data_files=[('share/ament_index/resource_index/packages', ['resource/uav_nav_sim']),
             ('share/uav_nav_sim', ['package.xml'])],
 install_requires=['setuptools'], zip_safe=True,
 entry_points={'console_scripts': ['map_session = uav_nav_sim.map_session:main',
 'planning_sources = uav_nav_sim.planning_sources_node:main',
 'planning_context = uav_nav_sim.planning_context_node:main',
 'gazebo_executor = uav_nav_sim.executor:main', 'map_bundle = uav_nav_sim.map_cli:main',
 'rviz_goal = uav_nav_sim.rviz_goal:main']})
