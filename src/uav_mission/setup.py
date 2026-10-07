from setuptools import setup

setup(name='uav_mission', version='0.1.0', packages=['uav_mission'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/uav_mission']),
                  ('share/uav_mission', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': [
          'aircraft_state = uav_mission.aircraft_state_node:main']})
