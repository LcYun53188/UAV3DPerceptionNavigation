from glob import glob
from setuptools import setup

setup(
    name='uav_target_perception', version='0.1.0', packages=['uav_target_perception'],
    data_files=[('share/ament_index/resource_index/packages', ['resource/uav_target_perception']),
                ('share/uav_target_perception', ['package.xml']),
                ('share/uav_target_perception/launch', glob('launch/*.launch.py'))],
    install_requires=['setuptools', 'numpy'], zip_safe=True,
    entry_points={'console_scripts': [
        'person_mapper = uav_target_perception.node:main',
        'person_model = uav_target_perception.detector:main']},
)
