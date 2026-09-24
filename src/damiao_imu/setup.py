from setuptools import setup
setup(name='damiao_imu', version='0.1.0', packages=['damiao_imu'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/damiao_imu']),
                  ('share/damiao_imu', ['package.xml'])],
      install_requires=['setuptools', 'pyserial'],
      entry_points={'console_scripts': ['usb_driver = damiao_imu.node:main']})
