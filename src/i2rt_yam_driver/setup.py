from setuptools import setup
import os
from glob import glob

package_name = 'i2rt_yam_driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='I2RT Support',
    maintainer_email='support@i2rt.com',
    description='ROS2 driver node for I2RT YAM robotic arm',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yam_hardware_interface = i2rt_yam_driver.yam_hardware_interface:main',
            'yam_controller = i2rt_yam_driver.yam_controller:main',
            'yam_hardware_interface_bimanual = i2rt_yam_driver.yam_hardware_interface_bimanual:main',
            'yam_controller_bimanual = i2rt_yam_driver.yam_controller_bimanual:main',
            'disp_tf = i2rt_yam_driver.disp_tf:main',
        ],
    },
)
