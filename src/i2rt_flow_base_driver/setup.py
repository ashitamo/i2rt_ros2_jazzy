from setuptools import setup
import os
from glob import glob

package_name = 'i2rt_flow_base_driver'

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
    description='ROS2 driver node for I2RT Flow Base mobile platform',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flow_base_node = i2rt_flow_base_driver.flow_base_node:main',
        ],
    },
)
