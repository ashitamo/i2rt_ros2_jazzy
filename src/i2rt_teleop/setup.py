from setuptools import setup

package_name = 'i2rt_teleop'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='I2RT Support',
    maintainer_email='support@i2rt.com',
    description='Teleoperation nodes for I2RT robots',
    license='MIT',
    entry_points={
        'console_scripts': [
            'leader_follower = i2rt_teleop.leader_follower:main',
        ],
    },
)
