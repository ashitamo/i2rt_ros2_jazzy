from setuptools import setup

package_name = "i2rt_yam_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="I2RT Support",
    maintainer_email="support@i2rt.com",
    description="Dedicated ROS 2 leader-follower teleoperation for YAM arms",
    license="MIT",
    entry_points={
        "console_scripts": [
            "teleop_hardware_interface = i2rt_yam_teleop.teleop_hardware_interface:main",
            "leader_follower_teleop = i2rt_yam_teleop.leader_follower_teleop:main",
            "record_data = i2rt_yam_teleop.record:main"
        ],
    },
)
