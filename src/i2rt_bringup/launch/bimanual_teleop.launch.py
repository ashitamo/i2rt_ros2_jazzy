#!/usr/bin/env python3
"""
Bimanual teleoperation: leader + follower arms
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    leader_channel_arg = DeclareLaunchArgument(
        'leader_channel', default_value='can1', description='Leader arm CAN channel')
    follower_channel_arg = DeclareLaunchArgument(
        'follower_channel', default_value='can0', description='Follower arm CAN channel')

    # Leader arm (with teaching handle)
    leader_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('i2rt_yam_driver'),
                'launch',
                'yam_hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'can_channel': LaunchConfiguration('leader_channel'),
            'gripper_type': 'yam_teaching_handle',
            'robot_name': 'yam_leader',
        }.items()
    )

    # Follower arm
    follower_arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('i2rt_yam_driver'),
                'launch',
                'yam_hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'can_channel': LaunchConfiguration('follower_channel'),
            'gripper_type': 'crank_4310',
            'robot_name': 'yam_follower',
        }.items()
    )

    return LaunchDescription([
        leader_channel_arg,
        follower_channel_arg,
        leader_arm,
        follower_arm
    ])
