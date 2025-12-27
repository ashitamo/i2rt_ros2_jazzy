#!/usr/bin/env python3
"""
Launch YAM hardware interface node
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    can_channel_arg = DeclareLaunchArgument(
        'can_channel',
        default_value='can0',
        description='CAN bus channel (can0, can1, etc.)'
    )

    gripper_type_arg = DeclareLaunchArgument(
        'gripper_type',
        default_value='crank_4310',
        description='Gripper type: crank_4310, linear_3507, linear_4310, yam_teaching_handle'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='yam',
        description='Robot name (namespace)'
    )

    # Get config file
    config_file = PathJoinSubstitution([
        FindPackageShare('i2rt_yam_driver'),
        'config',
        'yam_params.yaml'
    ])

    # Hardware interface node
    yam_hardware_node = Node(
        package='i2rt_yam_driver',
        executable='yam_hardware_interface',
        name='yam_hardware_interface',
        output='screen',
        parameters=[
            config_file,
            {
                'can_channel': LaunchConfiguration('can_channel'),
                'gripper_type': LaunchConfiguration('gripper_type'),
                'robot_name': LaunchConfiguration('robot_name'),
            }
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        can_channel_arg,
        gripper_type_arg,
        robot_name_arg,
        yam_hardware_node
    ])
