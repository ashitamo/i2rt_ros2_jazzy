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
        default_value='linear_4310',
        description='Gripper type: linear_4310, linear_3507, linear_4310, yam_teaching_handle'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='yam_left',
        description='Robot name (namespace)'
    )

    # Get config file
    config_file = PathJoinSubstitution([
        FindPackageShare('i2rt_yam_driver'),
        'config',
        'yam_params.yaml'
    ])
    # finds the yam_params.yaml file which is in the i2rt_yam_driver package and inside the folder
    # called "config" and in the config find the yaml file called yam_params

    # Hardware interface node
    yam_hardware_node = Node(
        package='i2rt_yam_driver',
        executable='yam_hardware_interface_bimanual',
        namespace=LaunchConfiguration('robot_name'),
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
    # this launches the yam_hardware_interface as a node which is in the i2rt_yam_driver package
    # and then input the can_channel, gripper_type and also robot_name to it for further use.
    # the output='screen' tells the ros2 to directly display the node outputs on the terminal screen
    # this is useful in seeing what the node is doing and also for debugging

    return LaunchDescription([
        can_channel_arg,
        gripper_type_arg,
        robot_name_arg,
        yam_hardware_node
    ])
