#!/usr/bin/env python3
"""
Launch file to visualize YAM robot in RViz
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare arguments
    gripper_type_arg = DeclareLaunchArgument(
        'gripper_type',
        default_value='crank_4310',
        description='Gripper type: crank_4310, linear_3507, linear_4310'
    )

    # Get package directories
    pkg_description = get_package_share_directory('i2rt_description')

    # URDF file path based on gripper type
    gripper_type = LaunchConfiguration('gripper_type')
    urdf_file = PathJoinSubstitution([
        FindPackageShare('i2rt_description'),
        'urdf',
        ['yam_', gripper_type, '.urdf.xacro']
    ])

    # Process xacro file
    robot_description = Command(['xacro ', urdf_file])

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )

    # Joint state publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('i2rt_description'),
        'rviz',
        'view_robot.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        gripper_type_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
