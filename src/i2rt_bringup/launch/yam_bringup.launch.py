#!/usr/bin/env python3
"""
Complete YAM robot bringup: hardware + visualization
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Arguments
    can_channel_arg = DeclareLaunchArgument(
        'can_channel', default_value='can0', description='CAN channel')
    gripper_type_arg = DeclareLaunchArgument(
        'gripper_type', default_value='linear_4310', description='Gripper type')
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true', description='Launch RViz')

    # Hardware interface
    yam_hardware = IncludeLaunchDescription(
        # launches the file in the description according to the path provided in the arguments
        PythonLaunchDescriptionSource([
        # specifies launch file is a python launch file
            PathJoinSubstitution([
                FindPackageShare('i2rt_yam_driver'),
                'launch',
                'yam_hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'can_channel': LaunchConfiguration('can_channel'),
            'gripper_type': LaunchConfiguration('gripper_type'),
        }.items()
    )
    # so using this line launches the yam_hardware.launch.py parellel with the current code and then
    # pass the 2 arguments can_channel and also gripper_type into it.
    

    # URDF/Robot description
    urdf_file = PathJoinSubstitution([
        FindPackageShare('i2rt_description'),
        'urdf',
        ['yam_', LaunchConfiguration('gripper_type'), '.urdf.xacro']
    ])
    # finds the i2rt_description package and go into its urdf folder and find the corresponding gripper_type.urdf.xacro and the gripper type is the
    # above argument.

    robot_description = Command(['xacro ', urdf_file])
    # processes the xacro file and generates the robot description

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # when we use Node and the package is 'robot_state_publisher' we are running a new node called
    # 'robot state publisher' and we send the robot description into it, so we are telling this node
    # robot structure of our robot and then publishes the tf2 frames of our robot

    # RViz
    rviz_config = PathJoinSubstitution([
        FindPackageShare('i2rt_description'),
        'rviz',
        'moveit.rviz'
    ])
    # specify where the rviz file path is

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )
    # runs and displays the rviz of our robot

    return LaunchDescription([
        can_channel_arg,
        gripper_type_arg,
        use_rviz_arg,
        yam_hardware,
        robot_state_publisher,
        rviz
    ])
