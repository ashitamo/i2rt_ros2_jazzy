#!/usr/bin/env python3
"""Dedicated YAM leader-follower teleoperation without MoveIt or Servo."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "leader",
            default_value="can1",
            description="CAN channel connected to the teaching-handle leader",
        ),
        DeclareLaunchArgument(
            "follower",
            default_value="can0",
            description="CAN channel connected to the follower arm",
        ),
        DeclareLaunchArgument(
            "follower_gripper_type",
            default_value="linear_4310",
            description="Follower gripper type",
        ),
        DeclareLaunchArgument(
            "bilateral_kp", default_value="0.05",
            description="Leader bilateral Kp scale; 0 disables force feedback",
        ),
        DeclareLaunchArgument(
            "slow_sync_duration", default_value="3.0",
            description="Seconds used to align follower after button press",
        ),
        DeclareLaunchArgument(
            "command_frequency", default_value="100.0",
            description="Teleoperation command rate in Hz",
        ),
        DeclareLaunchArgument(
            "record_output_directory", default_value="/home/lab606/ros2_ws/src/i2rt_ros2_humble/src/i2rt_yam_teleop/teleop_data",
            description="Root directory for synchronized teleoperation recordings",
        ),
    ]

    leader_can_channel = LaunchConfiguration("leader")
    follower_can_channel = LaunchConfiguration("follower")
    follower_gripper_type = LaunchConfiguration("follower_gripper_type")
    bilateral_kp = LaunchConfiguration("bilateral_kp")
    slow_sync_duration = LaunchConfiguration("slow_sync_duration")
    command_frequency = LaunchConfiguration("command_frequency")
    record_output_directory = LaunchConfiguration("record_output_directory")

    leader_hardware = Node(
        package="i2rt_yam_teleop",
        executable="teleop_hardware_interface",
        name="yam_leader_hardware_interface",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "can_channel": leader_can_channel,
                "gripper_type": "yam_teaching_handle",
                "control_frequency": 250.0,
                "joint_command_timeout": 0.2,
                "use_feedback_timestamp": True,
                "gravity_comp_enabled": True,
                "gravity_comp_factor": 1.3,
                "zero_gravity_mode": True,
                "robot_name": "yam_leader",
                "publish_tf": False,
                "teleop_role": "leader",
            }
        ],
    )
    # so the Node function is used to open existing node using a python script for example here it does to the package
    # i2rt_yam_teleop and find the executable node which is the teleop_hardware_interface and then it will launch the node
    # the name of the node is yam_leader_hardware_interface. And output = "screen" means the output of the node will be 
    # printed on the screen. emulate_tty = True means that the node will be run in a pseudo-terminal. 
    
    follower_hardware = Node(
        package="i2rt_yam_teleop",
        executable="teleop_hardware_interface",
        name="yam_follower_hardware_interface",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "can_channel": follower_can_channel,
            "gripper_type": follower_gripper_type,
            "control_frequency": 250.0,
            "joint_command_timeout": 0.2,
            "use_feedback_timestamp": True,
            "gravity_comp_enabled": True,
            "gravity_comp_factor": 1.3,
            "zero_gravity_mode": True,
            "robot_name": "yam_follower",
            "publish_tf": False,
            "teleop_role": "follower",
        }],
        remappings=[("/joint_states", "/yam_follower/joint_states")],
    )

    teleop_coordinator = Node(
        package="i2rt_yam_teleop",
        executable="leader_follower_teleop",
        name="leader_follower_teleop",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "leader_name": "yam_leader",
            "follower_name": "yam_follower",
            "bilateral_kp": ParameterValue(bilateral_kp, value_type=float),
            "slow_sync_duration": ParameterValue(
                slow_sync_duration, value_type=float
            ),
            "command_frequency": ParameterValue(
                command_frequency, value_type=float
            ),
        }],
    )

    data_recorder = Node(
        package="i2rt_yam_teleop",
        executable="record_data",
        name="record_data",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "output_directory": record_output_directory,
        }],
    )

    # Use the standard launch file provided by the apt-installed RealSense package.
    # rs_launch_path = os.path.join(
    #     get_package_share_directory('realsense2_camera'),
    #     'launch',
    #     'rs_launch.py',
    # )

    # camera_include = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(rs_launch_path),
    #     launch_arguments={
    #         'camera_name': 'camera',
    #         'camera_namespace': 'camera',
    #         'enable_infra1': 'true',
    #     }.items(),
    # )

    return LaunchDescription(
        declared_arguments
        + [
            leader_hardware,
            follower_hardware,
            teleop_coordinator,
            data_recorder,
            # camera_include,
        ]
    )
