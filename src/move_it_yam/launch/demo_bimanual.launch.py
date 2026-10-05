#!/usr/bin/env python3
"""
YAM robot MoveIt demo with hardware control and visualization
"""

import copy
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gripper_type_follower",
            default_value="linear_4310",
        ),
        DeclareLaunchArgument(
            "can_channel_left",
            default_value="can0",
            description="CAN channel for motor communication",
        ),
        DeclareLaunchArgument(
            "can_channel_right",
            default_value="can1",
            description="CAN channel for motor communication",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz for visualization",
        ),
        DeclareLaunchArgument(
            "use_servo",
            default_value="true",
            description="Launch MoveIt Servo and absolute-pose trackers",
        ),
    ]

    gripper_type_follower = LaunchConfiguration("gripper_type_follower")
    can_channel_left = LaunchConfiguration("can_channel_left")
    can_channel_right = LaunchConfiguration("can_channel_right")
    use_rviz = LaunchConfiguration("use_rviz")
    use_servo = LaunchConfiguration("use_servo")

    # Include hardware bringup (yam_hardware_interface)
    # This connects to real YAM hardware via CAN bus
    yam_left_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('i2rt_yam_driver'),
                'launch',
                'yam_hardware_bimanual.launch.py'
            ])
        ]),
        launch_arguments={
            'can_channel': can_channel_left,
            'gripper_type': gripper_type_follower,
            'robot_name': 'yam_left',
        }.items()
    )

    yam_right_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('i2rt_yam_driver'),
                'launch',
                'yam_hardware_bimanual.launch.py'
            ])
        ]),
        launch_arguments={
            'can_channel': can_channel_right,
            'gripper_type': gripper_type_follower,
            'robot_name': 'yam_right',
        }.items()
    )

    disp_tf_node = Node(
        package='i2rt_yam_driver',
        executable='disp_tf',
        output='screen',
    )

    # MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("output", package_name="move_it_yam")
        .robot_description(file_path="config/bimanual/output.urdf.xacro")
        .robot_description_semantic(file_path="config/bimanual/output.srdf")
        .robot_description_kinematics(
            file_path="config/bimanual/kinematics.yaml"
        )
        .joint_limits(file_path="config/bimanual/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/bimanual/moveit_controllers.yaml"
        )
        .pilz_cartesian_limits(
            file_path="config/bimanual/pilz_cartesian_limits.yaml"
        )
        .planning_pipelines(
            pipelines=["pilz_industrial_motion_planner", "ompl"]
        )
        .to_moveit_configs()
    )

    servo_config_path = os.path.join(
        get_package_share_directory("move_it_yam"), "config", "servo.yaml"
    )
    with open(servo_config_path, "r", encoding="utf-8") as servo_config_file:
        shared_servo_config = yaml.safe_load(servo_config_file)

    left_servo_config = copy.deepcopy(shared_servo_config)
    left_servo_config.update({
        "move_group_name": "left_yam_arm",
        "planning_frame": "left_base",
        "ee_frame_name": "left_grasp_point",
        "robot_link_command_frame": "left_base",
        "command_out_topic": "/yam_left/servo_joint_command",
    })
    right_servo_config = copy.deepcopy(shared_servo_config)
    right_servo_config.update({
        "move_group_name": "right_yam_arm",
        "planning_frame": "right_base",
        "ee_frame_name": "right_grasp_point",
        "robot_link_command_frame": "right_base",
        "command_out_topic": "/yam_right/servo_joint_command",
    })

    left_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="left_servo",
        output="screen",
        parameters=[
            {"moveit_servo": left_servo_config},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_servo),
    )
    right_servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="right_servo",
        output="screen",
        parameters=[
            {"moveit_servo": right_servo_config},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_servo),
    )

    left_pose_tracker = Node(
        package="move_it_yam",
        executable="pose_to_twist",
        name="left_pose_tracker",
        output="screen",
        parameters=[{
            "target_topic": "/yam_left/target_pose",
            "command_topic": "/left_servo/delta_twist_cmds",
            "servo_start_service": "/left_servo/start_servo",
            "planning_frame": "left_base",
            "ee_frame": "left_grasp_point",
        }],
        condition=IfCondition(use_servo),
    )
    right_pose_tracker = Node(
        package="move_it_yam",
        executable="pose_to_twist",
        name="right_pose_tracker",
        output="screen",
        parameters=[{
            "target_topic": "/yam_right/target_pose",
            "command_topic": "/right_servo/delta_twist_cmds",
            "servo_start_service": "/right_servo/start_servo",
            "planning_frame": "right_base",
            "ee_frame": "right_grasp_point",
        }],
        condition=IfCondition(use_servo),
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description]
    )

    # Make MoveIt resample trajectory points more densely
    moveit_params = moveit_config.to_dict()

    moveit_params.setdefault("ompl", {})
    moveit_params["ompl"]["resample_dt"] = 0.01

    moveit_params["pilz_industrial_motion_planner"]["sampling_time"] = 0.01

    # Move group node
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    # RViz node
    rviz_cfg = os.path.join(
        get_package_share_directory("move_it_yam"),
        "config",
        "bimanual",
        "moveit_bimanual.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_cfg],
        parameters=[moveit_params],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        declared_arguments
        + [
            yam_left_launch,
            yam_right_launch,
            robot_state_publisher,
            disp_tf_node,
            run_move_group_node,
            left_servo_node,
            right_servo_node,
            left_pose_tracker,
            right_pose_tracker,
            rviz_node,
        ]
    )
