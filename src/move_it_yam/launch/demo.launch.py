#!/usr/bin/env python3
"""
YAM robot MoveIt demo with hardware control and visualization.

use_moveit_servo:=false:
  /yam_follower/cartesian_pose_cmd
    -> tcp_pose_controller
    -> /yam_follower/cartesian_twist_cmd
    -> twist_to_joint_jog
    -> /yam_follower/raw_joint_velocity_cmd

use_moveit_servo:=true:
  use MoveIt Servo instead of the custom Cartesian stack.
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
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
            "can_channel",
            default_value="can0",
            description="CAN channel for motor communication",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Launch RViz for visualization",
        ),
        DeclareLaunchArgument(
            "control_frequency",
            default_value="100.0",
            description="YAM hardware-interface command loop frequency",
        ),
        DeclareLaunchArgument(
            "use_moveit_servo",
            default_value="false",
            description=(
                "true = MoveIt Servo; "
                "false = custom TCP pose controller + DLS controller"
            ),
        ),
        DeclareLaunchArgument(
            "trajectory_servo_lockout",
            default_value="0.35",
            description=(
                "Seconds to keep Cartesian Servo blocked after a MoveIt "
                "trajectory completes"
            ),
        ),
    ]

    gripper_type_follower = LaunchConfiguration("gripper_type_follower")
    can_channel = LaunchConfiguration("can_channel")
    use_rviz = LaunchConfiguration("use_rviz")
    control_frequency = LaunchConfiguration("control_frequency")
    use_moveit_servo = LaunchConfiguration("use_moveit_servo")
    trajectory_servo_lockout = LaunchConfiguration(
        "trajectory_servo_lockout"
    )

    yam_follower_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("i2rt_yam_driver"),
                "launch",
                "yam_hardware.launch.py",
            ])
        ]),
        launch_arguments={
            "can_channel": can_channel,
            "gripper_type": gripper_type_follower,
            "robot_name": "yam_follower",
            "control_frequency": control_frequency,
            "trajectory_servo_lockout": trajectory_servo_lockout,
        }.items(),
    )

    urdf_file = PathJoinSubstitution([
        FindPackageShare("i2rt_description"),
        "urdf",
        ["yam_", gripper_type_follower, ".urdf.xacro"],
    ])

    robot_description = Command(["xacro ", urdf_file])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            # Dynamic TF otherwise defaults to about 20 Hz even though the
            # hardware publishes joint feedback much faster.
            "publish_frequency": 100.0,
        }],
    )

    moveit_config = MoveItConfigsBuilder(
        "output",
        package_name="move_it_yam",
    ).to_moveit_configs()

    moveit_params = moveit_config.to_dict()
    moveit_params.setdefault("ompl", {})
    moveit_params["ompl"]["resample_dt"] = 0.01
    moveit_params.setdefault("chomp", {})
    moveit_params["chomp"]["resample_dt"] = 0.01

    servo_config_path = os.path.join(
        get_package_share_directory("move_it_yam"),
        "config",
        "servo.yaml",
    )

    with open(servo_config_path, "r", encoding="utf-8") as servo_config_file:
        servo_config = yaml.safe_load(servo_config_file)

    servo_config.update({
        "move_group_name": "yam_arm",
        "planning_frame": "base",
        "ee_frame_name": "grasp_point",
        "robot_link_command_frame": "base",
        "command_out_topic": "/yam_follower/servo_joint_command",
    })

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            {"moveit_servo": servo_config},
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(use_moveit_servo),
    )

    pose_tracker = Node(
        package="move_it_yam",
        executable="pose_to_twist",
        name="pose_tracker",
        output="screen",
        parameters=[{
            "target_topic": "/yam_follower/target_pose",
            "command_topic": "/servo_node/delta_twist_cmds",
            "servo_start_service": "/servo_node/start_servo",
            "planning_frame": "base",
            "ee_frame": "grasp_point",
        }],
        condition=IfCondition(use_moveit_servo),
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params],
    )

    rviz_cfg = os.path.join(
        get_package_share_directory("move_it_yam"),
        "config",
        "moveit.rviz",
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

    tcp_pose_controller_node = Node(
        package="yam_cartesian_control",
        executable="tcp_pose_controller",
        name="tcp_pose_controller",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {
                "planning_group": "yam_arm",
                "base_frame": "base",
                "ee_link": "grasp_point",
                "position_kp": 4.0,
                "orientation_kp":3.0,
                "control_rate": 100.0,
                "target_timeout": 0.25,
            },
        ],
        condition=UnlessCondition(use_moveit_servo),
    )

    cartesian_control_node = Node(
        package="yam_cartesian_control",
        executable="twist_to_joint_jog",
        name="twist_to_joint_jog",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {
                "planning_group": "yam_arm",
                "base_frame": "base",
                "ee_link": "grasp_point",
                "damping": 0.01,
                "max_linear_velocity": 0.50,
                "max_angular_velocity": 2.0,
                "max_linear_acceleration": 3.5,
                "max_angular_acceleration": 7.5,
                "max_joint_velocity": 3.0,
                "joint_limit_slow_margin": 0.10,
                "joint_limit_stop_margin": 0.05,
            },
        ],
        condition=UnlessCondition(use_moveit_servo),
    )

    return LaunchDescription(
        declared_arguments
        + [
            yam_follower_launch,
            robot_state_publisher,
            run_move_group_node,
            servo_node,
            pose_tracker,
            rviz_node,
            tcp_pose_controller_node,
            cartesian_control_node,
        ]
    )
