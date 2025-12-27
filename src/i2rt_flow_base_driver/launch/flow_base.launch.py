#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('i2rt_flow_base_driver'),
        'config',
        'flow_base_params.yaml'
    ])

    flow_base_node = Node(
        package='i2rt_flow_base_driver',
        executable='flow_base_node',
        name='flow_base_node',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )

    return LaunchDescription([flow_base_node])
