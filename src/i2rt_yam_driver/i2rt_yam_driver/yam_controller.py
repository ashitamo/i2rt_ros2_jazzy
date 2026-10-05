#!/usr/bin/env python3
"""
Simple position controller for YAM arm

Provides simple interfaces for commanding the robot:
- Command individual joint positions
- Command Cartesian poses (with IK)
- Command gripper
"""

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from i2rt_msgs.msg import GripperCommand


class YAMController(Node):
    """Simple controller node for commanding YAM robot"""

    def __init__(self):
        super().__init__('yam_controller')

        self.declare_parameter('robot_name', 'yam')
        self.robot_name = self.get_parameter('robot_name').value

        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointTrajectory,
            f'/{self.robot_name}/joint_command',
            10
        )

        self.gripper_cmd_pub = self.create_publisher(
            GripperCommand,
            f'/{self.robot_name}/gripper_command',
            10
        )

        # Subscriber for current state
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            f'/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info('YAM controller ready')

    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg

    def command_joints(self, positions, duration=1.0):
        """
        Command joint positions

        Args:
            positions: List or array of joint positions (6 or 7 elements)
            duration: Time to reach target (seconds)
        """
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [f'joint{i+1}' for i in range(len(positions))]

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start.sec = int(duration)
        point.time_from_start.nanosec = int((duration % 1.0) * 1e9)

        msg.points.append(point)

        self.joint_cmd_pub.publish(msg)
        self.get_logger().info(f'Commanded joints: {positions}')

    def command_gripper(self, position, force_limit=10.0):
        """
        Command gripper position

        Args:
            position: Gripper position 0.0 (open) to 1.0 (closed)
            force_limit: Maximum force in Newtons
        """
        msg = GripperCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = GripperCommand.POSITION_CONTROL
        msg.position = float(np.clip(position, 0.0, 1.0))
        msg.max_force = float(force_limit)
        msg.duration = 1.0

        self.gripper_cmd_pub.publish(msg)
        self.get_logger().info(f'Commanded gripper: {position}')

    def get_current_position(self):
        """Get current joint positions"""
        if self.current_joint_state is None:
            return None
        return np.array(self.current_joint_state.position)


def main(args=None):
    rclpy.init(args=args)

    controller = YAMController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()