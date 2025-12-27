#!/usr/bin/env python3
"""
Leader-Follower Teleoperation Node

Synchronizes follower arm to leader arm motion.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class LeaderFollowerNode(Node):
    def __init__(self):
        super().__init__('leader_follower_teleop')

        # Subscribe to leader joint states
        self.leader_sub = self.create_subscription(
            JointState, '/yam_leader/joint_states',
            self.leader_callback, 10)

        # Publish to follower commands
        self.follower_pub = self.create_publisher(
            JointTrajectory, '/yam_follower/joint_command', 10)

        self.get_logger().info('Leader-follower teleoperation active')

    def leader_callback(self, msg):
        # Mirror leader position to follower
        cmd = JointTrajectory()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.joint_names = msg.name

        point = JointTrajectoryPoint()
        point.positions = list(msg.position)
        cmd.points.append(point)

        self.follower_pub.publish(cmd)


def main():
    rclpy.init()
    node = LeaderFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
