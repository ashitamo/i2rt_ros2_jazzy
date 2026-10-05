#!/usr/bin/env python3
"""Plan or execute a collision-aware MoveIt joint-space goal."""

import argparse
import sys

import rclpy
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from rclpy.action import ActionClient
from rclpy.node import Node


class JointPoseMover(Node):
    def __init__(self, args):
        super().__init__("move_to_joint_pose")
        self.args = args
        self.client = ActionClient(self, MoveGroup, args.action)

    def run(self):
        if not self.client.wait_for_server(timeout_sec=self.args.server_timeout):
            self.get_logger().error(
                f"MoveGroup action server {self.args.action} is unavailable"
            )
            return 2

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.args.group
        request.pipeline_id = self.args.pipeline
        request.num_planning_attempts = self.args.planning_attempts
        request.allowed_planning_time = self.args.planning_time
        request.max_velocity_scaling_factor = self.args.velocity_scale
        request.max_acceleration_scaling_factor = self.args.acceleration_scale
        request.start_state.is_diff = True

        constraints = Constraints()
        constraints.name = "joint_pose_goal"
        for index, position in enumerate(self.args.positions, start=1):
            constraint = JointConstraint()
            constraint.joint_name = f"joint{index}"
            constraint.position = position
            constraint.tolerance_above = self.args.tolerance
            constraint.tolerance_below = self.args.tolerance
            constraint.weight = 1.0
            constraints.joint_constraints.append(constraint)
        request.goal_constraints.append(constraints)

        goal.planning_options.plan_only = not self.args.execute
        goal.planning_options.look_around = False
        goal.planning_options.replan = False

        operation = "PLAN+EXECUTE" if self.args.execute else "PLAN ONLY"
        self.get_logger().warning(
            f"{operation}: group={self.args.group}, positions={self.args.positions}, "
            f"velocity/acceleration scale={self.args.velocity_scale:.3f}/"
            f"{self.args.acceleration_scale:.3f}"
        )
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("MoveGroup rejected the goal")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped_result = result_future.result()
        if wrapped_result is None:
            self.get_logger().error("MoveGroup returned no result")
            return 1
        result = wrapped_result.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f"MoveGroup failed with error code {result.error_code.val}"
            )
            return 1

        if self.args.execute:
            self.get_logger().info("Planning and execution completed successfully")
        else:
            point_count = len(result.planned_trajectory.joint_trajectory.points)
            self.get_logger().info(
                f"Planning succeeded ({point_count} trajectory points); "
                "the arm was not commanded"
            )
        return 0


def scale(value):
    parsed = float(value)
    if not 0.0 < parsed <= 1.0:
        raise argparse.ArgumentTypeError("must be in the range (0, 1]")
    return parsed


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description="Plan or execute a six-joint MoveIt goal"
    )
    parser.add_argument(
        "--positions", type=float, nargs=6, required=True,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
    )
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--group", default="yam_arm")
    parser.add_argument("--action", default="/move_action")
    parser.add_argument("--pipeline", default="ompl")
    parser.add_argument("--velocity-scale", type=scale, default=0.03)
    parser.add_argument("--acceleration-scale", type=scale, default=0.03)
    parser.add_argument("--tolerance", type=float, default=0.005)
    parser.add_argument("--planning-attempts", type=int, default=10)
    parser.add_argument("--planning-time", type=float, default=10.0)
    parser.add_argument("--server-timeout", type=float, default=5.0)
    args = parser.parse_args(argv)
    if args.tolerance <= 0.0:
        parser.error("--tolerance must be positive")
    if args.planning_attempts < 1:
        parser.error("--planning-attempts must be at least 1")
    if args.planning_time <= 0.0 or args.server_timeout <= 0.0:
        parser.error("timeouts must be positive")
    return args


def main(argv=None):
    args = parse_arguments(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = JointPoseMover(args)
    try:
        return node.run()
    except KeyboardInterrupt:
        node.get_logger().warning("Interrupted")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
