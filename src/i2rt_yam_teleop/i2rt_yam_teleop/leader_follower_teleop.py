#!/usr/bin/env python3
"""ROS 2 equivalent of minimum_gello.py's leader mode."""

from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from i2rt_msgs.msg import GripperCommand, TeachingHandleState

class TeleopState(Enum):
    STARTUP = "startup"
    IDLE = "idle"
    ALIGNING = "aligning"
    SYNCHRONIZED = "synchronized"

class LeaderFollowerTeleop(Node):
    """Button-controlled, bilateral leader-follower joint teleoperation."""

    ARM_DOF = 6

    # Edit this array to set the shared automatic startup pose. Values are in
    # radians and ordered as joint1, joint2, ..., joint6. During startup, the
    # follower moves toward this target and the leader follows its measurements.
    START_JOINT_POSITIONS = np.array(
        [-0.039, 0.57, 0.99, -1.25, 0.0337, 0.0509], dtype=float
    )

    # Startup motion settings. The leader needs a nonzero gain while it moves;
    # it becomes compliant again after both arms reach their startup poses.
    STARTUP_MOVE_DURATION = 5.0
    STARTUP_COMMAND_PRELOAD_DURATION = 0.1
    STARTUP_POSITION_TOLERANCE = 0.348
    STARTUP_LEADER_KP_SCALE = 0.05

    def __init__(self):
        super().__init__("leader_follower_teleop")
        self.declare_parameter("leader_name", "yam_leader")
        self.declare_parameter("follower_name", "yam_follower")
        self.declare_parameter("bilateral_kp", 0.0)
        self.declare_parameter("slow_sync_duration", 3.0)
        self.declare_parameter("command_frequency", 100.0)
        self.declare_parameter("state_timeout", 0.25)
        self.declare_parameter("follower_gripper_joint_scale", -0.0475)

        self.leader_name = str(self.get_parameter("leader_name").value)
        self.follower_name = str(self.get_parameter("follower_name").value)
        self.bilateral_kp = float(self.get_parameter("bilateral_kp").value)
        self.slow_sync_duration = float(self.get_parameter("slow_sync_duration").value)
        self.command_frequency = float(self.get_parameter("command_frequency").value)
        self.state_timeout = float(self.get_parameter("state_timeout").value)
        self.gripper_joint_scale = float(
            self.get_parameter("follower_gripper_joint_scale").value
        )

        if self.bilateral_kp < 0.0:
            raise ValueError("bilateral_kp must be non-negative")
        if self.slow_sync_duration <= 0.0:
            raise ValueError("slow_sync_duration must be positive")
        if self.command_frequency <= 0.0:
            raise ValueError("command_frequency must be positive")
        if self.state_timeout <= 0.0:
            raise ValueError("state_timeout must be positive")
        if self.START_JOINT_POSITIONS.shape != (self.ARM_DOF,):
            raise ValueError("START_JOINT_POSITIONS must contain six values")
        if not np.all(np.isfinite(self.START_JOINT_POSITIONS)):
            raise ValueError("START_JOINT_POSITIONS must be finite")
        if self.STARTUP_MOVE_DURATION <= 0.0:
            raise ValueError("STARTUP_MOVE_DURATION must be positive")
        if self.STARTUP_COMMAND_PRELOAD_DURATION < 0.0:
            raise ValueError("STARTUP_COMMAND_PRELOAD_DURATION cannot be negative")
        if self.STARTUP_POSITION_TOLERANCE <= 0.0:
            raise ValueError("STARTUP_POSITION_TOLERANCE must be positive")
        if self.STARTUP_LEADER_KP_SCALE <= 0.0:
            raise ValueError("STARTUP_LEADER_KP_SCALE must be positive")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        sync_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.leader_position = None
        self.follower_position = None
        self.follower_gripper = 1.0
        self.leader_trigger = 1.0
        self.leader_state_time_ns = None
        self.follower_state_time_ns = None
        self.state = TeleopState.STARTUP
        self.startup_prepare_time_ns = None
        self.startup_motion_start_time_ns = None
        self.startup_leader_initial = None
        self.startup_follower_initial = None
        self.alignment_start_time_ns = None
        self.alignment_start_arm = None
        self.alignment_target_arm = None
        self.alignment_start_gripper = 1.0
        self.alignment_target_gripper = 1.0

        self.create_subscription(
            JointState, f"/{self.leader_name}/joint_states",
            self._leader_state_callback, qos,
        )
        self.create_subscription(
            JointState, f"/{self.follower_name}/joint_states",
            self._follower_state_callback, qos,
        )
        self.create_subscription(
            TeachingHandleState, f"/{self.leader_name}/teaching_handle_state",
            self._handle_callback, qos,
        )
        self.follower_arm_pub = self.create_publisher(
            JointTrajectory, f"/{self.follower_name}/joint_command", qos,
        )
        self.follower_gripper_pub = self.create_publisher(
            GripperCommand, f"/{self.follower_name}/gripper_command", qos,
        )
        self.leader_arm_pub = self.create_publisher(
            JointTrajectory, f"/{self.leader_name}/joint_command", qos,
        )
        self.leader_gain_pub = self.create_publisher(
            Float64, f"/{self.leader_name}/pd_gain_scale", qos,
        )
        self.sync_state_pub = self.create_publisher(
            Bool, "/leader_follower/synchronized", sync_qos,
        )
        self.create_timer(1.0 / self.command_frequency, self._control_tick)
        self._publish_gain_scale(0.0)
        self._publish_synchronization_state(False)
        self.get_logger().info(
            "Waiting for fresh leader and follower states before startup motion"
        )

    @staticmethod
    def _extract_arm_positions(msg: JointState):
        by_name = dict(zip(msg.name, msg.position))
        names = [f"joint{i + 1}" for i in range(6)]
        if all(name in by_name for name in names):
            return np.asarray([by_name[name] for name in names], dtype=float)
        if len(msg.position) >= 6:
            return np.asarray(msg.position[:6], dtype=float)
        return None

    def _leader_state_callback(self, msg: JointState) -> None:
        self.get_logger().debug(msg)
        positions = self._extract_arm_positions(msg)
        if positions is None:
            self.get_logger().warn("Leader JointState has fewer than six joints")
            return
        self.leader_position = positions
        self.leader_state_time_ns = self.get_clock().now().nanoseconds
        

    def _follower_state_callback(self, msg: JointState) -> None:
        positions = self._extract_arm_positions(msg)
        if positions is None:
            self.get_logger().warn("Follower JointState has fewer than six joints")
            return
        self.follower_position = positions
        self.follower_state_time_ns = self.get_clock().now().nanoseconds
        if len(msg.position) > 6 and abs(self.gripper_joint_scale) > 1e-9:
            self.follower_gripper = float(
                np.clip(msg.position[6] / self.gripper_joint_scale, 0.0, 1.0)
            )

    def _handle_callback(self, msg: TeachingHandleState) -> None:
        self.leader_trigger = float(np.clip(msg.trigger_position, 0.0, 1.0))
        if not msg.button1_rising_edge:
            return
        if self.state == TeleopState.STARTUP:
            self.get_logger().warn("Ignoring sync button during startup motion")
        elif self.state == TeleopState.IDLE:
            self._start_alignment()
        else:
            self._disable_synchronization("button pressed")

    def _states_available(self) -> bool:
        return self.leader_position is not None and self.follower_position is not None

    def _states_fresh(self, now_ns: int) -> bool:
        if self.leader_state_time_ns is None or self.follower_state_time_ns is None:
            return False
        timeout_ns = int(self.state_timeout * 1e9)
        return (
            now_ns - self.leader_state_time_ns <= timeout_ns
            and now_ns - self.follower_state_time_ns <= timeout_ns
        )

    def _start_alignment(self) -> None:
        if not self._states_available():
            self.get_logger().warn(
                "Cannot synchronize until both robot states have been received"
            )
            return
        now_ns = self.get_clock().now().nanoseconds
        if not self._states_fresh(now_ns):
            self.get_logger().warn("Cannot synchronize with stale robot state")
            return

        self.alignment_start_time_ns = now_ns
        self.alignment_start_arm = self.follower_position.copy()
        self.alignment_target_arm = self.leader_position.copy()
        self.alignment_start_gripper = self.follower_gripper
        self.alignment_target_gripper = self.leader_trigger
        self._publish_gain_scale(self.bilateral_kp)
        self._publish_arm_command(self.leader_arm_pub, self.leader_position)
        self.state = TeleopState.ALIGNING
        self.get_logger().info(
            f"Aligning follower over {self.slow_sync_duration:.2f} seconds"
        )

    def _disable_synchronization(self, reason: str) -> None:
        self._publish_gain_scale(0.0)
        if self.leader_position is not None:
            self._publish_arm_command(self.leader_arm_pub, self.leader_position)
        if self.follower_position is not None:
            self._publish_arm_command(self.follower_arm_pub, self.follower_position)
        self.state = TeleopState.STARTUP
        self.startup_prepare_time_ns = None
        self.startup_motion_start_time_ns = None
        self.startup_leader_initial = None
        self.startup_follower_initial = None
        self.alignment_start_time_ns = None
        self._publish_synchronization_state(False)
        self.get_logger().info(
            f"Teleoperation disabled: {reason}; returning both arms to "
            "their startup poses"
        )

    def _control_tick(self) -> None:
        if self.state == TeleopState.STARTUP:
            self._startup_tick()
            return
        if self.state == TeleopState.IDLE:
            return
        now_ns = self.get_clock().now().nanoseconds
        if not self._states_fresh(now_ns):
            self._disable_synchronization("robot state timeout")
            return

        if self.state == TeleopState.ALIGNING:
            elapsed = (now_ns - self.alignment_start_time_ns) * 1e-9
            alpha = float(np.clip(elapsed / self.slow_sync_duration, 0.0, 1.0))
            arm_target = (
                (1.0 - alpha) * self.alignment_start_arm
                + alpha * self.alignment_target_arm
            )
            gripper_target = (
                (1.0 - alpha) * self.alignment_start_gripper
                + alpha * self.alignment_target_gripper
            )
            self._publish_arm_command(self.follower_arm_pub, arm_target)
            self._publish_gripper_command(gripper_target)
            if alpha >= 1.0:
                self.state = TeleopState.SYNCHRONIZED
                self._publish_synchronization_state(True)
                self.get_logger().info("Leader and follower synchronized")
            return

        self._publish_arm_command(self.follower_arm_pub, self.leader_position)
        self._publish_gripper_command(self.leader_trigger)
        self._publish_arm_command(self.leader_arm_pub, self.follower_position)

    def _startup_tick(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if not self._states_available() or not self._states_fresh(now_ns):
            if self.startup_prepare_time_ns is not None:
                self._publish_gain_scale(0.0)
                self.startup_prepare_time_ns = None
                self.startup_motion_start_time_ns = None
                self.get_logger().warn(
                    "Startup motion paused because robot state is unavailable or stale"
                )
            return

        if self.startup_prepare_time_ns is None:
            self.startup_leader_initial = self.leader_position.copy()
            self.startup_follower_initial = self.follower_position.copy()
            self.startup_prepare_time_ns = now_ns
            self._publish_arm_command(
                self.leader_arm_pub, self.startup_leader_initial
            )
            self._publish_arm_command(
                self.follower_arm_pub, self.startup_follower_initial
            )
            self.get_logger().info(
                f"Preparing automatic {self.STARTUP_MOVE_DURATION:.2f}-second "
                "startup motion"
            )
            return

        if self.startup_motion_start_time_ns is None:
            self._publish_arm_command(
                self.leader_arm_pub, self.startup_leader_initial
            )
            self._publish_arm_command(
                self.follower_arm_pub, self.startup_follower_initial
            )
            preload_elapsed = (now_ns - self.startup_prepare_time_ns) * 1e-9
            if preload_elapsed >= self.STARTUP_COMMAND_PRELOAD_DURATION:
                self._publish_gain_scale(self.STARTUP_LEADER_KP_SCALE)
                self.startup_motion_start_time_ns = now_ns
                self.get_logger().info("Moving leader and follower to startup poses")
            return

        elapsed = (now_ns - self.startup_motion_start_time_ns) * 1e-9
        alpha = float(np.clip(elapsed / self.STARTUP_MOVE_DURATION, 0.0, 1.0))
        # Smoothstep gives zero target velocity at the beginning and end.
        blend = alpha * alpha * (3.0 - 2.0 * alpha)
        follower_target = (
            (1.0 - blend) * self.startup_follower_initial
            + blend * self.START_JOINT_POSITIONS
        )
        # Blend into following the follower's measured motion. This avoids a
        # sudden leader target jump when the two arms begin at different poses.
        leader_target = (
            (1.0 - blend) * self.startup_leader_initial
            + blend * self.follower_position
        )
        self._publish_arm_command(self.follower_arm_pub, follower_target)
        self._publish_arm_command(self.leader_arm_pub, leader_target)

        if alpha < 1.0:
            return

        leader_error = float(
            np.max(np.abs(self.leader_position - self.START_JOINT_POSITIONS))
        )
        follower_error = float(
            np.max(np.abs(self.follower_position - self.START_JOINT_POSITIONS))
        )
        if max(leader_error, follower_error) > self.STARTUP_POSITION_TOLERANCE:
            self.get_logger().info(
                "Waiting for startup poses: "
                f"leader error={leader_error:.3f} rad, "
                f"follower error={follower_error:.3f} rad",
                throttle_duration_sec=1.0,
            )
            return

        self._publish_gain_scale(0.0)
        self.state = TeleopState.IDLE
        self.get_logger().info(
            "Startup poses reached; teleop ready. "
            "Press teaching-handle button 1 to synchronize"
        )

    def _publish_arm_command(self, publisher, positions: np.ndarray) -> None:
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.joint_names = [f"joint{i + 1}" for i in range(self.ARM_DOF)]
        point = JointTrajectoryPoint()
        point.positions = np.asarray(positions[: self.ARM_DOF], dtype=float).tolist()
        msg.points.append(point)
        publisher.publish(msg)

    def _publish_gripper_command(self, position: float) -> None:
        msg = GripperCommand()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mode = GripperCommand.POSITION_CONTROL
        msg.position = float(np.clip(position, 0.0, 1.0))
        msg.max_force = 50.0
        msg.duration = 0.0
        self.follower_gripper_pub.publish(msg)

    def _publish_gain_scale(self, scale: float) -> None:
        msg = Float64()
        msg.data = float(scale)
        self.leader_gain_pub.publish(msg)

    def _publish_synchronization_state(self, synchronized: bool) -> None:
        msg = Bool()
        msg.data = synchronized
        self.sync_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
