#!/usr/bin/env python3
"""Replay a timestamped end-effector CSV as PoseStamped targets."""

import argparse
import bisect
import csv
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from tf2_ros import Buffer, TransformException, TransformListener


REQUIRED_COLUMNS = (
    "timestamp_sec",
    "x_m",
    "y_m",
    "z_m",
    "qx",
    "qy",
    "qz",
    "qw",
)

SERVO_STATUS_NAMES = {
    -1: "invalid",
    0: "no warning",
    1: "approaching singularity; decelerating",
    2: "too close to singularity; halted",
    3: "approaching collision; decelerating",
    4: "collision detected; halted",
    5: "close to joint bound; halted",
    6: "leaving singularity; decelerating",
}
SERVO_HALT_STATUSES = {-1, 2, 4, 5}


def vector_add(lhs, rhs):
    return tuple(a + b for a, b in zip(lhs, rhs))


def vector_subtract(lhs, rhs):
    return tuple(a - b for a, b in zip(lhs, rhs))


def vector_scale(vector, scale):
    return tuple(value * scale for value in vector)


def quaternion_normalize(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in quaternion)


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return -x, -y, -z, w


def quaternion_multiply(lhs, rhs):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_from_rpy(roll, pitch, yaw):
    """Return an xyzw quaternion for fixed-axis roll, pitch, yaw angles."""
    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return quaternion_normalize((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def rotate_vector(quaternion, vector):
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, (*vector, 0.0)),
        quaternion_conjugate(quaternion),
    )
    return rotated[:3]


def quaternion_slerp(lhs, rhs, fraction):
    lhs = quaternion_normalize(lhs)
    rhs = quaternion_normalize(rhs)
    dot = sum(a * b for a, b in zip(lhs, rhs))
    if dot < 0.0:
        rhs = tuple(-value for value in rhs)
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return quaternion_normalize(
            tuple(a + fraction * (b - a) for a, b in zip(lhs, rhs))
        )
    angle = math.acos(dot)
    sin_angle = math.sin(angle)
    lhs_scale = math.sin((1.0 - fraction) * angle) / sin_angle
    rhs_scale = math.sin(fraction * angle) / sin_angle
    return tuple(
        lhs_scale * a + rhs_scale * b for a, b in zip(lhs, rhs)
    )


def quaternion_angle(lhs, rhs):
    dot = abs(
        sum(
            a * b
            for a, b in zip(
                quaternion_normalize(lhs), quaternion_normalize(rhs)
            )
        )
    )
    return 2.0 * math.acos(min(1.0, max(-1.0, dot)))


def load_trajectory(csv_path):
    samples = []
    with open(csv_path, newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        missing = [name for name in REQUIRED_COLUMNS if name not in reader.fieldnames]
        if missing:
            raise ValueError(f"missing CSV columns: {', '.join(missing)}")
        for line_number, row in enumerate(reader, start=2):
            try:
                values = [float(row[name]) for name in REQUIRED_COLUMNS]
            except (TypeError, ValueError) as error:
                raise ValueError(f"invalid number on CSV line {line_number}") from error
            if not all(math.isfinite(value) for value in values):
                raise ValueError(f"non-finite value on CSV line {line_number}")
            stamp = values[0]
            position = tuple(values[1:4])
            orientation = quaternion_normalize(tuple(values[4:8]))
            if samples and stamp <= samples[-1][0]:
                raise ValueError(
                    f"timestamps must increase strictly (CSV line {line_number})"
                )
            samples.append((stamp, position, orientation))
    if len(samples) < 2:
        raise ValueError("CSV must contain at least two trajectory samples")
    start_stamp = samples[0][0]
    return [
        (stamp - start_stamp, position, orientation)
        for stamp, position, orientation in samples
    ]


def trajectory_statistics(samples):
    first_position = samples[0][1]
    first_orientation = samples[0][2]
    max_offset = max(
        math.dist(first_position, sample[1]) for sample in samples
    )
    max_rotation = max(
        quaternion_angle(first_orientation, sample[2]) for sample in samples
    )
    max_speed = 0.0
    for previous, current in zip(samples, samples[1:]):
        dt = current[0] - previous[0]
        max_speed = max(
            max_speed, math.dist(previous[1], current[1]) / dt
        )
    return max_offset, max_rotation, max_speed


def interpolate_sample(samples, sample_times, trajectory_time):
    if trajectory_time <= 0.0:
        return samples[0][1], samples[0][2]
    if trajectory_time >= sample_times[-1]:
        return samples[-1][1], samples[-1][2]
    upper = bisect.bisect_right(sample_times, trajectory_time)
    lower = upper - 1
    first = samples[lower]
    second = samples[upper]
    fraction = (trajectory_time - first[0]) / (second[0] - first[0])
    position = tuple(
        a + fraction * (b - a) for a, b in zip(first[1], second[1])
    )
    orientation = quaternion_slerp(first[2], second[2], fraction)
    return position, orientation


class CsvPosePlayer(Node):
    def __init__(self, args, samples):
        super().__init__("csv_pose_player")
        self.args = args
        self.samples = samples
        self.sample_times = [sample[0] for sample in samples]
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(
            PoseStamped, args.target_topic, qos
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.servo_status = None
        self.last_reported_servo_status = None
        self.status_subscriber = self.create_subscription(
            Int8, args.status_topic, self.status_callback, qos
        )
        self.joint_state = None
        self.joint_state_subscriber = self.create_subscription(
            JointState, args.joint_state_topic, self.joint_state_callback, 10
        )
        self.ik_client = self.create_client(GetPositionIK, args.ik_service)

    def status_callback(self, message):
        self.servo_status = message.data
        if message.data == self.last_reported_servo_status:
            return
        self.last_reported_servo_status = message.data
        status_name = SERVO_STATUS_NAMES.get(
            message.data, f"unknown status {message.data}"
        )
        if message.data in SERVO_HALT_STATUSES:
            self.get_logger().error(
                f"Servo status {message.data}: {status_name}"
            )
        elif message.data != 0:
            self.get_logger().warning(
                f"Servo status {message.data}: {status_name}; playback continues"
            )
        else:
            self.get_logger().info("Servo status returned to 0: no warning")

    def joint_state_callback(self, message):
        self.joint_state = message

    def wait_for_joint_state(self):
        deadline = time.monotonic() + self.args.tf_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.joint_state is not None:
                return self.joint_state
        raise RuntimeError(
            f"no joint state on {self.args.joint_state_topic} within "
            f"{self.args.tf_timeout:.1f} s"
        )

    def wait_for_current_pose(self):
        deadline = time.monotonic() + self.args.tf_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.args.frame, self.args.ee_frame, Time()
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                return (
                    (translation.x, translation.y, translation.z),
                    quaternion_normalize(
                        (rotation.x, rotation.y, rotation.z, rotation.w)
                    ),
                )
            except TransformException:
                continue
        raise RuntimeError(
            f"no transform {self.args.frame} -> {self.args.ee_frame} "
            f"within {self.args.tf_timeout:.1f} s"
        )

    def output_pose(self, source_position, source_orientation, start_pose):
        if self.args.mode == "absolute":
            return source_position, source_orientation

        source_start_position = self.samples[0][1]
        source_start_orientation = self.samples[0][2]
        current_position, current_orientation = start_pose
        source_delta_rotation = quaternion_multiply(
            quaternion_conjugate(source_start_orientation),
            source_orientation,
        )
        scaled_delta_rotation = quaternion_slerp(
            (0.0, 0.0, 0.0, 1.0),
            source_delta_rotation,
            self.args.orientation_scale,
        )
        output_orientation = quaternion_normalize(
            quaternion_multiply(current_orientation, scaled_delta_rotation)
        )
        source_delta_position = vector_scale(
            vector_subtract(source_position, source_start_position),
            self.args.position_scale,
        )
        if self.args.mode == "relative_se3":
            alignment_rotation = quaternion_multiply(
                current_orientation,
                quaternion_conjugate(source_start_orientation),
            )
            source_delta_position = rotate_vector(
                alignment_rotation, source_delta_position
            )
        output_position = vector_add(current_position, source_delta_position)
        return output_position, output_orientation

    def publish_pose(self, position, orientation):
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.args.frame
        message.pose.position.x, message.pose.position.y, message.pose.position.z = (
            position
        )
        (
            message.pose.orientation.x,
            message.pose.orientation.y,
            message.pose.orientation.z,
            message.pose.orientation.w,
        ) = orientation
        self.publisher.publish(message)

    def make_ik_request(self, position, orientation, seed_state):
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = self.args.ik_group
        ik_request.ik_link_name = self.args.ee_frame
        ik_request.avoid_collisions = True
        ik_request.robot_state.joint_state = seed_state
        ik_request.pose_stamped.header.frame_id = self.args.frame
        ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        ik_request.pose_stamped.pose.position.x = position[0]
        ik_request.pose_stamped.pose.position.y = position[1]
        ik_request.pose_stamped.pose.position.z = position[2]
        ik_request.pose_stamped.pose.orientation.x = orientation[0]
        ik_request.pose_stamped.pose.orientation.y = orientation[1]
        ik_request.pose_stamped.pose.orientation.z = orientation[2]
        ik_request.pose_stamped.pose.orientation.w = orientation[3]
        timeout_seconds = int(self.args.ik_timeout)
        ik_request.timeout.sec = timeout_seconds
        ik_request.timeout.nanosec = int(
            (self.args.ik_timeout - timeout_seconds) * 1e9
        )
        return request

    def check_ik(self):
        current_pose = self.wait_for_current_pose()
        anchor_rpy = tuple(
            math.radians(value) for value in self.args.anchor_rpy_offset_deg
        )
        anchor_rotation = quaternion_from_rpy(*anchor_rpy)
        start_pose = (
            vector_add(current_pose[0], tuple(self.args.anchor_offset)),
            quaternion_normalize(
                quaternion_multiply(anchor_rotation, current_pose[1])
            ),
        )
        initial_joint_state = self.wait_for_joint_state()
        if not self.ik_client.wait_for_service(timeout_sec=self.args.tf_timeout):
            raise RuntimeError(
                f"IK service {self.args.ik_service} is not available"
            )

        selected_samples = list(enumerate(self.samples))[:: self.args.ik_stride]
        if selected_samples[-1][0] != len(self.samples) - 1:
            selected_samples.append((len(self.samples) - 1, self.samples[-1]))
        seed_state = JointState()
        seed_state.header = initial_joint_state.header
        seed_state.name = list(initial_joint_state.name)
        seed_state.position = list(initial_joint_state.position)
        seed_state.velocity = []
        seed_state.effort = []
        arm_joint_names = [f"joint{index}" for index in range(1, 7)]
        previous_arm_positions = None
        report_rows = []

        self.get_logger().info(
            f"Checking {len(selected_samples)} poses through "
            f"{self.args.ik_service}; no motion commands will be published"
        )
        self.get_logger().info(
            f"Current anchor position: {current_pose[0]}; virtual IK anchor "
            f"position: {start_pose[0]}; orientation: {start_pose[1]}"
        )
        for checked_number, (local_index, sample) in enumerate(selected_samples):
            source_index = local_index + self.args.index_offset
            _, source_position, source_orientation = sample
            target_position, target_orientation = self.output_pose(
                source_position, source_orientation, start_pose
            )
            future = self.ik_client.call_async(
                self.make_ik_request(
                    target_position, target_orientation, seed_state
                )
            )
            rclpy.spin_until_future_complete(
                self,
                future,
                timeout_sec=max(1.0, self.args.ik_timeout + 0.5),
            )
            response = future.result() if future.done() else None
            error_code = (
                response.error_code.val
                if response is not None
                else MoveItErrorCodes.TIMED_OUT
            )
            success = (
                response is not None
                and error_code == MoveItErrorCodes.SUCCESS
            )
            joint_positions = {}
            max_joint_step = math.nan
            if success:
                solution = response.solution.joint_state
                joint_positions = dict(zip(solution.name, solution.position))
                if not all(name in joint_positions for name in arm_joint_names):
                    success = False
                    error_code = MoveItErrorCodes.INVALID_ROBOT_STATE
                else:
                    arm_positions = [
                        joint_positions[name] for name in arm_joint_names
                    ]
                    if previous_arm_positions is not None:
                        max_joint_step = max(
                            abs(current - previous)
                            for current, previous in zip(
                                arm_positions, previous_arm_positions
                            )
                        )
                        if max_joint_step > self.args.max_joint_step:
                            success = False
                            error_code = MoveItErrorCodes.INVALID_MOTION_PLAN
                    if success:
                        previous_arm_positions = arm_positions
                        seed_state = solution
            if not success:
                previous_arm_positions = None
                seed_state = JointState()
                seed_state.header = initial_joint_state.header
                seed_state.name = list(initial_joint_state.name)
                seed_state.position = list(initial_joint_state.position)

            report_rows.append({
                "index": source_index,
                "time_sec": sample[0],
                "x": target_position[0],
                "y": target_position[1],
                "z": target_position[2],
                "qx": target_orientation[0],
                "qy": target_orientation[1],
                "qz": target_orientation[2],
                "qw": target_orientation[3],
                "success": int(success),
                "error_code": error_code,
                "max_joint_step": max_joint_step,
                **{
                    name: joint_positions.get(name, math.nan)
                    for name in arm_joint_names
                },
            })
            if checked_number % 50 == 0 or checked_number == len(selected_samples) - 1:
                self.get_logger().info(
                    f"IK check {checked_number + 1}/{len(selected_samples)}"
                )

        report_path = os.path.abspath(os.path.expanduser(self.args.ik_report))
        with open(report_path, "w", newline="", encoding="utf-8") as report_file:
            writer = csv.DictWriter(
                report_file, fieldnames=list(report_rows[0].keys())
            )
            writer.writeheader()
            writer.writerows(report_rows)

        longest_start = None
        longest_end = None
        current_start = None
        previous_index = None
        checked_step = self.args.ik_stride
        for row in report_rows:
            if row["success"]:
                if (
                    current_start is None
                    or previous_index is None
                    or row["index"] - previous_index > checked_step
                ):
                    current_start = row["index"]
                if (
                    longest_start is None
                    or row["index"] - current_start
                    > longest_end - longest_start
                ):
                    longest_start = current_start
                    longest_end = row["index"]
                previous_index = row["index"]
            else:
                current_start = None
                previous_index = None

        successful_count = sum(row["success"] for row in report_rows)
        self.get_logger().info(
            f"IK check complete: {successful_count}/{len(report_rows)} valid; "
            f"report={report_path}"
        )
        if longest_start is None:
            self.get_logger().error("No valid IK sample was found")
            return 1
        self.get_logger().info(
            f"Longest checked valid interval: CSV index "
            f"{longest_start}..{longest_end}"
        )
        return 0 if successful_count == len(report_rows) else 1

    def play(self):
        start_pose = self.wait_for_current_pose()
        if self.args.mode == "absolute":
            position_error = math.dist(start_pose[0], self.samples[0][1])
            orientation_error = quaternion_angle(
                start_pose[1], self.samples[0][2]
            )
            if (
                position_error > self.args.max_start_position_error
                or orientation_error > self.args.max_start_orientation_error
            ):
                raise RuntimeError(
                    "absolute trajectory start mismatch: "
                    f"position={position_error:.3f} m, "
                    f"orientation={math.degrees(orientation_error):.1f} deg"
                )

        self.get_logger().info(
            f"Current TCP: position={start_pose[0]}, orientation={start_pose[1]}"
        )
        self.get_logger().warning(
            f"Publishing to {self.args.target_topic} in {self.args.countdown:.1f} s; "
            "Ctrl-C aborts and lets the Servo timeout hold the arm"
        )
        countdown_deadline = time.monotonic() + self.args.countdown
        while rclpy.ok() and time.monotonic() < countdown_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

        start_time = time.monotonic()
        next_publish = start_time
        real_duration = self.sample_times[-1] / self.args.speed_scale
        period = 1.0 / self.args.publish_rate
        last_progress_second = -1
        while rclpy.ok():
            now = time.monotonic()
            elapsed = now - start_time
            if elapsed > real_duration:
                break
            if now < next_publish:
                rclpy.spin_once(
                    self, timeout_sec=min(0.01, next_publish - now)
                )
                continue
            if (
                self.args.abort_on_servo_warning
                and self.servo_status in SERVO_HALT_STATUSES
            ):
                status_name = SERVO_STATUS_NAMES.get(
                    self.servo_status, "unknown status"
                )
                raise RuntimeError(
                    f"Servo halt status {self.servo_status} ({status_name}); "
                    "playback aborted"
                )
            trajectory_time = min(
                elapsed * self.args.speed_scale, self.sample_times[-1]
            )
            source_pose = interpolate_sample(
                self.samples, self.sample_times, trajectory_time
            )
            self.publish_pose(*self.output_pose(*source_pose, start_pose))
            progress_second = int(elapsed)
            if progress_second != last_progress_second:
                self.get_logger().info(
                    f"Playback {elapsed:.1f}/{real_duration:.1f} s "
                    f"({100.0 * elapsed / real_duration:.1f}%)"
                )
                last_progress_second = progress_second
            next_publish += period
            if next_publish < now - period:
                next_publish = now + period

        final_source_pose = self.samples[-1][1], self.samples[-1][2]
        final_pose = self.output_pose(*final_source_pose, start_pose)
        hold_deadline = time.monotonic() + self.args.hold_seconds
        while rclpy.ok() and time.monotonic() < hold_deadline:
            self.publish_pose(*final_pose)
            rclpy.spin_once(self, timeout_sec=period)
        self.get_logger().info(
            "Playback complete; target publication stopped. Return to init_pose "
            "before shutting down the robot launch."
        )


def positive_float(value):
    parsed = float(value)
    if parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be greater than zero")
    return parsed


def scale_float(value):
    parsed = float(value)
    if not 0.0 < parsed <= 1.0:
        raise argparse.ArgumentTypeError("must be in the range (0, 1]")
    return parsed


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description="Replay a timestamped pose CSV through MoveIt Servo"
    )
    parser.add_argument("csv_path", help="CSV file containing timestamp_sec and pose columns")
    parser.add_argument("--execute", action="store_true", help="publish the trajectory; otherwise only validate it")
    parser.add_argument(
        "--check-ik",
        action="store_true",
        help="check retargeted poses with MoveIt IK without publishing motion commands",
    )
    parser.add_argument(
        "--mode",
        choices=("relative", "relative_se3", "absolute"),
        default="relative",
        help=(
            "relative translates base-frame positions to the current TCP; "
            "relative_se3 also rotates position offsets with the start-pose "
            "alignment; absolute publishes CSV poses unchanged"
        ),
    )
    parser.add_argument("--speed-scale", type=scale_float, default=0.2)
    parser.add_argument("--position-scale", type=scale_float, default=0.25)
    parser.add_argument("--orientation-scale", type=scale_float, default=0.25)
    parser.add_argument("--start-index", type=int, default=0)
    parser.add_argument("--end-index", type=int)
    parser.add_argument("--publish-rate", type=positive_float, default=60.0)
    parser.add_argument("--frame", default="base")
    parser.add_argument("--ee-frame", default="grasp_point")
    parser.add_argument("--target-topic", default="/yam_follower/target_pose")
    parser.add_argument("--status-topic", default="/servo_node/status")
    parser.add_argument("--joint-state-topic", default="/joint_states")
    parser.add_argument("--ik-service", default="/compute_ik")
    parser.add_argument("--ik-group", default="yam_arm")
    parser.add_argument("--ik-timeout", type=positive_float, default=0.05)
    parser.add_argument("--ik-stride", type=int, default=1)
    parser.add_argument("--max-joint-step", type=positive_float, default=0.15)
    parser.add_argument("--ik-report", default="trajectory_ik_report.csv")
    parser.add_argument(
        "--anchor-offset",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=(0.0, 0.0, 0.0),
        help=(
            "planning-frame translation applied only during --check-ik; use "
            "it to test a virtual start TCP without moving the robot"
        ),
    )
    parser.add_argument(
        "--anchor-rpy-offset-deg",
        type=float,
        nargs=3,
        metavar=("ROLL", "PITCH", "YAW"),
        default=(0.0, 0.0, 0.0),
        help=(
            "planning-frame RPY rotation in degrees applied only to the "
            "virtual IK anchor orientation"
        ),
    )
    parser.add_argument("--countdown", type=float, default=5.0)
    parser.add_argument("--hold-seconds", type=positive_float, default=1.0)
    parser.add_argument("--tf-timeout", type=positive_float, default=5.0)
    parser.add_argument("--max-start-position-error", type=positive_float, default=0.03)
    parser.add_argument("--max-start-orientation-error", type=positive_float, default=0.25)
    parser.add_argument(
        "--no-abort-on-servo-warning",
        dest="abort_on_servo_warning",
        action="store_false",
        help="continue even for Servo halt statuses (unsafe; not recommended)",
    )
    parser.set_defaults(abort_on_servo_warning=True)
    args = parser.parse_args(argv)
    if args.countdown < 0.0:
        parser.error("--countdown must be non-negative")
    if args.ik_stride < 1:
        parser.error("--ik-stride must be at least 1")
    if args.start_index < 0:
        parser.error("--start-index must be non-negative")
    if args.end_index is not None and args.end_index < args.start_index:
        parser.error("--end-index must not be smaller than --start-index")
    if args.execute and args.check_ik:
        parser.error("--execute and --check-ik cannot be used together")
    if args.execute and (
        any(abs(value) > 1e-12 for value in args.anchor_offset)
        or any(abs(value) > 1e-12 for value in args.anchor_rpy_offset_deg)
    ):
        parser.error(
            "--anchor-offset and --anchor-rpy-offset-deg are only allowed "
            "with --check-ik"
        )
    return args


def main(argv=None):
    args = parse_arguments(sys.argv[1:] if argv is None else argv)
    csv_path = os.path.abspath(os.path.expanduser(args.csv_path))
    try:
        all_samples = load_trajectory(csv_path)
    except (OSError, ValueError) as error:
        print(f"CSV validation failed: {error}", file=sys.stderr)
        return 2

    end_index = (
        len(all_samples) - 1 if args.end_index is None else args.end_index
    )
    if args.start_index >= len(all_samples) or end_index >= len(all_samples):
        print(
            f"CSV index range must be within 0..{len(all_samples) - 1}",
            file=sys.stderr,
        )
        return 2
    selected = all_samples[args.start_index : end_index + 1]
    selected_start_time = selected[0][0]
    samples = [
        (stamp - selected_start_time, position, orientation)
        for stamp, position, orientation in selected
    ]
    args.index_offset = args.start_index

    max_offset, max_rotation, max_speed = trajectory_statistics(samples)
    duration = samples[-1][0]
    print(f"CSV: {csv_path}")
    print(f"Samples: {len(samples)}")
    print(f"CSV index range: {args.start_index}..{end_index}")
    print(f"Source duration: {duration:.3f} s")
    print(f"Playback duration: {duration / args.speed_scale:.3f} s")
    print(f"Mode: {args.mode}")
    print(f"Speed scale: {args.speed_scale:.3f}")
    print(f"Position/orientation scale: {args.position_scale:.3f}/{args.orientation_scale:.3f}")
    print(f"Maximum source position offset: {max_offset:.3f} m")
    print(f"Maximum source orientation offset: {math.degrees(max_rotation):.1f} deg")
    print(f"Estimated scaled peak linear speed: {max_speed * args.speed_scale * args.position_scale:.3f} m/s")
    if not args.execute and not args.check_ik:
        print(
            "Dry run only: no ROS topic was published. Add --check-ik for "
            "offline IK validation or --execute to run."
        )
        return 0

    rclpy.init()
    node = CsvPosePlayer(args, samples)
    return_code = 1
    try:
        if args.check_ik:
            return_code = node.check_ik()
        else:
            node.play()
            return_code = 0
    except KeyboardInterrupt:
        node.get_logger().warning("Playback interrupted; target publication stopped")
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
