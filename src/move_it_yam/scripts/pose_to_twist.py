#!/usr/bin/env python3
"""Track an absolute TCP pose with bounded Cartesian twist commands."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1e-9:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in quaternion)


def quaternion_multiply(lhs, rhs):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return -x, -y, -z, w


def rotate_vector(quaternion, vector):
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, (*vector, 0.0)),
        quaternion_conjugate(quaternion),
    )
    return rotated[:3]


def clamp_vector(vector, maximum_norm):
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= maximum_norm or norm < 1e-12:
        return vector
    scale = maximum_norm / norm
    return tuple(value * scale for value in vector)


def orientation_error(target, current):
    """Return the shortest rotation vector from current to target."""
    error = normalize_quaternion(
        quaternion_multiply(target, quaternion_conjugate(current))
    )
    if error[3] < 0.0:
        error = tuple(-value for value in error)

    vector_norm = math.sqrt(sum(value * value for value in error[:3]))
    if vector_norm < 1e-9:
        return 0.0, 0.0, 0.0

    angle = 2.0 * math.atan2(vector_norm, max(0.0, error[3]))
    return tuple(value * angle / vector_norm for value in error[:3])


class PoseToTwist(Node):
    def __init__(self):
        super().__init__("pose_to_twist")

        self.declare_parameter("target_topic", "target_pose")
        self.declare_parameter("command_topic", "servo_node/delta_twist_cmds")
        self.declare_parameter("servo_start_service", "servo_node/start_servo")
        self.declare_parameter("planning_frame", "base")
        self.declare_parameter("ee_frame", "gripper")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("target_timeout", 0.15)
        self.declare_parameter("linear_gain", 2.0)
        self.declare_parameter("angular_gain", 2.0)
        self.declare_parameter("max_linear_speed", 0.15)
        self.declare_parameter("max_angular_speed", 0.5)
        self.declare_parameter("position_tolerance", 0.001)
        self.declare_parameter("orientation_tolerance", 0.01)

        self.target_topic = self.get_parameter("target_topic").value
        self.command_topic = self.get_parameter("command_topic").value
        self.planning_frame = self.get_parameter("planning_frame").value
        self.ee_frame = self.get_parameter("ee_frame").value
        self.target_timeout = self.get_parameter("target_timeout").value
        self.linear_gain = self.get_parameter("linear_gain").value
        self.angular_gain = self.get_parameter("angular_gain").value
        self.max_linear_speed = self.get_parameter(
            "max_linear_speed"
        ).value
        self.max_angular_speed = self.get_parameter(
            "max_angular_speed"
        ).value
        self.position_tolerance = self.get_parameter(
            "position_tolerance"
        ).value
        self.orientation_tolerance = self.get_parameter(
            "orientation_tolerance"
        ).value

        publish_rate = self.get_parameter("publish_rate").value
        if publish_rate <= 0.0:
            raise ValueError("publish_rate must be greater than zero")

        command_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.target_subscriber = self.create_subscription(
            PoseStamped, self.target_topic, self.target_callback, command_qos
        )
        self.twist_publisher = self.create_publisher(
            TwistStamped, self.command_topic, command_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        start_service = self.get_parameter("servo_start_service").value
        self.start_client = self.create_client(Trigger, start_service)
        self.start_future = None
        self.servo_started = False
        self.start_timer = self.create_timer(0.5, self.try_start_servo)

        self.target_pose = None
        self.target_received_time = None
        self.stale_reported = False
        self.control_timer = self.create_timer(
            1.0 / publish_rate, self.control_loop
        )

        self.get_logger().info(
            f"Tracking {self.target_topic} in {self.planning_frame}; "
            f"publishing twists to {self.command_topic}"
        )

    def try_start_servo(self):
        if self.servo_started:
            self.start_timer.cancel()
            return
        if self.start_future is not None:
            return
        if not self.start_client.service_is_ready():
            return

        self.start_future = self.start_client.call_async(Trigger.Request())
        self.start_future.add_done_callback(self.start_servo_done)

    def start_servo_done(self, future):
        self.start_future = None
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - ROS future exceptions vary
            self.get_logger().error(
                f"Failed to call Servo start service: {error}"
            )
            return

        if response.success:
            self.servo_started = True
            self.get_logger().info("MoveIt Servo started")
        else:
            self.get_logger().error(
                f"MoveIt Servo did not start: {response.message}"
            )

    def target_callback(self, message):
        try:
            transformed = self.transform_target(message)
            quaternion = transformed.pose.orientation
            normalize_quaternion(
                (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            )
        except (TransformException, ValueError) as error:
            self.get_logger().warning(f"Rejected target pose: {error}")
            return

        self.target_pose = transformed
        self.target_received_time = self.get_clock().now()
        self.stale_reported = False

    def transform_target(self, message):
        source_frame = message.header.frame_id or self.planning_frame
        if source_frame == self.planning_frame:
            transformed = PoseStamped()
            transformed.header = message.header
            transformed.header.frame_id = self.planning_frame
            transformed.pose = message.pose
            return transformed

        transform = self.tf_buffer.lookup_transform(
            self.planning_frame, source_frame, rclpy.time.Time()
        )
        transform_rotation = normalize_quaternion(
            (
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            )
        )
        pose_rotation = normalize_quaternion(
            (
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            )
        )
        rotated_position = rotate_vector(
            transform_rotation,
            (
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            ),
        )

        transformed = PoseStamped()
        transformed.header.stamp = message.header.stamp
        transformed.header.frame_id = self.planning_frame
        transformed.pose.position.x = (
            transform.transform.translation.x + rotated_position[0]
        )
        transformed.pose.position.y = (
            transform.transform.translation.y + rotated_position[1]
        )
        transformed.pose.position.z = (
            transform.transform.translation.z + rotated_position[2]
        )
        transformed_rotation = quaternion_multiply(
            transform_rotation, pose_rotation
        )
        transformed.pose.orientation.x = transformed_rotation[0]
        transformed.pose.orientation.y = transformed_rotation[1]
        transformed.pose.orientation.z = transformed_rotation[2]
        transformed.pose.orientation.w = transformed_rotation[3]
        return transformed

    def control_loop(self):
        if not self.servo_started or self.target_pose is None:
            return

        age = (
            self.get_clock().now() - self.target_received_time
        ).nanoseconds * 1e-9
        if age > self.target_timeout:
            if not self.stale_reported:
                self.get_logger().warning(
                    f"Target pose is stale ({age:.3f} s); Servo will halt"
                )
                self.stale_reported = True
            return

        try:
            current = self.tf_buffer.lookup_transform(
                self.planning_frame, self.ee_frame, rclpy.time.Time()
            )
        except TransformException as error:
            self.get_logger().warning(
                f"Cannot read current TCP transform: {error}"
            )
            return

        target = self.target_pose.pose
        position_error = (
            target.position.x - current.transform.translation.x,
            target.position.y - current.transform.translation.y,
            target.position.z - current.transform.translation.z,
        )
        current_rotation = normalize_quaternion(
            (
                current.transform.rotation.x,
                current.transform.rotation.y,
                current.transform.rotation.z,
                current.transform.rotation.w,
            )
        )
        target_rotation = normalize_quaternion(
            (
                target.orientation.x,
                target.orientation.y,
                target.orientation.z,
                target.orientation.w,
            )
        )
        rotation_error = orientation_error(target_rotation, current_rotation)

        position_norm = math.sqrt(
            sum(value * value for value in position_error)
        )
        rotation_norm = math.sqrt(
            sum(value * value for value in rotation_error)
        )
        linear_command = (0.0, 0.0, 0.0)
        angular_command = (0.0, 0.0, 0.0)
        if position_norm > self.position_tolerance:
            linear_command = clamp_vector(
                tuple(self.linear_gain * value for value in position_error),
                self.max_linear_speed,
            )
        if rotation_norm > self.orientation_tolerance:
            angular_command = clamp_vector(
                tuple(self.angular_gain * value for value in rotation_error),
                self.max_angular_speed,
            )

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = self.planning_frame
        (
            command.twist.linear.x,
            command.twist.linear.y,
            command.twist.linear.z,
        ) = linear_command
        (
            command.twist.angular.x,
            command.twist.angular.y,
            command.twist.angular.z,
        ) = angular_command
        self.twist_publisher.publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = PoseToTwist()
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
