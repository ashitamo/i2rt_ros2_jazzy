#!/usr/bin/env python3
"""Teleop-only extensions around the existing YAM ROS 2 hardware node."""

import numpy as np
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from i2rt_msgs.msg import TeachingHandleState
from i2rt_yam_driver.yam_hardware_interface import YAMHardwareInterface


class TeleopYAMHardwareInterface(YAMHardwareInterface):
    """Add teaching-handle and bilateral-gain interfaces to the YAM driver."""

    def __init__(self):
        super().__init__()
        self.declare_parameter("teleop_role", "follower")
        self.teleop_role = str(self.get_parameter("teleop_role").value)
        if self.teleop_role not in ("leader", "follower"):
            raise ValueError("teleop_role must be 'leader' or 'follower'")

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._teleop_base_kp = np.asarray(self.robot._kp, dtype=float).copy()
        self._teleop_base_kd = np.asarray(self.robot._kd, dtype=float).copy()
        self._previous_buttons = (False, False)
        self._current_gain_scale = 0.0
        self._teaching_handle_pub = None
        self._gain_scale_sub = None

        if self.teleop_role == "leader":
            # Start compliant so a target cannot briefly apply full stiffness
            # before the coordinator's gain-scale message is processed.
            self.robot.update_kp_kd(
                np.zeros_like(self._teleop_base_kp),
                np.zeros_like(self._teleop_base_kd),
            )
            self._gain_scale_sub = self.create_subscription(
                Float64,
                f"/{self.robot_name}/pd_gain_scale",
                self._gain_scale_callback,
                10,
            )
            self._teaching_handle_pub = self.create_publisher(
                TeachingHandleState,
                f"/{self.robot_name}/teaching_handle_state",
                sensor_qos,
            )

        self.get_logger().info(
            f"Dedicated teleop hardware ready as {self.teleop_role}"
        )

    def _gain_scale_callback(self, msg: Float64) -> None:
        scale = float(msg.data)
        if not np.isfinite(scale) or scale < 0.0:
            self.get_logger().error(f"Invalid bilateral Kp scale: {scale}")
            return
        if scale > 0.0 and self._current_gain_scale <= 0.0:
            # Never reactivate stiffness around a target retained from a
            # previous mode or episode. Establish a fresh measured hold first.
            measured = np.asarray(self.robot.get_joint_pos(), dtype=float)
            self.update_arm_target(measured[:6])
        self.robot.update_kp_kd(
            self._teleop_base_kp * scale,
            np.zeros_like(self._teleop_base_kd),
        )
        self._current_gain_scale = scale
        self.get_logger().info(f"Bilateral Kp scale set to {scale:.3f}")

    def control_loop_callback(self) -> None:
        super().control_loop_callback()
        if self.teleop_role == "leader":
            self._publish_teaching_handle_state()

    def _publish_teaching_handle_state(self) -> None:
        if self._teaching_handle_pub is None:
            return
        try:
            encoder_states = self.robot.motor_chain.get_same_bus_device_states()
            if not encoder_states:
                return
            encoder = encoder_states[0]
            inputs = list(encoder.io_inputs)
            button1 = bool(inputs[0] > 0.5) if len(inputs) > 0 else False
            button2 = bool(inputs[1] > 0.5) if len(inputs) > 1 else False
            previous1, previous2 = self._previous_buttons

            msg = TeachingHandleState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.trigger_position = float(
                np.clip(1.0 - float(encoder.position), 0.0, 1.0)
            )
            msg.button1_pressed = button1
            msg.button2_pressed = button2
            msg.button1_rising_edge = button1 and not previous1
            msg.button1_falling_edge = previous1 and not button1
            msg.button2_rising_edge = button2 and not previous2
            msg.button2_falling_edge = previous2 and not button2
            self._previous_buttons = (button1, button2)
            self._teaching_handle_pub.publish(msg)
        except Exception as error:
            self.get_logger().error(
                f"Teaching-handle read failed: {error}",
                throttle_duration_sec=1.0,
            )

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TeleopYAMHardwareInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
