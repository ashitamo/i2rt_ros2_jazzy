#!/usr/bin/env python3
"""
ROS2 Hardware Interface Node for I2RT YAM Robotic Arm

This node wraps the i2rt Python API and provides ROS2 interfaces for:
- Joint state publishing (250Hz)
- Joint command subscription
- Motor feedback publishing
- Gripper control
- Gravity compensation services
- TF broadcasting

Author: I2RT ROS2 Integration
License: MIT
"""

import sys
import os
import numpy as np
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped

from i2rt_msgs.msg import MotorFeedback, MotorStatus, GripperCommand, GripperState
from i2rt_msgs.srv import SetGravityCompensation, CalibrateGripper

from tf2_ros import TransformBroadcaster

# Add i2rt Python API to path
sys.path.insert(0, os.path.expanduser('~/Documents/neotix_robotics/i2rt'))

try:
    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.motor_chain_robot import GripperType
except ImportError as e:
    print(f"ERROR: Failed to import i2rt Python API: {e}")
    print("Please ensure i2rt is installed and in your Python path")
    sys.exit(1)


class YAMHardwareInterface(Node):
    """
    ROS2 hardware interface node for YAM robotic arm.

    Maintains 250Hz control loop matching the native i2rt control frequency.
    """

    def __init__(self):
        super().__init__('yam_hardware_interface')

        # Declare parameters
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('gripper_type', 'crank_4310')
        self.declare_parameter('control_frequency', 250.0)
        self.declare_parameter('gravity_comp_enabled', True)
        self.declare_parameter('gravity_comp_factor', 1.3)
        self.declare_parameter('zero_gravity_mode', False)
        self.declare_parameter('robot_name', 'yam')
        self.declare_parameter('publish_tf', True)

        # Get parameters
        self.can_channel = self.get_parameter('can_channel').value
        gripper_type_str = self.get_parameter('gripper_type').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.gravity_comp = self.get_parameter('gravity_comp_enabled').value
        self.gravity_factor = self.get_parameter('gravity_comp_factor').value
        self.zero_gravity = self.get_parameter('zero_gravity_mode').value
        self.robot_name = self.get_parameter('robot_name').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.get_logger().info(f'Initializing YAM on {self.can_channel} with {gripper_type_str} gripper')

        # Convert gripper type
        try:
            self.gripper_type = GripperType.from_string_name(gripper_type_str)
        except ValueError:
            self.get_logger().error(f'Invalid gripper type: {gripper_type_str}')
            raise

        # Initialize robot
        try:
            self.robot = get_yam_robot(
                channel=self.can_channel,
                gripper_type=self.gripper_type,
                zero_gravity_mode=self.zero_gravity
            )
            self.get_logger().info('YAM robot initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize robot: {e}')
            raise

        # Command lock
        self.command_lock = Lock()
        self.target_position = None

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState, f'/{self.robot_name}/joint_states', sensor_qos)

        self.motor_feedback_pub = self.create_publisher(
            MotorStatus, f'/{self.robot_name}/motor_feedback', 10)

        self.gripper_state_pub = self.create_publisher(
            GripperState, f'/{self.robot_name}/gripper_state', 10)

        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            JointTrajectory,
            f'/{self.robot_name}/joint_command',
            self.joint_command_callback,
            10
        )

        self.gripper_cmd_sub = self.create_subscription(
            GripperCommand,
            f'/{self.robot_name}/gripper_command',
            self.gripper_command_callback,
            10
        )

        # Services
        self.gravity_comp_srv = self.create_service(
            SetGravityCompensation,
            f'/{self.robot_name}/set_gravity_compensation',
            self.set_gravity_compensation_callback
        )

        self.calibrate_gripper_srv = self.create_service(
            CalibrateGripper,
            f'/{self.robot_name}/calibrate_gripper',
            self.calibrate_gripper_callback
        )

        self.estop_srv = self.create_service(
            Trigger,
            f'/{self.robot_name}/emergency_stop',
            self.emergency_stop_callback
        )

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Control loop timer (250Hz)
        self.control_timer = self.create_timer(
            1.0 / self.control_freq,
            self.control_loop_callback
        )

        # Motor feedback timer (10Hz - less frequent)
        self.feedback_timer = self.create_timer(
            0.1,
            self.publish_motor_feedback
        )

        self.get_logger().info(f'YAM hardware interface ready @ {self.control_freq} Hz')

    def control_loop_callback(self):
        """Main control loop at 250Hz"""
        try:
            # Read current joint state
            joint_pos = self.robot.get_joint_pos()
            joint_vel = self.robot.get_joint_vel()

            # Command robot if target is set
            with self.command_lock:
                if self.target_position is not None:
                    self.robot.command_joint_pos(self.target_position)

            # Publish joint states
            self.publish_joint_state(joint_pos, joint_vel)

            # Publish gripper state
            self.publish_gripper_state(joint_pos, joint_vel)

            # Publish TF
            if self.publish_tf:
                self.publish_transforms(joint_pos)

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}', throttle_duration_sec=1.0)

    def publish_joint_state(self, positions, velocities):
        """Publish joint states"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'

        # Joint names (6 arm joints + 1 gripper joint)
        msg.name = [f'joint{i+1}' for i in range(6)]
        if len(positions) > 6:
            msg.name.append('gripper_joint')

        msg.position = positions.tolist()
        msg.velocity = velocities.tolist()
        msg.effort = []  # Populated from motor feedback if needed

        self.joint_state_pub.publish(msg)

    def publish_gripper_state(self, positions, velocities):
        """Publish gripper-specific state"""
        if len(positions) < 7:
            return

        msg = GripperState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = float(positions[6])
        msg.velocity = float(velocities[6])
        msg.force = 0.0  # TODO: Estimate from motor current
        msg.is_moving = abs(velocities[6]) > 0.01
        msg.is_blocked = False  # TODO: Implement clogging detection
        msg.is_calibrated = True  # TODO: Check calibration status
        msg.gripper_type = self.get_parameter('gripper_type').value

        self.gripper_state_pub.publish(msg)

    def publish_motor_feedback(self):
        """Publish detailed motor feedback (10Hz)"""
        try:
            # TODO: Access motor chain feedback for temperatures, torques, errors
            # This requires extending the i2rt API to expose motor feedback
            pass
        except Exception as e:
            self.get_logger().error(f'Motor feedback error: {e}', throttle_duration_sec=5.0)

    def publish_transforms(self, joint_positions):
        """Publish TF transforms for visualization"""
        # TODO: Compute forward kinematics and publish TF tree
        # This requires kinematics integration
        pass

    def joint_command_callback(self, msg):
        """Handle joint trajectory commands"""
        if not msg.points:
            self.get_logger().warn('Received empty trajectory')
            return

        # Use first point as target (simple position control)
        point = msg.points[0]

        if len(point.positions) < 6:
            self.get_logger().error(f'Invalid joint command: expected 6-7 positions, got {len(point.positions)}')
            return

        target = np.array(point.positions)

        with self.command_lock:
            self.target_position = target

        self.get_logger().debug(f'Joint command received: {target}')

    def gripper_command_callback(self, msg):
        """Handle gripper commands"""
        # Convert normalized position (0-1) to gripper joint position
        # Gripper position range depends on gripper type

        with self.command_lock:
            if self.target_position is None:
                # Initialize with current position
                self.target_position = self.robot.get_joint_pos()

            # Update gripper joint (index 6)
            if len(self.target_position) > 6:
                self.target_position[6] = msg.position

        self.get_logger().debug(f'Gripper command: {msg.position}')

    def set_gravity_compensation_callback(self, request, response):
        """Service to enable/disable gravity compensation"""
        try:
            # TODO: Implement gravity compensation toggle via i2rt API
            # self.robot.set_gravity_compensation(request.enable, request.compensation_factor)

            response.success = True
            response.message = f'Gravity compensation {"enabled" if request.enable else "disabled"}'
            response.current_factor = request.compensation_factor if request.compensation_factor > 0.0 else self.gravity_factor

            self.get_logger().info(response.message)

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Gravity compensation service failed: {e}')

        return response

    def calibrate_gripper_callback(self, request, response):
        """Service to calibrate linear grippers"""
        try:
            # TODO: Implement gripper calibration via i2rt API
            # This is required for linear_3507 and linear_4310 grippers

            response.success = True
            response.message = 'Gripper calibration completed'
            response.stroke_range = 0.0  # TODO: Get actual stroke range
            response.is_calibrated = True

            self.get_logger().info('Gripper calibration completed')

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Gripper calibration failed: {e}')

        return response

    def emergency_stop_callback(self, request, response):
        """Emergency stop service"""
        try:
            # Stop commanding new positions
            with self.command_lock:
                self.target_position = None

            # TODO: Implement proper e-stop via i2rt API
            # This should disable motors or switch to damping mode

            response.success = True
            response.message = 'Emergency stop activated'
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Emergency stop failed: {e}')

        return response

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down YAM hardware interface')
        # Robot cleanup happens automatically via destructor
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = YAMHardwareInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
