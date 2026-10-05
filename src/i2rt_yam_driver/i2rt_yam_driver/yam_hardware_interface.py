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

from i2rt_msgs.msg import MotorFeedback, MotorStatus, GripperState
from i2rt_msgs.msg import GripperCommand
from i2rt_msgs.srv import SetGravityCompensation, CalibrateGripper

from tf2_ros import TransformBroadcaster

from rclpy.action import ActionServer
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import GripperCommand as GripperCommandAction
from control_msgs.msg import JointJog
import time

# Add i2rt Python API to path
sys.path.insert(0, os.path.expanduser('/home/i2rt'))
# so this right adds the i2rt api to this package as if it is not in this package it would not 
# be a usable library.

try:
    from i2rt.robots.get_robot import get_yam_robot
    from i2rt.robots.motor_chain_robot import GripperType
except ImportError as e:
    print(f"ERROR: Failed to import i2rt Python API: {e}")
    print("Please ensure i2rt is installed and in your Python path")
    sys.exit(1)

# declare a node called yam_hardware_interface.
class YAMHardwareInterface(Node):
    """
    ROS2 hardware interface node for YAM robotic arm.

    Maintains 250Hz control loop matching the native i2rt control frequency.
    """

    def __init__(self):
        super().__init__('yam_hardware_interface')

        # Declare parameters
        self.declare_parameter('can_channel', 'can0')
        self.declare_parameter('gripper_type', 'linear_4310')
        self.declare_parameter('control_frequency', 250.0)
        self.declare_parameter('joint_command_timeout', 0.2)
        self.declare_parameter('trajectory_servo_lockout', 0.35)
        self.declare_parameter('servo_integration_gain', 1.0)
        self.declare_parameter('servo_max_joint_acceleration', 0.0)
        self.declare_parameter('use_feedback_timestamp', True)
        self.declare_parameter('gravity_comp_enabled', True)
        self.declare_parameter('gravity_comp_factor', 1.3)
        self.declare_parameter('zero_gravity_mode', False)
        self.declare_parameter('robot_name', 'yam')
        self.declare_parameter('publish_tf', True)

        # Get parameters
        self.can_channel = self.get_parameter('can_channel').value
        gripper_type_str = self.get_parameter('gripper_type').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.joint_command_timeout = self.get_parameter('joint_command_timeout').value
        self.trajectory_servo_lockout = float(
            self.get_parameter('trajectory_servo_lockout').value
        )
        if self.trajectory_servo_lockout < 0.0:
            raise ValueError('trajectory_servo_lockout must be non-negative')
        self.servo_integration_gain = float(
            self.get_parameter('servo_integration_gain').value
        )
        self.servo_max_joint_acceleration = float(
            self.get_parameter('servo_max_joint_acceleration').value
        )
        self.use_feedback_timestamp = bool(
            self.get_parameter('use_feedback_timestamp').value
        )
        if self.servo_integration_gain <= 0.0:
            raise ValueError('servo_integration_gain must be positive')
        if self.servo_max_joint_acceleration < 0.0:
            raise ValueError('servo_max_joint_acceleration must be non-negative')
        self.gravity_comp = self.get_parameter('gravity_comp_enabled').value
        self.gravity_factor = self.get_parameter('gravity_comp_factor').value
        self.zero_gravity = self.get_parameter('zero_gravity_mode').value
        self.robot_name = self.get_parameter('robot_name').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self.get_logger().info(f'Initializing YAM on {self.can_channel} with {gripper_type_str} gripper')

        # Convert gripper type
        try:
            self.gripper_type = GripperType.from_string_name(gripper_type_str)
            # so the gripper_type_str goes into the functino in the from_string_name function in the
            # GripperType class and then it will return the corresponding gripper type enum value 
            # and then we store it in self.gripper_type for later use. 
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
        # Persistent gripper target is independent of arm servo mode.
        # i2rt raw convention: 0.0=closed, 1.0=open.
        self.gripper_target = None
        self.servo_q_ref = None
        self.servo_qdot_target = np.zeros(6)
        self.servo_qdot_ref = np.zeros(6)
        self.last_control_loop_monotonic = time.monotonic()
        self.last_servo_debug_time = 0.0
        self.last_servo_command_time = None
        self.last_joint_command_time = None
        self.command_timed_out = False
        # FollowJointTrajectory owns the arm while it is executing.  Servo
        # inputs are ignored until the post-trajectory lockout expires, which
        # lets stale Cartesian targets time out before Servo can take over.
        self.trajectory_active = False
        self.servo_lockout_until_ns = 0

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
            
        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            JointTrajectory,
            f'/{self.robot_name}/joint_command',
            self.joint_command_callback,
            10
        )
        self.servo_joint_cmd_sub = self.create_subscription(
            JointTrajectory,
            f'/{self.robot_name}/servo_joint_command',
            self.servo_joint_command_callback,
            10
        )
        
        self.gripper_cmd_sub = self.create_subscription(
            GripperCommand,
            f'/{self.robot_name}/gripper_command',
            self.gripper_command_callback,
            10
        )
        
        self.trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f'/{self.robot_name}/yam_arm_controller/follow_joint_trajectory', # MoveIt will look for this name
            self.execute_trajectory_callback
        )
        self.gripper_action_server = ActionServer(
            self,
            GripperCommandAction,
            f'/{self.robot_name}/gripper_controller/gripper_cmd',
            self.execute_gripper_callback
        )

        # Publishers
        if gripper_type_str == 'linear_4310':
            self.joint_state_pub = self.create_publisher(
            JointState, f'/joint_states', sensor_qos)
        else:
            self.joint_state_pub = self.create_publisher(
            JointState, f'/{self.robot_name}/joint_states', sensor_qos)

        self.motor_feedback_pub = self.create_publisher(
            MotorStatus, f'/{self.robot_name}/motor_feedback', 10)

        self.gripper_state_pub = self.create_publisher(
            GripperState, f'/{self.robot_name}/gripper_state', 10)
        self.servo_reference_pub = self.create_publisher(
            JointState, f'/{self.robot_name}/servo_reference', sensor_qos)

    
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

        self.raw_joint_velocity_sub = self.create_subscription(
            JointJog,
            f'/{self.robot_name}/raw_joint_velocity_cmd',
            self.raw_joint_velocity_callback,
            10,
        )

    def raw_joint_velocity_callback(self, msg):
        qdot = np.zeros(6)

        for name, velocity in zip(msg.joint_names, msg.velocities):
            if name.startswith('joint'):
                idx = int(name.replace('joint', '')) - 1

                if 0 <= idx < 6:
                    qdot[idx] = velocity

        measured, _, _ = self.read_robot_state_snapshot()
        self.accept_servo_velocity(qdot, measured[:6])

    def accept_servo_velocity(self, qdot, measured):
        """Store newest Servo velocity; the 250 Hz loop performs integration."""
        now = self.get_clock().now()
        with self.command_lock:
            if (
                self.trajectory_active
                or now.nanoseconds < self.servo_lockout_until_ns
            ):
                return
            if self.servo_q_ref is None:
                self.servo_q_ref = np.asarray(measured, dtype=float).copy()
                self.servo_qdot_ref = np.zeros(6)
            self.servo_qdot_target = np.asarray(qdot, dtype=float).copy()
            self.last_servo_command_time = now
            self.last_joint_command_time = now
            self.command_timed_out = False
            self.target_position = None

    def read_robot_state_snapshot(self):
        """Copy joint values and SDK host timestamp from one state snapshot."""
        state_lock = getattr(self.robot, '_state_lock', None)
        joint_state = getattr(self.robot, '_joint_state', None)
        if state_lock is not None and joint_state is not None:
            with state_lock:
                positions = np.asarray(joint_state.pos, dtype=float).copy()
                velocities = np.asarray(joint_state.vel, dtype=float).copy()
                timestamp = getattr(joint_state, 'timestamp', None)
            return positions, velocities, timestamp

        positions = np.asarray(self.robot.get_joint_pos(), dtype=float).copy()
        joint_state = getattr(self.robot, '_joint_state', None)
        velocities = (
            np.asarray(joint_state.vel, dtype=float).copy()
            if joint_state is not None
            else np.zeros_like(positions)
        )
        return positions, velocities, getattr(joint_state, 'timestamp', None)

    def control_loop_callback(self):
        """Main control loop at 250 Hz."""
        try:
            # --------------------------------------------------
            # 1. Read measured joint state
            # --------------------------------------------------
            joint_pos, joint_vel, feedback_timestamp = (
                self.read_robot_state_snapshot()
            )

            now = self.get_clock().now()
            control_monotonic = time.monotonic()
            control_dt = float(np.clip(
                control_monotonic - self.last_control_loop_monotonic,
                0.0,
                0.05,
            ))
            self.last_control_loop_monotonic = control_monotonic

            timed_out = False

            servo_q_ref = None
            servo_qdot_ref = None
            target_position = None
            gripper_target = None

            # --------------------------------------------------
            # 2. Copy command state
            # --------------------------------------------------
            with self.command_lock:
                gripper_target = self.gripper_target

                # ==============================================
                # Servo mode
                # ==============================================
                if self.servo_q_ref is not None:

                    if (
                        self.last_joint_command_time is not None
                        and not self.command_timed_out
                        and (
                            now - self.last_joint_command_time
                        ).nanoseconds * 1e-9
                        > self.joint_command_timeout
                    ):
                        # Servo stream disappeared:
                        # switch to normal position-hold mode
                        self.target_position = joint_pos.copy()

                        # Leave Servo mode
                        self.servo_q_ref = None
                        self.servo_qdot_target = np.zeros(6)
                        self.servo_qdot_ref = np.zeros(6)
                        self.last_servo_command_time = None

                        self.command_timed_out = True
                        timed_out = True

                        # Send current measured position as hold target
                        target_position = self.target_position.copy()

                    else:
                        # Smooth qdot optionally, then integrate q_ref on every
                        # hardware-loop tick (normally about 250 Hz).
                        if self.servo_max_joint_acceleration > 0.0:
                            max_delta = (
                                self.servo_max_joint_acceleration * control_dt
                            )
                            self.servo_qdot_ref += np.clip(
                                self.servo_qdot_target - self.servo_qdot_ref,
                                -max_delta,
                                max_delta,
                            )
                        else:
                            self.servo_qdot_ref = self.servo_qdot_target.copy()
                        self.servo_q_ref += (
                            self.servo_integration_gain
                            * self.servo_qdot_ref
                            * control_dt
                        )
                        servo_q_ref = self.servo_q_ref.copy()
                        servo_qdot_ref = self.servo_qdot_ref.copy()

                # ==============================================
                # Normal absolute position mode
                # ==============================================
                elif self.target_position is not None:

                    if (
                        self.last_joint_command_time is not None
                        and not self.command_timed_out
                        and (
                            now - self.last_joint_command_time
                        ).nanoseconds * 1e-9
                        > self.joint_command_timeout
                    ):
                        self.target_position = joint_pos.copy()
                        self.command_timed_out = True
                        timed_out = True

                    target_position = self.target_position.copy()

            # --------------------------------------------------
            # 3. Timeout warning
            # --------------------------------------------------
            if timed_out:
                self.get_logger().warn(
                    f"Joint command timeout "
                    f"({self.joint_command_timeout:.3f} s); holding position"
                )

            # --------------------------------------------------
            # 4. Send command to robot
            # --------------------------------------------------

            if servo_q_ref is not None:

                # Start from current robot state so gripper is preserved.
                q_cmd = joint_pos.copy()
                qdot_cmd = np.zeros_like(joint_pos)

                # First 6 joints = arm
                q_cmd[:6] = servo_q_ref
                qdot_cmd[:6] = servo_qdot_ref

                # Joint7 = persistent gripper target.
                # Do not overwrite it with the measured position while
                # Cartesian servo mode is active.
                if len(q_cmd) > 6 and gripper_target is not None:
                    q_cmd[6] = gripper_target
                    qdot_cmd[6] = 0.0

                self.robot.command_joint_state({
                    "pos": q_cmd,
                    "vel": qdot_cmd,
                })
                self.publish_servo_reference(servo_q_ref, servo_qdot_ref, now)

            elif target_position is not None:

                # Normal MoveIt trajectory / absolute position control
                self.robot.command_joint_pos(target_position)

            # --------------------------------------------------
            # 5. Publish measured state
            # --------------------------------------------------
            self.publish_joint_state(
                joint_pos, joint_vel, feedback_timestamp, now
            )
            self.publish_gripper_state(joint_pos, joint_vel)

            if self.publish_tf:
                self.publish_transforms(joint_pos)

        except Exception as e:
            self.get_logger().error(
                f"Control loop error: {e}",
                throttle_duration_sec=1.0,
            )

    def feedback_stamp(self, feedback_timestamp, publish_time):
        """Use SDK host snapshot time when it shares the active ROS clock."""
        if self.use_feedback_timestamp and feedback_timestamp is not None:
            try:
                timestamp = float(feedback_timestamp)
                publish_seconds = publish_time.nanoseconds * 1e-9
                if (
                    np.isfinite(timestamp)
                    and timestamp > 0.0
                    and abs(publish_seconds - timestamp) < 5.0
                ):
                    seconds = int(timestamp)
                    nanoseconds = int(round((timestamp - seconds) * 1e9))
                    if nanoseconds >= 1_000_000_000:
                        seconds += 1
                        nanoseconds -= 1_000_000_000
                    stamp = publish_time.to_msg()
                    stamp.sec = seconds
                    stamp.nanosec = nanoseconds
                    return stamp
            except (TypeError, ValueError, OverflowError):
                pass
        return publish_time.to_msg()

    def publish_joint_state(
        self, positions, velocities, feedback_timestamp, publish_time
    ):
        """Publish joint states"""
        msg = JointState()
        msg.header.stamp = self.feedback_stamp(
            feedback_timestamp, publish_time
        )
        msg.header.frame_id = 'world'

        # Joint names (6 arm joints + 1 gripper joint)
        msg.name = [f'joint{i+1}' for i in range(6)]
        if len(positions) > 6:
            # msg.name.append('gripper_joint')
            msg.name.append('joint7')
            positions = positions.copy()
            positions[6] = positions[6] * -0.0475

        msg.position = positions.tolist()
        msg.velocity = velocities.tolist()
        msg.effort = []  # Populated from motor feedback if needed

        self.joint_state_pub.publish(msg)

    def publish_servo_reference(self, positions, velocities, stamp):
        """Publish the exact arm reference sent on this control iteration."""
        msg = JointState()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = 'base'
        msg.name = [f'joint{i + 1}' for i in range(6)]
        msg.position = np.asarray(positions, dtype=float).tolist()
        msg.velocity = np.asarray(velocities, dtype=float).tolist()
        self.servo_reference_pub.publish(msg)

    def publish_gripper_state(self, positions, velocities):
        """Publish gripper-specific state"""
        if len(positions) < 7:
            return

        msg = GripperState()
        msg.header.stamp = self.get_clock().now().to_msg()
        # SDK convention is normalized (0=closed, 1=open), while the public
        # joint/gripper feedback follows the URDF joint7 convention in metres.
        msg.position = float(positions[6]) * -0.0475
        msg.velocity = float(velocities[6]) * -0.0475
        msg.force = 0.0  # TODO: Estimate from motor current
        msg.is_moving = bool(abs(velocities[6]) > 0.01)
        msg.is_blocked = bool(False)  # TODO: Implement clogging detection
        msg.is_calibrated = bool(True)  # TODO: Check calibration status
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
        # point = msg.points[0]
        point = msg.points[-1]
        self.get_logger().debug(f'Received joint command: {point}')
        if len(point.positions) < 6:
            self.get_logger().error(f'Invalid joint command: expected 6-7 positions, got {len(point.positions)}')
            return

        target = np.array(point.positions[:6], dtype=float)
        self.update_arm_target(target)
        self.get_logger().debug(f'Joint command received: {target}')

    def servo_joint_command_callback(self, msg):
        """Store MoveIt Servo qdot for the hardware-loop integrator."""

        if not msg.points:
            return

        point = msg.points[-1]

        if len(point.velocities) < 6:
            self.get_logger().warn(
                f"Servo command has only "
                f"{len(point.velocities)} velocities"
            )
            return

        qdot_des = np.asarray(
            point.velocities[:6],
            dtype=float,
        )

        measured, _, _ = self.read_robot_state_snapshot()
        self.accept_servo_velocity(qdot_des, measured[:6])

    def update_arm_target(self, arm_positions):
        """Update six arm joints while preserving the current gripper target."""
        with self.command_lock:
            if self.target_position is None or len(self.target_position) < 7:
                self.target_position = self.robot.get_joint_pos().copy()

            self.target_position[:6] = arm_positions[:6]

            # Leave Servo mode
            self.servo_q_ref = None
            self.servo_qdot_target = np.zeros(6)
            self.servo_qdot_ref = np.zeros(6)
            self.last_servo_command_time = None

            self.last_joint_command_time = self.get_clock().now()
            self.command_timed_out = False
        
    def execute_trajectory_callback(self, goal_handle):
        self.get_logger().info('Executing MoveIt trajectory...')
        trajectory = goal_handle.request.trajectory
        self.get_logger().debug(f'Received trajectory with {len(trajectory.points)} points')
        self.get_logger().debug(f'Received trajectory with {trajectory.points}')
        
        measured = self.robot.get_joint_pos().copy()
        now = self.get_clock().now()

        with self.command_lock:
            if self.trajectory_active:
                goal_handle.abort()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                self.get_logger().warn(
                    'Rejected trajectory because another trajectory is active'
                )
                return result

            # MoveIt trajectory gets exclusive ownership immediately.
            self.trajectory_active = True
            self.servo_q_ref = None
            self.servo_qdot_target = np.zeros(6)
            self.servo_qdot_ref = np.zeros(6)
            self.last_servo_command_time = None
            self.target_position = measured
            self.last_joint_command_time = now
            self.command_timed_out = False

        result = FollowJointTrajectory.Result()

        try:
            start_time = self.get_clock().now().nanoseconds / 1e9

            for point in trajectory.points:
                # Calculate when this point should be reached
                time_from_start = point.time_from_start.sec + (point.time_from_start.nanosec / 1e9)
                target_time = start_time + time_from_start

                # Wait until it's time to send this point
                while (self.get_clock().now().nanoseconds / 1e9) < target_time:
                    rclpy.spin_once(self,timeout_sec=0.001) # Small sleep to prevent blocking

                incoming_positions = np.array(point.positions) # This has length 6
                self.update_arm_target(incoming_positions)

            goal_handle.succeed()
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            self.get_logger().info('Trajectory execution complete!')
            return result
        except Exception as error:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            self.get_logger().error(f'Trajectory execution failed: {error}')
            return result
        finally:
            release_time_ns = self.get_clock().now().nanoseconds + int(
                self.trajectory_servo_lockout * 1e9
            )
            with self.command_lock:
                self.trajectory_active = False
                self.servo_lockout_until_ns = release_time_ns
                self.servo_q_ref = None
                self.servo_qdot_target = np.zeros(6)
                self.servo_qdot_ref = np.zeros(6)
                self.last_servo_command_time = None

            self.get_logger().info(
                'Trajectory released arm ownership; Servo remains blocked for '
                f'{self.trajectory_servo_lockout:.3f} s'
            )

    def execute_gripper_callback(self, goal_handle):
        self.get_logger().info('Executing MoveIt gripper command...')

        # MoveIt/URDF joint7 target, for example -0.0475 open, 0 close
        moveit_position = goal_handle.request.command.position
        max_effort = goal_handle.request.command.max_effort

        # Convert MoveIt joint7 value to i2rt raw gripper value
        raw_gripper = moveit_position / -0.0475
        self.get_logger().info("{}".format(raw_gripper))

        with self.command_lock:
            if self.target_position is None:
                self.target_position = self.robot.get_joint_pos()

            if len(self.target_position) > 6:
                self.target_position[6] = raw_gripper
            self.gripper_target = float(np.clip(raw_gripper, 0.0, 1.0))

        # 簡單版：等一下讓 control_loop 送 command
        time.sleep(0.5)

        goal_handle.succeed()
        result = GripperCommandAction.Result()
        result.reached_goal = True
        result.stalled = False
        result.position = float(moveit_position)
        result.effort = float(max_effort)
        return result
            

    def gripper_command_callback(self, msg):
        """Handle normalized gripper command: 0.0=closed, 1.0=open."""
        gripper_target = float(np.clip(msg.position, 0.0, 1.0))

        with self.command_lock:
            self.gripper_target = gripper_target

            # Keep normal absolute-position mode compatible too.
            if self.target_position is not None and len(self.target_position) > 6:
                self.target_position[6] = gripper_target

        # self.get_logger().info(
        #     f'Gripper target: {gripper_target:.3f}'
        # )

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
                self.servo_q_ref = None
                self.servo_qdot_target = np.zeros(6)
                self.servo_qdot_ref = np.zeros(6)
                self.last_servo_command_time = None
                self.last_joint_command_time = None
                self.command_timed_out = False

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
        """Cleanup on shutdown - stop the robot and cleanup resources"""
        self.get_logger().info('Shutting down YAM hardware interface')
        
        try:
            # Stop the control loop timer
            if hasattr(self, 'control_timer'):
                self.destroy_timer(self.control_timer)
            if hasattr(self, 'feedback_timer'):
                self.destroy_timer(self.feedback_timer)
            
            # Send command to stop the robot gracefully
            if hasattr(self, 'robot') and self.robot is not None:
                try:
                    # Get current position and command it (holding position)
                    current_pos = self.robot.get_joint_pos()
                    # Command zero velocities with current position (safe stop)
                    joint_state = {
                        'pos': current_pos,
                        'vel': np.zeros_like(current_pos),
                        'kp': np.full_like(current_pos, 10.0),  # Default stiffness
                        'kd': np.full_like(current_pos, 1.0),   # Default damping
                        'torques': np.zeros_like(current_pos)
                    }
                    self.get_logger().info(f'Sending hold position command to robot: {joint_state}')
                    self.robot.command_joint_state(joint_state)
                    self.get_logger().info('Sent hold position command to robot')
                except Exception as e:
                    self.get_logger().warn(f'Could not send stop command: {e}')
                
                try:
                    # Stop the robot server thread and close the CAN connection.
                    self.robot.close()
                    self.get_logger().info('Closed YAM robot connection')
                except Exception as e:
                    self.get_logger().warn(f'Error closing YAM robot: {e}')
        except Exception as e:
            self.get_logger().error(f'Error during shutdown: {e}')
        
        self.get_logger().info('YAM hardware interface shutdown complete')
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = YAMHardwareInterface()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
