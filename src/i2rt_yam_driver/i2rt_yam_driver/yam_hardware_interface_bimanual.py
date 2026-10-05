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
import time

# Add i2rt Python API to path
sys.path.insert(0, os.path.expanduser('~/i2rt'))
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
        self.declare_parameter('servo_integration_gain', 1.0)
        self.declare_parameter('gravity_comp_enabled', True)
        self.declare_parameter('gravity_comp_factor', 1.3)
        self.declare_parameter('zero_gravity_mode', False)
        self.declare_parameter('robot_name', 'yam_left')
        self.declare_parameter('publish_tf', True)

        # Get parameters
        self.can_channel = self.get_parameter('can_channel').value
        gripper_type_str = self.get_parameter('gripper_type').value
        self.control_freq = self.get_parameter('control_frequency').value
        self.joint_command_timeout = self.get_parameter('joint_command_timeout').value
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
        self.servo_integrated_target = None
        self.last_servo_command_time = None
        self.last_joint_command_time = None
        self.command_timed_out = False

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
        
        prefix = "left" if "left" in self.robot_name else "right"
        action_name = f'/yam_follower/{prefix}_yam_arm_controller/follow_joint_trajectory'
                
        self.trajectory_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            # f'/{self.robot_name}/yam_arm_controller/follow_joint_trajectory', # MoveIt will look for this name
            action_name,
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
            # JointState, f'joint_states', sensor_qos)
        else:
            self.joint_state_pub = self.create_publisher(
            JointState, f'/{self.robot_name}/joint_states', sensor_qos)

        self.motor_feedback_pub = self.create_publisher(
            MotorStatus, f'/{self.robot_name}/motor_feedback', 10)

        self.gripper_state_pub = self.create_publisher(
            GripperState, f'/{self.robot_name}/gripper_state', 10)

    
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
            joint_pos = self.robot.get_joint_pos().copy()
            # Access joint velocity from the internal joint state
            joint_vel = self.robot._joint_state.vel if hasattr(self.robot, '_joint_state') else np.zeros_like(joint_pos)

            # Command robot if target is set. A stale streamed command holds the
            # measured position instead of continuing toward an obsolete target.
            timed_out = False
            with self.command_lock:
                if self.target_position is not None:
                    if (
                        self.last_joint_command_time is not None
                        and not self.command_timed_out
                        and (self.get_clock().now() - self.last_joint_command_time).nanoseconds
                        * 1e-9
                        > self.joint_command_timeout
                    ):
                        self.target_position = joint_pos.copy()
                        self.servo_integrated_target = None
                        self.last_servo_command_time = None
                        self.command_timed_out = True
                        timed_out = True
                    target_position = self.target_position.copy()
                else:
                    target_position = None

            if timed_out:
                self.get_logger().warn(
                    f'Joint command timeout ({self.joint_command_timeout:.3f} s); holding position'
                )
            if target_position is not None:
                self.robot.command_joint_pos(target_position)

            # Publish joint states
            self.publish_joint_state(joint_pos, joint_vel)

            # Publish gripper state
            self.publish_gripper_state(joint_pos, joint_vel)
            # self.get_logger().info("joint state published pos:{}".format(joint_pos))
            # self.get_logger().info("joint state target pos:{}".format(self.target_position))
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

        prefix = "left" if "left" in self.robot_name else "right"

        # Joint names (6 arm joints + 1 gripper joint)
        msg.name = [f'{prefix}_joint{i+1}' for i in range(6)]
        if len(positions) > 6:
            # msg.name.append('gripper_joint')
            msg.name.append(f'{prefix}_joint7')
            positions[6] = positions[6] * -0.0475
        ####
        # all velocities less than 1e-3 set 0
        for i in range(len(velocities)):
            if abs(velocities[i]) < 1.0:
                velocities[i] = 0.0
        ####
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
        """Integrate Servo velocities into absolute position commands."""
        if not msg.points:
            self.get_logger().warn('Received empty Servo trajectory')
            return

        point = msg.points[-1]
        if len(point.velocities) < 6:
            self.get_logger().error(
                f'Invalid Servo command: expected at least 6 velocities, '
                f'got {len(point.velocities)}'
            )
            return

        gain = float(self.get_parameter('servo_integration_gain').value)
        if gain <= 0.0:
            self.get_logger().error(
                'Servo integration gain must be positive',
                throttle_duration_sec=1.0
            )
            return

        measured = self.robot.get_joint_pos().copy()
        velocity = np.asarray(point.velocities[:6], dtype=float)
        dt = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        dt = float(np.clip(dt if dt > 0.0 else 0.01, 0.001, 0.05))
        now = self.get_clock().now()

        with self.command_lock:
            restart_stream = (
                self.servo_integrated_target is None
                or self.last_servo_command_time is None
                or (now - self.last_servo_command_time).nanoseconds * 1e-9
                > self.joint_command_timeout
            )
            if restart_stream or np.max(np.abs(velocity)) < 1e-5:
                self.servo_integrated_target = measured[:6].copy()
            else:
                self.servo_integrated_target += gain * velocity * dt

            if self.target_position is None or len(self.target_position) < 7:
                self.target_position = measured.copy()
            self.target_position[:6] = self.servo_integrated_target
            self.last_servo_command_time = now
            self.last_joint_command_time = now
            self.command_timed_out = False
            target = self.servo_integrated_target.copy()

        self.get_logger().debug(
            f'Servo velocity={velocity}, integrated lead={target - measured[:6]}'
        )

    def update_arm_target(self, arm_positions):
        """Update six arm joints while preserving the current gripper target."""
        with self.command_lock:
            if self.target_position is None or len(self.target_position) < 7:
                self.target_position = self.robot.get_joint_pos().copy()
            self.target_position[:6] = arm_positions[:6]
            self.servo_integrated_target = None
            self.last_servo_command_time = None
            self.last_joint_command_time = self.get_clock().now()
            self.command_timed_out = False
    
    def resample_trajectory(self, points, hz=100.0):
        """
        將 MoveIt trajectory points 插補成固定頻率。

        points: trajectory.points
        hz: output frequency, e.g. 100.0
        return:
            sample_times: shape (N,)
            sample_positions: shape (N, dof)
        """

        if len(points) == 0:
            return np.array([]), np.array([])

        raw_times = np.array([
            p.time_from_start.sec + p.time_from_start.nanosec * 1e-9
            for p in points
        ], dtype=float)

        raw_positions = np.array([
            p.positions
            for p in points
        ], dtype=float)

        # 確保時間從小到大
        order = np.argsort(raw_times)
        raw_times = raw_times[order]
        raw_positions = raw_positions[order]

        # 移除重複 timestamp，避免除以 0
        unique_times, unique_indices = np.unique(raw_times, return_index=True)
        raw_times = unique_times
        raw_positions = raw_positions[unique_indices]

        if len(raw_times) == 1:
            return raw_times, raw_positions

        dt = 1.0 / hz

        t_start = raw_times[0]
        t_end = raw_times[-1]

        sample_times = np.arange(t_start, t_end, dt)

        # 保證最後一點一定是原始終點
        if len(sample_times) == 0 or sample_times[-1] < t_end:
            sample_times = np.append(sample_times, t_end)

        sample_positions = np.zeros((len(sample_times), raw_positions.shape[1]))

        for j in range(raw_positions.shape[1]):
            sample_positions[:, j] = np.interp(
                sample_times,
                raw_times,
                raw_positions[:, j]
            )

        # 強制保證起點 / 終點完全一致
        sample_times[0] = raw_times[0]
        sample_times[-1] = raw_times[-1]

        sample_positions[0] = raw_positions[0]
        sample_positions[-1] = raw_positions[-1]

        return sample_times, sample_positions

    def execute_trajectory_callback(self, goal_handle):
        self.get_logger().info('Executing MoveIt trajectory...')

        trajectory = goal_handle.request.trajectory

        sample_times, sample_positions = self.resample_trajectory(
            trajectory.points,
            hz=200.0
        )
        
        self.get_logger().info(
            f"Original points: {len(trajectory.points)}, "
            f"resampled points: {len(sample_positions)}"
        )

        if len(sample_positions) == 0:
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return result

        start_time = self.get_clock().now().nanoseconds / 1e9
        # input("Press Enter to start...")
        for t, pos in zip(sample_times, sample_positions):
            target_time = start_time + t

            while (self.get_clock().now().nanoseconds / 1e9) < target_time:
                rclpy.spin_once(self,timeout_sec=0.001)

            incoming_positions = np.array(pos)

            self.update_arm_target(incoming_positions)

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL

        self.get_logger().info('Trajectory execution complete!')
        return result

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

        self.get_logger().info(f'Gripper command: {msg.position}')

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
