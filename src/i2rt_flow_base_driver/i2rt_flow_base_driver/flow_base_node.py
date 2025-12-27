#!/usr/bin/env python3
"""
ROS2 Node for I2RT Flow Base Omnidirectional Mobile Platform

Provides ROS2 interface for the Flow Base including:
- Odometry publishing (200Hz)
- Velocity command subscription
- TF broadcasting
- Nav2 compatibility

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

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from std_srvs.srv import Trigger

from tf2_ros import TransformBroadcaster
import tf_transformations

# Add i2rt Python API to path
sys.path.insert(0, os.path.expanduser('~/Documents/neotix_robotics/i2rt'))

try:
    from i2rt.flow_base.flow_base_controller import Vehicle
except ImportError as e:
    print(f"ERROR: Failed to import i2rt Python API: {e}")
    print("Please ensure i2rt is installed and in your Python path")
    sys.exit(1)


class FlowBaseNode(Node):
    """ROS2 node for Flow Base mobile platform"""

    def __init__(self):
        super().__init__('flow_base_node')

        # Declare parameters
        self.declare_parameter('control_frequency', 200.0)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_linear_velocity', 0.5)  # m/s
        self.declare_parameter('max_angular_velocity', 1.0)  # rad/s

        # Get parameters
        self.control_freq = self.get_parameter('control_frequency').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value

        self.get_logger().info('Initializing Flow Base')

        # Initialize vehicle
        try:
            self.vehicle = Vehicle()
            self.vehicle.start_control()
            self.get_logger().info('Flow Base initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize Flow Base: {e}')
            raise

        # Command lock
        self.cmd_lock = Lock()
        self.current_cmd = (0.0, 0.0, 0.0)  # (vx, vy, omega)

        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.odom_pub = self.create_publisher(
            Odometry, '/flow_base/odom', sensor_qos)

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/flow_base/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Services
        self.reset_odom_srv = self.create_service(
            Trigger,
            '/flow_base/reset_odometry',
            self.reset_odometry_callback
        )

        # TF broadcaster
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        # Control loop timer (200Hz)
        self.control_timer = self.create_timer(
            1.0 / self.control_freq,
            self.control_loop_callback
        )

        self.get_logger().info(f'Flow Base node ready @ {self.control_freq} Hz')

    def control_loop_callback(self):
        """Main control loop at 200Hz"""
        try:
            # Get current velocity command
            with self.cmd_lock:
                vx, vy, omega = self.current_cmd

            # Command vehicle
            self.vehicle.set_target_velocity((vx, vy, omega), frame="local")

            # Get odometry (if available)
            try:
                odom_data = self.vehicle.get_odometry()
                self.publish_odometry(odom_data)
            except AttributeError:
                # Odometry method may not exist in all versions
                pass

        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}', throttle_duration_sec=1.0)

    def publish_odometry(self, odom_data):
        """Publish odometry message"""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame

        # Position (x, y, theta from odometry data)
        # Note: Actual structure depends on i2rt API implementation
        # This is a placeholder - adjust based on actual API
        x, y, theta = odom_data.get('position', (0.0, 0.0, 0.0))

        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = 0.0

        # Orientation (quaternion from theta)
        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        msg.pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        # Velocity
        vx, vy, omega = odom_data.get('velocity', (0.0, 0.0, 0.0))
        msg.twist.twist.linear.x = float(vx)
        msg.twist.twist.linear.y = float(vy)
        msg.twist.twist.angular.z = float(omega)

        # Covariance (placeholder - should be tuned)
        msg.pose.covariance = [0.1] * 36
        msg.twist.covariance = [0.1] * 36

        self.odom_pub.publish(msg)

        # Publish TF
        if self.publish_tf:
            self.publish_transform(x, y, theta)

    def publish_transform(self, x, y, theta):
        """Publish TF transform from odom to base_link"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        self.tf_broadcaster.sendTransform(t)

    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        # Clamp velocities to limits
        vx = np.clip(msg.linear.x, -self.max_linear_vel, self.max_linear_vel)
        vy = np.clip(msg.linear.y, -self.max_linear_vel, self.max_linear_vel)
        omega = np.clip(msg.angular.z, -self.max_angular_vel, self.max_angular_vel)

        with self.cmd_lock:
            self.current_cmd = (vx, vy, omega)

        self.get_logger().debug(f'cmd_vel: vx={vx:.2f}, vy={vy:.2f}, omega={omega:.2f}')

    def reset_odometry_callback(self, request, response):
        """Reset odometry to origin"""
        try:
            self.vehicle.reset_odometry()
            response.success = True
            response.message = 'Odometry reset to origin'
            self.get_logger().info('Odometry reset')
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f'Odometry reset failed: {e}')

        return response

    def destroy_node(self):
        """Cleanup on shutdown"""
        self.get_logger().info('Shutting down Flow Base node')
        # Stop the vehicle
        with self.cmd_lock:
            self.current_cmd = (0.0, 0.0, 0.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = FlowBaseNode()
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
