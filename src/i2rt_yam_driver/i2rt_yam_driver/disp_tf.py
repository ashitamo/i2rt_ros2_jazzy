from sensor_msgs.msg import Image,PointCloud2,CameraInfo
from i2rt_msgs.msg import GripperState
from i2rt_msgs.msg import GripperCommand
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformException,TransformListener
from realsense2_camera_msgs.msg import Extrinsics
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from cv_bridge import CvBridge
import queue
import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R
import math
import threading
from rclpy.executors import MultiThreadedExecutor
from pathlib import Path
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from pymoveit2 import MoveIt2, MoveIt2State
from pymoveit2.robots import yam_bimanual as robot
from scipy.spatial.transform import Rotation as R
from itertools import product
from visualization_msgs.msg import Marker, MarkerArray

class DispTFNode(Node):
    def __init__(self,yam_namespace='yam_1'):
        super().__init__(f'{yam_namespace}_disp_tf_node',)

        prefix = "left_" if "left" in yam_namespace else "right_"
        self.prefix = prefix
        
        self.yam_namespace = yam_namespace

        self.callback_group = ReentrantCallbackGroup()
        
        self.marker_pub = self.create_publisher(Marker, f'/{self.yam_namespace}/ee_pose_marker', 10)
        
        self.tf_timer = self.create_timer(0.05, self.disp_tf, callback_group=self.callback_group)
        
            
        self.tf_buffer = Buffer() 
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            # spin_thread=True
        )
        
    def disp_tf(self):
        """Continuously fetches the current TF and publishes it as a 3D text marker in RViz."""
        base_frame_name = robot.base_link_name(side=self.prefix)
        end_effector_name = robot.end_effector_name(side=self.prefix)
        
        self.get_logger().debug(f"Attempting to lookup TF from {base_frame_name} to {end_effector_name}")

        try:
            # Look up the latest transform available (non-blocking)
            tf = self.tf_buffer.lookup_transform(
                base_frame_name, 
                end_effector_name, 
                rclpy.time.Time()
            )
        except TransformException:
            # Quietly pass if TF isn't ready yet to avoid spamming warnings
            return

        t = tf.transform.translation
        q = tf.transform.rotation

        # Convert quaternion to euler angles
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)

        # Build the RViz text marker
        marker = Marker()
        marker.header.frame_id = base_frame_name
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = f"{self.yam_namespace}_pose"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        # Position text floating 15cm above the actual end-effector frame
        marker.pose.position.x = t.x
        marker.pose.position.y = t.y
        marker.pose.position.z = t.z + 0.15 
        marker.pose.orientation.w = 1.0 

        # Style preferences
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05  # Text size in meters
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0    # Fully opaque

        # Dynamic text content
        marker.text = (
            f"[{self.prefix.upper()[:-1]}]\n"
            f"X: {t.x:.3f}  Y: {t.y:.3f}  Z: {t.z:.3f}\n"
            f"Rx: {rot[0]:.2f} Ry: {rot[1]:.2f} Rz: {rot[2]:.2f}"
        )

        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    arm1 = DispTFNode('yam_left')
    arm2 = DispTFNode('yam_right')
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(arm1)
    executor.add_node(arm2)
    executor.spin()
    rclpy.shutdown()
if __name__ == '__main__':
    main()