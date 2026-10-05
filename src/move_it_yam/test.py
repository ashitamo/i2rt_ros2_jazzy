import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import numpy as np
from scipy.spatial.transform import Rotation as R


class GripperParentSubscriber(Node):
    def __init__(self):
        super().__init__('gripper_parent_listener')
        
        # Subscribe to the /tf topic
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)

    def tf_callback(self, msg):
        # msg.transforms is an array of TransformStamped
        for transform in msg.transforms:
            # Check if the PARENT frame is 'gripper'
            if transform.header.frame_id == 'gripper':
                
                child = transform.child_frame_id
                pos = transform.transform.translation
                rot = transform.transform.rotation
                
                euler = R.from_quat([rot.x, rot.y, rot.z, rot.w]).as_euler('xyz', degrees=False)
                self.current_positions = np.concatenate((np.array([pos.x, pos.y, pos.z]), euler))

                self.get_logger().info(f'current positions: {self.current_positions}')

def main(args=None):
    rclpy.init(args=args)
    node = GripperParentSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
main()