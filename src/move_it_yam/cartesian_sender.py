import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
from std_srvs.srv import Trigger
import threading

# Added OrientationConstraint here!
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive

class CartesianSenderNode(Node):
    def __init__(self):
        super().__init__('cartesian_sender_node')
        self.action_client = ActionClient(self, MoveGroup, 'move_action')
        self.subscription = self.create_subscription(Pose, 'coor_quat', self.cartesian_callback, 10)
        self.execute_service = self.create_service(Trigger, 'execute_trajectory', self.execute_callback)
        
        self.last_goal = None
        self.goal_handle = None
        self.plan_ready = False
        
        self.get_logger().info('Node started. Waiting for Pose messages on /coor_quat...')
        self.get_logger().info('Press ENTER to execute the trajectory after planning is complete.')
        
        self.input_thread = threading.Thread(target=self.input_listener, daemon=True)
        self.input_thread.start()

    def input_listener(self):
        """Listen for Enter key to execute trajectory"""
        while rclpy.ok():
            try:
                input() 
                if self.plan_ready:
                    self.get_logger().info("Executing trajectory...")
                    self.execute_async()
                else:
                    self.get_logger().warn("No trajectory planned yet. Send a target first.")
            except EOFError:
                break
            except Exception as e:
                self.get_logger().error(f"Input error: {str(e)}")

    def execute_async(self):
        """Execute the last planned trajectory"""
        if self.last_goal is None:
            self.get_logger().warn("No trajectory has been planned yet.")
            return
            
        exec_goal = MoveGroup.Goal()
        exec_goal.request = self.last_goal.request
        exec_goal.planning_options.plan_only = False # NOW execute!
        self.action_client.send_goal_async(exec_goal).add_done_callback(self.execution_response_callback)
        self.plan_ready = False 

    def cartesian_callback(self, msg):
        self.get_logger().info(f'Received Cartesian coordinates: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}')
        self.send_moveit(msg)

    def send_moveit(self, msg):
        goal = MoveGroup.Goal()

        # Tell MoveIt to plan from the robot's CURRENT hardware state, not 0,0,0
        goal.request.start_state.is_diff = True

        goal.request.group_name = "yam_arm"
        goal.request.num_planning_attempts = 50       # Increased from 10
        goal.request.allowed_planning_time = 10.0      # Increased from 5.0
        target_constraints = Constraints()

        # --- 1. Position Constraint ---
        pos_con = PositionConstraint()
        pos_con.header.frame_id = "world"
        pos_con.link_name = "gripper"
        
        s = SolidPrimitive()
        s.type = SolidPrimitive.BOX
        s.dimensions = [0.0001, 0.0001, 0.0001]
        
        bv = BoundingVolume()
        bv.primitives.append(s)
        bv.primitive_poses.append(msg)
        pos_con.constraint_region = bv
        pos_con.weight = 1.0
        target_constraints.position_constraints.append(pos_con)

        # --- 2. Orientation Constraint (THIS WAS MISSING) ---
        ori_con = OrientationConstraint()
        ori_con.header.frame_id = "world"
        ori_con.link_name = "gripper"
        ori_con.orientation = msg.orientation # Use the quaternion from the message
        
        # 0.05 rad is about 3 degrees of tolerance. Adjust if it fails to plan.
        ori_con.absolute_x_axis_tolerance = 0.001
        ori_con.absolute_y_axis_tolerance = 0.001
        ori_con.absolute_z_axis_tolerance = 0.001
        ori_con.weight = 1.0
        target_constraints.orientation_constraints.append(ori_con)

        # 3. PLAN ONLY
        goal.request.goal_constraints.append(target_constraints)
        goal.planning_options.plan_only = True

        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available!")
            return

        self.get_logger().info("Planning trajectory...")
        self.last_goal = goal
        self.action_client.send_goal_async(goal).add_done_callback(self.plan_response_callback)

    def plan_response_callback(self, future):
        try:
            self.goal_handle = future.result()
            self.plan_ready = True
            self.get_logger().info("✓ Trajectory planned successfully!")
            self.get_logger().info("Press ENTER to execute, or send another target to replan.")
        except Exception as e:
            self.get_logger().error(f"Planning failed: {str(e)}")

    def execute_callback(self, request, response):
        if self.last_goal is None:
            response.success = False
            response.message = "No trajectory planned yet."
            self.get_logger().warn(response.message)
            return response
            
        exec_goal = MoveGroup.Goal()
        exec_goal.request = self.last_goal.request
        exec_goal.planning_options.plan_only = False 
        self.get_logger().info("Executing trajectory...")
        self.action_client.send_goal_async(exec_goal).add_done_callback(self.execution_response_callback)
        
        response.success = True
        response.message = "Execution started!"
        return response

    def execution_response_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info("✓ Trajectory execution completed!")
        except Exception as e:
            self.get_logger().error(f"Execution failed: {str(e)}")

def main():
    rclpy.init()
    node = CartesianSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()