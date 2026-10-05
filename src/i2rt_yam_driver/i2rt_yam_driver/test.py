import os
import sys
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


if __name__ == "__main__":
    robot = get_yam_robot(channel='can1', gripper_type='yam_teaching_handle')
    print(robot.get_joint_pos())