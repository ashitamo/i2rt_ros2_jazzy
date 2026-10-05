import numpy as np
from scipy.spatial.transform import Rotation as R

def create_homogenous_matrix(xyz, rpy):
    mat = np.eye(4)
    # URDF uses static 'xyz' Euler order
    mat[:3, :3] = R.from_euler('xyz', rpy).as_matrix()
    mat[:3, 3] = xyz
    return mat

# --- 1. OLD STATE (The "Goal" configuration) ---
# Joint 6 relative to Joint 5 (Old)
xyz_5_6_old = [2.39858e-07, -0.0419481, 0.0404996]
rpy_5_6_old = [-1.5708, -1.5708, 0]
T5_6_old = create_homogenous_matrix(xyz_5_6_old, rpy_5_6_old)

# Joint 7 relative to Joint 6 (Old)
xyz_6_7_old = [-3.57066e-05, 0.000249371, -0.0293133]
rpy_6_7_old = [0 ,0 ,0]
T6_7_old = create_homogenous_matrix(xyz_6_7_old, rpy_6_7_old)

# Calculate where Joint 7 was relative to Joint 5
# This is the "World" position we want to keep
T5_7_goal = T5_6_old @ T6_7_old

# --- 2. NEW STATE ---
# Joint 6 relative to Joint 5 (New)
xyz_5_6_new = [2.39858e-07, -0.0419481, 0.0404996]
rpy_5_6_new = [-1.5708, 0, 3.14159]
T5_6_new = create_homogenous_matrix(xyz_5_6_new, rpy_5_6_new)

# --- 3. SOLVE FOR NEW JOINT 7 ORIGIN ---
# T5_6_new @ T6_7_new = T5_7_goal
T6_7_new = np.linalg.inv(T5_6_new) @ T5_7_goal

# --- 4. EXTRACT XYZ AND RPY ---
new_xyz = T6_7_new[:3, 3]
new_rpy = R.from_matrix(T6_7_new[:3, :3]).as_euler('xyz')

print("--- New URDF Origin for Joint 7 ---")
print(f'xyz="{new_xyz[0]:.7f} {new_xyz[1]:.7f} {new_xyz[2]:.7f}"')
print(f'rpy="{new_rpy[0]:.7f} {new_rpy[1]:.7f} {new_rpy[2]:.7f}"')