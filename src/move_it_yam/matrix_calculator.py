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
xyz_O_6old = [0, 0, 0]
rpy_O_6old = [0, 0, 0]
T_O_6old = create_homogenous_matrix(xyz_O_6old, rpy_O_6old)

# Joint 7 relative to Joint 6 (Old)
xyz_6old_7 = [-0.014, -0.0463995, 0.0731]
rpy_6old_7 = [-2.40808e-14, 1.46014e-15, -3.15544e-30]
T_6old_7 = create_homogenous_matrix(xyz_6old_7, rpy_6old_7)

# Calculate where Joint 7 was relative to Joint 5
# This is the "World" position we want to keep
T_O_7 = T_O_6old @ T_6old_7

# --- 2. NEW STATE ---
# Joint 6 relative to Joint 5 (New)
xyz_O_6new = [0, 0, 0]
rpy_O_6new = [-1.5708, 0, 3.14159]
T_O_6new = create_homogenous_matrix(xyz_O_6new, rpy_O_6new)

# --- 3. SOLVE FOR NEW JOINT 7 ORIGIN ---
T_6new_7 = np.linalg.inv(T_O_6new) @ T_O_7

# --- 4. EXTRACT XYZ AND RPY ---
new_xyz = T_6new_7[:3, 3]
new_rpy = R.from_matrix(T_6new_7[:3, :3]).as_euler('xyz')

print("--- New URDF Origin for Joint 7 ---")
print(f'xyz="{new_xyz[0]:.7f} {new_xyz[1]:.7f} {new_xyz[2]:.7f}"')
print(f'rpy="{new_rpy[0]:.7f} {new_rpy[1]:.7f} {new_rpy[2]:.7f}"')