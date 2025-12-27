# i2rt_description

URDF robot descriptions, meshes, and visualization for I2RT hardware.

## Package Contents

### URDF Models

| File | Description |
|------|-------------|
| `yam_macro.xacro` | Xacro macro for YAM 6-DOF arm |
| `yam_crank_4310.urdf.xacro` | YAM with crank gripper (zero-linkage) |
| `yam_linear_3507.urdf.xacro` | YAM with lightweight linear gripper |
| `yam_linear_4310.urdf.xacro` | YAM with standard linear gripper |
| `gripper_crank_4310.xacro` | Crank gripper macro |

### Meshes

STL mesh files for visualization and collision geometry:
- `meshes/yam/` - YAM arm link meshes (base through link_6)

### Configuration

- `config/joint_limits.yaml` - Joint limits, velocities, and effort limits

### Launch Files

- `launch/view_yam.launch.py` - Visualize YAM robot in RViz with joint state publisher GUI

### RViz Configurations

- `rviz/view_robot.rviz` - Default RViz configuration for robot visualization

## Usage

### Visualize YAM Robot

```bash
# Build the package
cd ~/i2rt_ros2_ws
colcon build --packages-select i2rt_description
source install/setup.bash

# Launch visualization with default gripper (crank_4310)
ros2 launch i2rt_description view_yam.launch.py

# Launch with specific gripper type
ros2 launch i2rt_description view_yam.launch.py gripper_type:=linear_3507
```

### Check URDF Validity

```bash
# Process xacro to URDF
xacro $(ros2 pkg prefix i2rt_description)/share/i2rt_description/urdf/yam_crank_4310.urdf.xacro > /tmp/yam.urdf

# Check URDF
check_urdf /tmp/yam.urdf
```

### View TF Tree

```bash
# While robot is running, view TF tree
ros2 run rqt_tf_tree rqt_tf_tree
```

## Coordinate Frames

The YAM robot uses the following TF tree structure:

```
world (fixed base)
  └─ base_link
      └─ link_1
          └─ link_2
              └─ link_3
                  └─ link_4
                      └─ link_5
                          └─ link_6
                              ├─ tcp (tool center point)
                              │   └─ gripper_base
                              │       ├─ left_finger
                              │       └─ right_finger
                              └─ grasp_point (offset by 134.7mm)
```

### Key Frames

- **world**: Fixed world frame (ground plane)
- **base_link**: Robot base mounting point
- **link_1 through link_6**: Arm joint frames following DH convention
- **tcp**: Tool Center Point (wrist flange center)
- **grasp_point**: Recommended grasping point (tip of gripper fingers)

## Joint Information

### YAM Arm Joints

| Joint | Range (rad) | Range (deg) | Motor Type | Max Effort (Nm) |
|-------|-------------|-------------|------------|-----------------|
| joint1 | -2.618 to 3.13 | -150° to 179° | DM4340 | 10.0 |
| joint2 | 0.0 to 3.65 | 0° to 209° | DM4340 | 10.0 |
| joint3 | 0.0 to 3.13 | 0° to 179° | DM4340 | 10.0 |
| joint4 | -1.65 to 1.65 | -95° to 95° | DM4310 | 10.0 |
| joint5 | -1.571 to 1.571 | -90° to 90° | DM4310 | 10.0 |
| joint6 | -2.094 to 2.094 | -120° to 120° | DM4310 | 10.0 |

### Gripper Joint

| Joint | Type | Range | Description |
|-------|------|-------|-------------|
| gripper_joint | Prismatic | 0.0 - 0.04 m | Parallel jaw gripper (40mm stroke) |

## Inertial Properties

All links include mass and inertia properties derived from the original MuJoCo models. These are used for:
- Gravity compensation calculations
- Dynamics simulation
- Motion planning with MoveIt2

## Using with MoveIt2

This description package is compatible with MoveIt2. Generate a MoveIt configuration:

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```

Load the URDF file from this package and follow the setup wizard.

## Customization

### Adding Custom Grippers

1. Create a new xacro macro in `urdf/gripper_custom.xacro`
2. Create a new complete URDF in `urdf/yam_custom.urdf.xacro` that includes your gripper
3. Add mesh files to `meshes/custom/` if needed

### Modifying Joint Limits

Edit `config/joint_limits.yaml` to adjust:
- Position limits
- Velocity limits
- Acceleration limits
- Effort limits

## License

MIT License
