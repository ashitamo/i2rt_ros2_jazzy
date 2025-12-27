# I2RT ROS2 - Quick Start Guide

Get up and running with I2RT ROS2 in 5 minutes!

## One-Time Setup

```bash
# 1. Install ROS2 Jazzy (if not already installed)
sudo apt update && sudo apt install ros-jazzy-desktop

# 2. Install I2RT Python API
cd ~/Documents/neotix_robotics/i2rt
source .venv/bin/activate
uv pip install -e .

# 3. Setup CAN bus
sudo ip link set can0 up type can bitrate 1000000

# 4. Build ROS2 workspace
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run Your First Example

### Option 1: Visualize Robot (No Hardware Required)

```bash
source ~/Documents/neotix_robotics/i2rt_ros2_ws/install/setup.bash
ros2 launch i2rt_description view_yam.launch.py
```

Move the robot using the GUI sliders!

### Option 2: Run Real Hardware

```bash
# Ensure CAN is up
sudo ip link set can0 up type can bitrate 1000000

# Launch YAM arm
source ~/Documents/neotix_robotics/i2rt_ros2_ws/install/setup.bash
ros2 launch i2rt_bringup yam_bringup.launch.py
```

## Common Commands

```bash
# List topics
ros2 topic list

# Monitor joint states
ros2 topic echo /yam/joint_states

# Control gripper
ros2 topic pub --once /yam/gripper_command i2rt_msgs/msg/GripperCommand "{mode: 0, position: 0.5}"

# Reset odometry (Flow Base)
ros2 service call /flow_base/reset_odometry std_srvs/srv/Trigger
```

## What's Next?

- Read [BUILD_GUIDE.md](BUILD_GUIDE.md) for detailed instructions
- Explore [examples](src/i2rt_examples/README.md)
- Check individual package READMEs for specific features

## Need Help?

- CAN bus issues? See BUILD_GUIDE.md "Common Issues"
- Package documentation: `src/<package_name>/README.md`
- Support: support@i2rt.com
