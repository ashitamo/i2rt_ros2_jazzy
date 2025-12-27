# I2RT ROS2 Jazzy - Complete Build Guide

Complete step-by-step guide to build and run the I2RT ROS2 Jazzy workspace.

## Prerequisites

### 1. System Requirements

- **OS**: Ubuntu 24.04 (Noble Numbat)
- **ROS2**: Jazzy Jalisco
- **Python**: 3.11+
- **Hardware**: CAN bus interface (CANable, PEAK, etc.)

### 2. Install ROS2 Jazzy

```bash
# Setup sources
sudo apt update && sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy Desktop
sudo apt update
sudo apt upgrade
sudo apt install ros-jazzy-desktop

# Install additional packages
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-joint-state-publisher-gui
sudo apt install ros-jazzy-xacro
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

### 3. Install I2RT Python API

```bash
cd ~/Documents/neotix_robotics/i2rt

# Create virtual environment
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env
uv venv --python 3.11
source .venv/bin/activate

# Install dependencies
sudo apt install build-essential python3-dev linux-headers-$(uname -r)
uv pip install -e .
```

### 4. Setup CAN Bus

```bash
# Bring up CAN interface
sudo ip link set can0 up type can bitrate 1000000

# OR use the i2rt convenience script
cd ~/Documents/neotix_robotics/i2rt
sh scripts/reset_all_can.sh

# For auto-startup on boot
sudo sh devices/install_devices.sh
```

## Building the ROS2 Workspace

### 1. Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### 2. Build Workspace

```bash
cd ~/Documents/neotix_robotics/i2rt_ros2_ws

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 3. Verify Build

```bash
# List installed packages
ros2 pkg list | grep i2rt

# Expected output:
# i2rt_bringup
# i2rt_description
# i2rt_examples
# i2rt_flow_base_driver
# i2rt_msgs
# i2rt_teleop
# i2rt_yam_driver
```

## Quick Start Examples

### Example 1: Visualize YAM Robot in RViz

```bash
# Terminal 1: Launch visualization
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
source install/setup.bash
ros2 launch i2rt_description view_yam.launch.py

# Use the GUI sliders to move the robot
```

### Example 2: Run YAM Hardware Interface

```bash
# Ensure CAN is up
sudo ip link set can0 up type can bitrate 1000000

# Launch hardware interface
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
source install/setup.bash
ros2 launch i2rt_yam_driver yam_hardware.launch.py can_channel:=can0 gripper_type:=crank_4310
```

### Example 3: Complete YAM System (Hardware + Visualization)

```bash
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
source install/setup.bash
ros2 launch i2rt_bringup yam_bringup.launch.py
```

### Example 4: Bimanual Teleoperation

```bash
# Ensure both CAN channels are up
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

# Launch bimanual system
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
source install/setup.bash
ros2 launch i2rt_bringup bimanual_teleop.launch.py leader_channel:=can1 follower_channel:=can0

# In another terminal, run teleoperation node
ros2 run i2rt_teleop leader_follower
```

### Example 5: Flow Base Mobile Platform

```bash
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
source install/setup.bash
ros2 launch i2rt_flow_base_driver flow_base.launch.py

# Control with keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Testing Individual Packages

### Test Message Definitions

```bash
# List custom messages
ros2 interface list | grep i2rt_msgs

# Show message definition
ros2 interface show i2rt_msgs/msg/MotorFeedback
ros2 interface show i2rt_msgs/srv/SetGravityCompensation
```

### Test URDF

```bash
# Process xacro to URDF
xacro $(ros2 pkg prefix i2rt_description)/share/i2rt_description/urdf/yam_crank_4310.urdf.xacro > /tmp/yam.urdf

# Check URDF validity
check_urdf /tmp/yam.urdf

# Visualize URDF
urdf_to_graphviz /tmp/yam.urdf
```

### Test Topics

```bash
# List topics
ros2 topic list

# Echo joint states
ros2 topic echo /yam/joint_states

# Monitor frequency
ros2 topic hz /yam/joint_states
```

### Test Services

```bash
# List services
ros2 service list | grep yam

# Call gravity compensation service
ros2 service call /yam/set_gravity_compensation i2rt_msgs/srv/SetGravityCompensation "{enable: true, compensation_factor: 1.3}"
```

## Common Issues & Solutions

### Issue 1: CAN Bus Not Found

```bash
# Check CAN devices
ls -l /sys/class/net/can*

# If not found, check USB connection
dmesg | grep can

# Unplug and replug CAN adapter
sudo ip link set can0 down
# Unplug/replug USB
sudo ip link set can0 up type can bitrate 1000000
```

### Issue 2: Motor Timeout Errors

YAM motors have a 400ms watchdog by default. Disable if needed:

```bash
cd ~/Documents/neotix_robotics/i2rt
source .venv/bin/activate
python i2rt/motor_config_tool/set_timeout.py --channel can0
```

### Issue 3: Import Error for i2rt API

Ensure the Python path is correctly set in the nodes. Edit if needed:

```python
# In yam_hardware_interface.py
sys.path.insert(0, os.path.expanduser('~/Documents/neotix_robotics/i2rt'))
```

Or set environment variable:

```bash
export PYTHONPATH=$PYTHONPATH:~/Documents/neotix_robotics/i2rt
```

### Issue 4: Permission Denied on CAN

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in for changes to take effect
```

### Issue 5: Build Errors

```bash
# Clean and rebuild
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
rm -rf build install log
colcon build --symlink-install

# If specific package fails, build it separately
colcon build --packages-select i2rt_msgs
```

## Development Workflow

### Rebuild After Changes

```bash
cd ~/Documents/neotix_robotics/i2rt_ros2_ws

# Rebuild specific package
colcon build --packages-select i2rt_yam_driver

# Rebuild with symlink install (faster for Python changes)
colcon build --symlink-install --packages-select i2rt_yam_driver

# Source updated workspace
source install/setup.bash
```

### Run Tests

```bash
# Run all tests
colcon test

# Run tests for specific package
colcon test --packages-select i2rt_msgs

# View test results
colcon test-result --all
```

## Environment Setup Script

Create a convenience script for sourcing:

```bash
# Create setup script
cat > ~/i2rt_ros2_setup.sh << 'EOF'
#!/bin/bash
# I2RT ROS2 Environment Setup

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
source ~/Documents/neotix_robotics/i2rt_ros2_ws/install/setup.bash

# Add i2rt Python API to path
export PYTHONPATH=$PYTHONPATH:~/Documents/neotix_robotics/i2rt

# Activate i2rt virtual environment
source ~/Documents/neotix_robotics/i2rt/.venv/bin/activate

echo "I2RT ROS2 environment ready!"
EOF

chmod +x ~/i2rt_ros2_setup.sh

# Use it
source ~/i2rt_ros2_setup.sh
```

## Next Steps

1. **Explore Examples**: See [i2rt_examples](src/i2rt_examples/README.md)
2. **Read Package Documentation**: Each package has a detailed README
3. **Customize Parameters**: Edit YAML files in `config/` directories
4. **Develop Custom Nodes**: Use the existing nodes as templates

## Support

- **I2RT Hardware/API**: support@i2rt.com
- **ROS2 Integration**: [GitHub Issues](https://github.com/your-repo/issues)
- **ROS2 Documentation**: https://docs.ros.org/en/jazzy/

## License

MIT License - See LICENSE file for details.
