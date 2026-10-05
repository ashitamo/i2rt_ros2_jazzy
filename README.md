# I2RT ROS2 Jazzy Workspace

ROS2 Jazzy integration for I2RT robotics products (YAM arm, ARX-R5, Flow Base).

# Warning
目前這個readme是沒有被更新的請不要參考它

## Overview

This workspace provides ROS2 interfaces for the [I2RT Python API](https://github.com/i2rt-robotics/i2rt), enabling seamless integration with the ROS2 ecosystem while maintaining high-performance real-time control.

## Packages

| Package | Description |
|---------|-------------|
| `i2rt_msgs` | Custom ROS2 message and service definitions |
| `i2rt_description` | URDF models, meshes, and robot descriptions |
| `i2rt_yam_driver` | ROS2 driver node for YAM 6-DOF robotic arm |
| `i2rt_flow_base_driver` | ROS2 driver node for Flow Base mobile platform |
| `i2rt_bringup` | Launch files and system configuration |
| `i2rt_teleop` | Teleoperation nodes (leader-follower, joystick) |
| `i2rt_examples` | Example launch files and demonstrations |

## System Requirements

- **OS**: Ubuntu 24.04 (Noble Numbat)
- **ROS2**: Jazzy Jalisco
- **Python**: 3.11+
- **Hardware**: CAN bus interface (CANable, PEAK, etc.)

## Installation

### 1. Install ROS2 Jazzy

```bash
sudo apt update
sudo apt install ros-jazzy-desktop
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
sudo apt install ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher
sudo apt install ros-jazzy-xacro
```

### 2. Install I2RT Python API

```bash
cd /path/to/i2rt
uv venv --python 3.11
source .venv/bin/activate
sudo apt install build-essential python3-dev linux-headers-$(uname -r)
uv pip install -e .
```

### 3. Build ROS2 Workspace

```bash
cd ~/i2rt_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Quick Start

### Launch YAM Arm

```bash
# Terminal 1: Bring up YAM arm on can0
ros2 launch i2rt_bringup yam_bringup.launch.py can_channel:=can0 gripper_type:=crank_4310

# Terminal 2: Visualize in RViz
ros2 launch i2rt_bringup yam_rviz.launch.py
```

### Launch Flow Base

```bash
ros2 launch i2rt_bringup flow_base_bringup.launch.py
```

### Bimanual Teleoperation

```bash
ros2 launch i2rt_bringup bimanual_teleop.launch.py leader_channel:=can1 follower_channel:=can0
```

## Hardware Setup

### CAN Bus Configuration

```bash
# Bring up CAN interface
sudo ip link set can0 up type can bitrate 1000000

# Or use the provided script
sh /path/to/i2rt/scripts/reset_all_can.sh
```

### Persistent CAN Device Names

For systems with multiple CAN interfaces, set persistent device names. See [i2rt documentation](../i2rt/doc/set_persist_id_socket_can.md).

## Architecture

```
┌─────────────────────────────────────────┐
│         ROS2 Application Layer          │
│  (MoveIt2, Nav2, Custom Controllers)    │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│       ROS2 Driver Nodes                 │
│  (i2rt_yam_driver, i2rt_flow_base)      │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│       I2RT Python API (unchanged)       │
│  (MotorChainRobot, FlowBase)            │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────▼───────────────────────┐
│       CAN Bus Communication             │
│  (DM Motor Drivers @ 250Hz)             │
└─────────────────────────────────────────┘
```

## Control Frequencies

- **YAM Arm**: 250 Hz (4ms cycle time)
- **Flow Base**: 200 Hz (5ms cycle time)
- **Joint State Publishing**: Same as control frequency
- **TF Broadcasting**: Same as control frequency

## Topics

### YAM Arm

| Topic | Type | Description |
|-------|------|-------------|
| `/yam/joint_states` | sensor_msgs/JointState | Joint positions, velocities, efforts |
| `/yam/joint_command` | trajectory_msgs/JointTrajectory | Joint position commands |
| `/yam/gripper_command` | i2rt_msgs/GripperCommand | Gripper control |
| `/yam/motor_feedback` | i2rt_msgs/MotorFeedback | Motor temperatures, torques |

### Flow Base

| Topic | Type | Description |
|-------|------|-------------|
| `/flow_base/odom` | nav_msgs/Odometry | Base odometry |
| `/flow_base/cmd_vel` | geometry_msgs/Twist | Velocity commands |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/yam/set_gravity_compensation` | i2rt_msgs/SetGravityCompensation | Enable/disable gravity comp |
| `/yam/calibrate_gripper` | i2rt_msgs/CalibrateGripper | Calibrate linear gripper |
| `/flow_base/reset_odometry` | std_srvs/Trigger | Reset odometry to origin |

## Examples

See the [i2rt_examples](src/i2rt_examples/README.md) package for detailed demonstrations.

## Troubleshooting

### CAN Bus Not Found
```bash
# Check CAN devices
ls -l /sys/class/net/can*

# If not found, check USB connection and drivers
dmesg | grep can
```

### Motor Timeout Errors
YAM motors have a 400ms watchdog by default. Disable if needed:
```bash
python i2rt/motor_config_tool/set_timeout.py --channel can0
```

### Permission Denied on CAN
```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER
# Log out and back in
```

## Contributing

Contributions welcome! Please:
1. Fork the repository
2. Create a feature branch
3. Follow ROS2 coding standards
4. Submit a pull request

## License

MIT License - See LICENSE file for details.

## Support

- **I2RT API**: support@i2rt.com
- **ROS2 Integration**: [GitHub Issues](https://github.com/your-repo/issues)

## Acknowledgments

- [I2RT](https://i2rt.com/) - Hardware and Python API
- [ROS2 Jazzy](https://docs.ros.org/en/jazzy/) - Robot Operating System
