# i2rt_yam_driver

ROS2 driver node for the I2RT YAM 6-DOF robotic arm.

## Overview

This package provides a ROS2 interface to the YAM robotic arm by wrapping the [i2rt Python API](https://github.com/i2rt-robotics/i2rt). It maintains the high-performance 250Hz control loop while exposing standard ROS2 topics, services, and actions.

## Features

- **High-frequency control**: 250Hz joint control loop
- **Joint state publishing**: Real-time position/velocity feedback
- **Gripper control**: Position and force control modes
- **Motor diagnostics**: Temperature, torque, error monitoring
- **Gravity compensation**: Enable/disable via service
- **TF broadcasting**: Full kinematic chain transforms
- **Emergency stop**: Safety service

## Prerequisites

1. **I2RT Python API** must be installed:
```bash
cd ~/Documents/neotix_robotics/i2rt
source .venv/bin/activate
uv pip install -e .
```

2. **CAN bus configured**:
```bash
sudo ip link set can0 up type can bitrate 1000000
```

3. **ROS2 Jazzy installed**

## Building

```bash
cd ~/i2rt_ros2_ws
colcon build --packages-select i2rt_yam_driver
source install/setup.bash
```

## Usage

### Launch Hardware Interface

```bash
# Basic launch with default parameters (can0, crank_4310 gripper)
ros2 launch i2rt_yam_driver yam_hardware.launch.py

# Specify CAN channel and gripper type
ros2 launch i2rt_yam_driver yam_hardware.launch.py can_channel:=can1 gripper_type:=linear_3507

# Custom robot name (for multi-arm setups)
ros2 launch i2rt_yam_driver yam_hardware.launch.py robot_name:=yam_left
```

### Topics

#### Published Topics

| Topic | Type | Frequency | Description |
|-------|------|-----------|-------------|
| `/yam/joint_states` | sensor_msgs/JointState | 250 Hz | Joint positions, velocities |
| `/yam/motor_feedback` | i2rt_msgs/MotorStatus | 10 Hz | Motor diagnostics |
| `/yam/gripper_state` | i2rt_msgs/GripperState | 250 Hz | Gripper position, force |
| `/tf` | tf2_msgs/TFMessage | 250 Hz | Transform tree |

#### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/yam/joint_command` | trajectory_msgs/JointTrajectory | Joint position commands |
| `/yam/gripper_command` | i2rt_msgs/GripperCommand | Gripper position/force commands |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/yam/set_gravity_compensation` | i2rt_msgs/SetGravityCompensation | Enable/disable gravity comp |
| `/yam/calibrate_gripper` | i2rt_msgs/CalibrateGripper | Calibrate linear gripper |
| `/yam/emergency_stop` | std_srvs/Trigger | Emergency stop |

### Example: Command Joints

```bash
# Publish joint trajectory command
ros2 topic pub --once /yam/joint_command trajectory_msgs/msg/JointTrajectory "{
  joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
  points: [{
    positions: [0.0, 0.5, 0.5, 0.0, 0.0, 0.0],
    time_from_start: {sec: 2}
  }]
}"
```

### Example: Control Gripper

```bash
# Close gripper
ros2 topic pub --once /yam/gripper_command i2rt_msgs/msg/GripperCommand "{
  mode: 0,
  position: 1.0,
  max_force: 10.0,
  duration: 1.0
}"

# Open gripper
ros2 topic pub --once /yam/gripper_command i2rt_msgs/msg/GripperCommand "{
  mode: 0,
  position: 0.0,
  duration: 1.0
}"
```

### Example: Enable Gravity Compensation

```bash
ros2 service call /yam/set_gravity_compensation i2rt_msgs/srv/SetGravityCompensation "{
  enable: true,
  compensation_factor: 1.3
}"
```

### Example: Emergency Stop

```bash
ros2 service call /yam/emergency_stop std_srvs/srv/Trigger
```

## Configuration

Edit [config/yam_params.yaml](config/yam_params.yaml) to customize:

```yaml
yam_hardware_interface:
  ros__parameters:
    can_channel: "can0"
    gripper_type: "crank_4310"
    control_frequency: 250.0
    gravity_comp_enabled: true
    gravity_comp_factor: 1.3
    zero_gravity_mode: false
    robot_name: "yam"
    publish_tf: true
```

## Python API Usage

```python
import rclpy
from i2rt_yam_driver.yam_controller import YAMController

rclpy.init()
controller = YAMController()

# Command joints
controller.command_joints([0.0, 0.5, 0.5, 0.0, 0.0, 0.0], duration=2.0)

# Control gripper
controller.command_gripper(position=0.8, force_limit=10.0)

rclpy.spin(controller)
```

## Gripper Types

| Gripper Type | Description | Calibration Required |
|--------------|-------------|---------------------|
| `crank_4310` | Zero-linkage crank gripper | No |
| `linear_3507` | Lightweight linear gripper (DM3507 motor) | Yes |
| `linear_4310` | Standard linear gripper (DM4310 motor) | Yes |
| `yam_teaching_handle` | Leader arm teaching handle | No |
| `no_gripper` | Arm only (no gripper attached) | No |

### Calibrating Linear Grippers

```bash
ros2 service call /yam/calibrate_gripper i2rt_msgs/srv/CalibrateGripper
```

## Multi-Arm Setup

For bimanual systems with leader and follower arms:

```bash
# Terminal 1: Leader arm (can1)
ros2 launch i2rt_yam_driver yam_hardware.launch.py \
  can_channel:=can1 \
  gripper_type:=yam_teaching_handle \
  robot_name:=yam_leader

# Terminal 2: Follower arm (can0)
ros2 launch i2rt_yam_driver yam_hardware.launch.py \
  can_channel:=can0 \
  gripper_type:=crank_4310 \
  robot_name:=yam_follower
```

## Troubleshooting

### CAN Bus Not Found

```bash
# Check CAN interface
ls -l /sys/class/net/can*

# Bring up CAN
sudo ip link set can0 up type can bitrate 1000000
```

### Motor Timeout Errors

YAM motors have a default 400ms watchdog. Disable if needed:

```bash
# From i2rt directory
python i2rt/motor_config_tool/set_timeout.py --channel can0
```

### Import Error for i2rt API

Ensure the i2rt Python path is correct in the node:

```python
sys.path.insert(0, os.path.expanduser('~/Documents/neotix_robotics/i2rt'))
```

Or set `PYTHONPATH`:

```bash
export PYTHONPATH=$PYTHONPATH:~/Documents/neotix_robotics/i2rt
```

## Architecture

```
┌─────────────────────────────────────────┐
│   YAM Hardware Interface Node (250Hz)   │
│                                          │
│  ┌────────────────────────────────────┐ │
│  │  Control Loop (Timer @ 250Hz)      │ │
│  │  - Read joint state                │ │
│  │  - Command target position         │ │
│  │  - Publish joint_states            │ │
│  │  - Publish TF                      │ │
│  └────────────────────────────────────┘ │
│                                          │
│  ┌────────────────────────────────────┐ │
│  │  i2rt Python API Wrapper           │ │
│  │  - get_yam_robot()                 │ │
│  │  - get_joint_pos()                 │ │
│  │  - command_joint_pos()             │ │
│  └────────────────────────────────────┘ │
└─────────────────┬────────────────────────┘
                  │
                  ▼
          ┌───────────────┐
          │   CAN Bus     │
          │   (250Hz)     │
          └───────┬───────┘
                  │
                  ▼
          ┌───────────────┐
          │  DM Motors    │
          │  (1-7)        │
          └───────────────┘
```

## License

MIT License
