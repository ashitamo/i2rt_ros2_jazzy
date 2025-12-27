# i2rt_msgs

Custom ROS2 message, service, and action definitions for I2RT robotics hardware.

## Package Contents

### Messages

| Message | Description |
|---------|-------------|
| `MotorFeedback.msg` | Individual DM motor feedback (position, velocity, torque, temperature, errors) |
| `MotorStatus.msg` | Aggregate status for all motors in a chain |
| `GripperCommand.msg` | Gripper position/force control commands |
| `GripperState.msg` | Current gripper state feedback |
| `TeachingHandleState.msg` | YAM teaching handle trigger and button states |
| `FlowBaseOdometry.msg` | Flow Base odometry with motor diagnostics |

### Services

| Service | Description |
|---------|-------------|
| `SetGravityCompensation.srv` | Enable/disable gravity compensation |
| `CalibrateGripper.srv` | Calibrate linear gripper stroke range |
| `SetMotorTimeout.srv` | Configure motor watchdog timeout |
| `SetZeroPosition.srv` | Set motor zero reference position |

### Actions

| Action | Description |
|--------|-------------|
| `MoveJoint.action` | Execute joint trajectory with feedback |
| `RecordTrajectory.action` | Record demonstration for imitation learning |

## Building

```bash
cd ~/i2rt_ros2_ws
colcon build --packages-select i2rt_msgs
source install/setup.bash
```

## Usage Examples

### Publish Gripper Command

```bash
ros2 topic pub /yam/gripper_command i2rt_msgs/msg/GripperCommand "{
  mode: 0,
  position: 0.5,
  max_force: 10.0,
  duration: 1.0
}"
```

### Call Gravity Compensation Service

```bash
ros2 service call /yam/set_gravity_compensation i2rt_msgs/srv/SetGravityCompensation "{
  enable: true,
  compensation_factor: 1.3
}"
```

### Monitor Motor Feedback

```bash
ros2 topic echo /yam/motor_feedback
```

## Message Definitions

### MotorFeedback

Provides detailed feedback for individual DM motors:
- Position, velocity, torque
- MOS and rotor temperatures
- Error codes with human-readable descriptions

Error codes (bitwise flags):
- `0` - No error
- `1` - Over voltage
- `2` - Under voltage
- `4` - Over current
- `8` - MOS over temperature
- `16` - Rotor over temperature
- `32` - Communication error
- `64` - Encoder error

### GripperCommand

Control modes:
- `POSITION_CONTROL (0)` - Position-based gripper control
- `FORCE_CONTROL (1)` - Force-limited gripping

Position range: `0.0` (fully open) to `1.0` (fully closed)

### TeachingHandleState

YAM teaching handle inputs:
- Trigger position (0.0 to 1.0)
- Two programmable buttons
- Edge detection for button events

## Integration with Standard Messages

This package extends standard ROS2 messages:
- Uses `sensor_msgs/JointState` for joint positions/velocities
- Uses `geometry_msgs/Twist` for Flow Base velocity commands
- Uses `nav_msgs/Odometry` for base odometry
- Custom messages add hardware-specific diagnostics

## License

MIT License
