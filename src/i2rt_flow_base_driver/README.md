# i2rt_flow_base_driver

ROS2 driver for the I2RT Flow Base omnidirectional mobile platform.

## Features

- 200Hz control loop
- Nav2 compatible odometry
- Omnidirectional velocity commands (vx, vy, omega)
- TF broadcasting

## Usage

```bash
# Launch Flow Base
ros2 launch i2rt_flow_base_driver flow_base.launch.py

# Control with keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Reset odometry
ros2 service call /flow_base/reset_odometry std_srvs/srv/Trigger
```

## Topics

- `/flow_base/odom` (nav_msgs/Odometry) - Published odometry
- `/flow_base/cmd_vel` (geometry_msgs/Twist) - Velocity commands

## License

MIT
