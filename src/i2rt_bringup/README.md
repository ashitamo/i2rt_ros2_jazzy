# i2rt_bringup

System launch files for I2RT robots.

## Launch Files

### yam_bringup.launch.py
Complete YAM arm system with hardware interface and visualization.

```bash
ros2 launch i2rt_bringup yam_bringup.launch.py
ros2 launch i2rt_bringup yam_bringup.launch.py can_channel:=can1 gripper_type:=linear_3507
```

### bimanual_teleop.launch.py
Leader-follower teleoperation setup.

```bash
ros2 launch i2rt_bringup bimanual_teleop.launch.py
```

### flow_base_bringup.launch.py
Flow Base mobile platform.

```bash
ros2 launch i2rt_bringup flow_base_bringup.launch.py
```

## License

MIT
