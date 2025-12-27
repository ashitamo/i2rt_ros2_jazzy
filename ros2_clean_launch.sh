#!/bin/bash
# ROS2 Clean Launch - bypasses snap library conflicts
# Usage: ./ros2_clean_launch.sh <ros2 command>
# Example: ./ros2_clean_launch.sh launch i2rt_description view_yam.launch.py

# Preload system pthread to override snap's version
export LD_PRELOAD=/lib/x86_64-linux-gnu/libpthread.so.0

# Clean snap paths
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:*$//')
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:*$//')

# Unset snap variables
for var in $(env | grep ^SNAP | cut -d= -f1); do
    unset $var
done

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
source install/setup.bash

# Run ROS2 command
ros2 "$@"
