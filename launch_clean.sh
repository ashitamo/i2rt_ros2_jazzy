#!/bin/bash
# Launch ROS2 in a clean environment without snap interference

# Function to clean environment
clean_env() {
    # Remove all snap-related paths from LD_LIBRARY_PATH
    if [ -n "$LD_LIBRARY_PATH" ]; then
        export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:*$//')
    fi

    # Clean PATH from snap
    if [ -n "$PATH" ]; then
        export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:*$//')
    fi

    # Unset all snap environment variables
    unset SNAP
    unset SNAP_INSTANCE_NAME
    unset SNAP_NAME
    unset SNAP_REVISION
    unset SNAP_USER_COMMON
    unset SNAP_USER_DATA
    unset SNAP_COMMON
    unset SNAP_DATA
    unset SNAP_LIBRARY_PATH
    unset SNAP_CONTEXT
}

# Clean the environment
clean_env

# Source ROS2
source /opt/ros/jazzy/setup.bash

# Source workspace
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
source install/setup.bash

# Launch with all arguments passed to this script
exec "$@"
