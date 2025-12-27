#!/bin/bash
# Fix snap library conflicts with ROS2

echo "Fixing snap library conflicts..."

# Remove snap paths from LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$(echo $LD_LIBRARY_PATH | tr ':' '\n' | grep -v snap | tr '\n' ':' | sed 's/:$//')

# Unset SNAP environment variables that might interfere
unset SNAP
unset SNAP_INSTANCE_NAME
unset SNAP_NAME

echo "✅ Snap paths removed from LD_LIBRARY_PATH"
echo ""
echo "Current LD_LIBRARY_PATH:"
echo "$LD_LIBRARY_PATH"
echo ""
echo "Now you can run ROS2 commands in this terminal session."
echo "Run: source /opt/ros/jazzy/setup.bash"
echo "Then: source install/setup.bash"
echo "Then: ros2 launch i2rt_description view_yam.launch.py"
