#!/bin/bash
# Install I2RT ROS2 Dependencies for Humble (Docker Optimized)

echo "Installing ROS2 Humble dependencies for I2RT workspace..."

# Update package list
apt update

# Install core ROS2 packages
echo "Installing core ROS2 packages..."
apt install -y \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-rviz2 \
    python3-colcon-common-extensions

# Install the "Backbone" (ros2_control & MoveIt 2)
echo "Installing ros2_control and MoveIt 2 components..."
apt install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-controller-manager

# Install additional useful packages
echo "Installing additional ROS2 packages..."
apt install -y \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools \
    ros-humble-rqt \
    ros-humble-rqt-common-plugins \
    ros-humble-teleop-twist-keyboard

# Optional: Install urdf-tools for validation
echo "Installing URDF tools..."
apt install -y \
    liburdfdom-tools

# Initialize rosdep if it hasn't been done (common in fresh Docker images)
if [ ! -d /etc/ros/rosdep/sources.list.d ]; then
    echo "Initializing rosdep..."
    rosdep init
fi
rosdep update

echo ""
echo "✅ Humble Dependencies installed successfully!"