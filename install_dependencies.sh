#!/bin/bash
# Install I2RT ROS2 Dependencies

echo "Installing ROS2 Jazzy dependencies for I2RT workspace..."

# Update package list
sudo apt update

# Install core ROS2 packages
echo "Installing core ROS2 packages..."
sudo apt install -y \
    ros-jazzy-xacro \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-rviz2 \
    python3-colcon-common-extensions

# Install additional useful packages
echo "Installing additional ROS2 packages..."
sudo apt install -y \
    ros-jazzy-tf2-ros \
    ros-jazzy-tf2-tools \
    ros-jazzy-rqt \
    ros-jazzy-rqt-common-plugins \
    ros-jazzy-teleop-twist-keyboard

# Optional: Install urdf-tools for validation
echo "Installing URDF tools..."
sudo apt install -y \
    liburdfdom-tools

echo ""
echo "✅ Dependencies installed successfully!"
echo ""
echo "Next steps:"
echo "1. Build the workspace: colcon build --symlink-install"
echo "2. Source the workspace: source install/setup.bash"
echo "3. Test visualization: ros2 launch i2rt_description view_yam.launch.py"
