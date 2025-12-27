# I2RT ROS2 Workspace - Verification Checklist

Use this checklist to verify your installation and build.

## ✅ Pre-Build Verification

### System Requirements
- [ ] Ubuntu 24.04 (Noble Numbat) installed
- [ ] ROS2 Jazzy installed (`ros2 --version` works)
- [ ] Python 3.11+ available (`python3 --version`)
- [ ] CAN hardware connected (CANable, PEAK, etc.)

### I2RT Python API
- [ ] i2rt repository cloned to `~/Documents/neotix_robotics/i2rt`
- [ ] i2rt virtual environment created (`.venv` directory exists)
- [ ] i2rt dependencies installed (`uv pip install -e .` completed)
- [ ] CAN bus up (`ip link show can0` shows UP state)

## ✅ Build Verification

### Workspace Structure
```bash
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
ls src/
```
- [ ] i2rt_bringup directory exists
- [ ] i2rt_description directory exists
- [ ] i2rt_examples directory exists
- [ ] i2rt_flow_base_driver directory exists
- [ ] i2rt_msgs directory exists
- [ ] i2rt_teleop directory exists
- [ ] i2rt_yam_driver directory exists

### Build Success
```bash
cd ~/Documents/neotix_robotics/i2rt_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```
- [ ] Build completes without errors
- [ ] `install/` directory created
- [ ] `build/` directory created
- [ ] No "package not found" errors

### Package Listing
```bash
source install/setup.bash
ros2 pkg list | grep i2rt
```
Expected output (all 7 packages):
- [ ] i2rt_bringup
- [ ] i2rt_description
- [ ] i2rt_examples
- [ ] i2rt_flow_base_driver
- [ ] i2rt_msgs
- [ ] i2rt_teleop
- [ ] i2rt_yam_driver

## ✅ Message/Service Verification

### Custom Interfaces
```bash
ros2 interface list | grep i2rt
```
- [ ] i2rt_msgs/msg/MotorFeedback
- [ ] i2rt_msgs/msg/GripperCommand
- [ ] i2rt_msgs/msg/GripperState
- [ ] i2rt_msgs/srv/SetGravityCompensation
- [ ] i2rt_msgs/srv/CalibrateGripper
- [ ] i2rt_msgs/action/MoveJoint

### Show Message Definition
```bash
ros2 interface show i2rt_msgs/msg/MotorFeedback
```
- [ ] Message definition displays correctly

## ✅ Description Package Verification

### URDF Processing
```bash
xacro $(ros2 pkg prefix i2rt_description)/share/i2rt_description/urdf/yam_crank_4310.urdf.xacro > /tmp/yam.urdf
check_urdf /tmp/yam.urdf
```
- [ ] xacro processes without errors
- [ ] check_urdf reports "robot name is: yam"
- [ ] No errors about missing links/joints

### Mesh Files
```bash
ls $(ros2 pkg prefix i2rt_description)/share/i2rt_description/meshes/yam/
```
- [ ] 14 STL files present (base_link through link_6, collision and visual)

## ✅ Visualization Test (No Hardware Required)

```bash
ros2 launch i2rt_description view_yam.launch.py
```
- [ ] RViz launches successfully
- [ ] Robot model visible in RViz
- [ ] Joint state publisher GUI appears
- [ ] Moving sliders moves the robot in RViz
- [ ] No red links (missing meshes)

## ✅ Hardware Tests (Requires YAM Arm)

### CAN Bus
```bash
sudo ip link set can0 up type can bitrate 1000000
ip link show can0
```
- [ ] CAN interface shows UP
- [ ] Bitrate is 1000000

### YAM Hardware Interface
```bash
ros2 launch i2rt_yam_driver yam_hardware.launch.py
```
- [ ] Node launches without errors
- [ ] "YAM robot initialized successfully" message appears
- [ ] "YAM hardware interface ready @ 250 Hz" message appears

### Joint States Publishing
```bash
# In another terminal
ros2 topic hz /yam/joint_states
```
- [ ] Topic publishes at ~250 Hz
- [ ] No errors about missing CAN device

### Echo Joint States
```bash
ros2 topic echo /yam/joint_states --once
```
- [ ] Position array has 7 elements (6 arm + 1 gripper)
- [ ] Velocity array has 7 elements
- [ ] Values are reasonable (not NaN or inf)

## ✅ Flow Base Tests (Requires Flow Base Hardware)

### Flow Base Node
```bash
ros2 launch i2rt_flow_base_driver flow_base.launch.py
```
- [ ] Node launches successfully
- [ ] "Flow Base initialized successfully" appears

### Odometry Publishing
```bash
ros2 topic hz /flow_base/odom
```
- [ ] Topic publishes at ~200 Hz

### Velocity Command
```bash
ros2 topic pub --once /flow_base/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```
- [ ] Command accepted without errors
- [ ] Base moves (if hardware present)

## ✅ Service Tests

### List Services
```bash
ros2 service list | grep yam
```
- [ ] /yam/set_gravity_compensation exists
- [ ] /yam/calibrate_gripper exists
- [ ] /yam/emergency_stop exists

### Call Service
```bash
ros2 service call /yam/set_gravity_compensation i2rt_msgs/srv/SetGravityCompensation "{enable: true, compensation_factor: 1.3}"
```
- [ ] Service call succeeds
- [ ] Response shows success: true

## ✅ Launch File Tests

### Complete YAM Bringup
```bash
ros2 launch i2rt_bringup yam_bringup.launch.py use_rviz:=true
```
- [ ] Hardware interface launches
- [ ] Robot state publisher launches
- [ ] RViz launches
- [ ] Robot visible and moving

### Bimanual Launch
```bash
ros2 launch i2rt_bringup bimanual_teleop.launch.py
```
- [ ] Leader arm node launches
- [ ] Follower arm node launches
- [ ] Both publish to separate namespaces

## ✅ Documentation Verification

### READMEs Present
- [ ] Workspace README.md exists
- [ ] BUILD_GUIDE.md exists
- [ ] QUICKSTART.md exists
- [ ] PROJECT_SUMMARY.md exists
- [ ] Each package has a README.md

### Documentation Quality
- [ ] Build instructions are clear
- [ ] Examples are provided
- [ ] Troubleshooting section exists
- [ ] Contact information provided

## 🎯 Final Checklist

- [ ] All packages build successfully
- [ ] Visualization works without hardware
- [ ] Hardware interface launches (with CAN bus)
- [ ] Topics publish at expected rates
- [ ] Services respond correctly
- [ ] Launch files work as expected
- [ ] Documentation is complete

## 📝 Notes

Use this space to record any issues or observations:

```
Date: _____________
Tested by: _____________

Issues found:
-
-
-

Resolved:
-
-
-
```

## ✅ Sign-Off

- [ ] **Basic Verification Complete** - All packages build and messages work
- [ ] **Visualization Verified** - Robot displays correctly in RViz
- [ ] **Hardware Verified** - Arm/base control works correctly
- [ ] **Integration Verified** - Multi-robot and launch files work
- [ ] **Production Ready** - All tests pass, documentation complete

---

**Verification Status**: _________________ (Pass/Fail/Partial)
**Date**: _________________
**Verified By**: _________________
