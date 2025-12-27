cmake_minimum_required(VERSION 3.8)
project(i2rt_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/MotorFeedback.msg"
  "msg/MotorStatus.msg"
  "msg/GripperCommand.msg"
  "msg/GripperState.msg"
  "msg/TeachingHandleState.msg"
  "msg/FlowBaseOdometry.msg"

  # Services
  "srv/SetGravityCompensation.srv"
  "srv/CalibrateGripper.srv"
  "srv/SetMotorTimeout.srv"
  "srv/SetZeroPosition.srv"

  # Actions
  "action/MoveJoint.action"
  "action/RecordTrajectory.action"

  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
