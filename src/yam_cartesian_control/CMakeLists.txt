cmake_minimum_required(VERSION 3.8)
project(yam_cartesian_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(
  twist_to_joint_jog
  src/twist_to_joint_jog.cpp
)

ament_target_dependencies(
  twist_to_joint_jog
  rclcpp
  sensor_msgs
  geometry_msgs
  control_msgs
  diagnostic_msgs
  moveit_core
  moveit_ros_planning
)

target_link_libraries(
  twist_to_joint_jog
  Eigen3::Eigen
)

install(
  TARGETS
  twist_to_joint_jog
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(tcp_pose_controller
  src/tcp_pose_controller.cpp
)

ament_target_dependencies(tcp_pose_controller
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
)

target_link_libraries(tcp_pose_controller
  Eigen3::Eigen
)

install(TARGETS
  tcp_pose_controller
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
