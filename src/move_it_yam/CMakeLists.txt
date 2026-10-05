cmake_minimum_required(VERSION 3.22)
project(move_it_yam)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/pose_to_twist.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pose_to_twist
)

install(
  PROGRAMS scripts/play_pose_csv.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME play_pose_csv
)

install(
  PROGRAMS scripts/move_to_joint_pose.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME move_to_joint_pose
)
