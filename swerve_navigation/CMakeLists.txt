cmake_minimum_required(VERSION 3.8)
project(swerve_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)

include_directories(include)

add_executable(waypoint_goal_sender_case1
  src/waypoint_goal_sender_case1.cpp
)

add_executable(waypoint_goal_sender_case2
  src/waypoint_goal_sender_case2.cpp
)

ament_target_dependencies(waypoint_goal_sender_case1
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

ament_target_dependencies(waypoint_goal_sender_case2
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)

install(
  TARGETS
    waypoint_goal_sender_case1
    waypoint_goal_sender_case2
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch params rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()