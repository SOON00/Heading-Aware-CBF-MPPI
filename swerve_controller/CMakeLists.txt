cmake_minimum_required(VERSION 3.8)
project(swerve_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(swerve_drive_controller
  src/swerve_drive_controller.cpp
)

target_include_directories(swerve_drive_controller
  PRIVATE
    include
)

ament_target_dependencies(swerve_drive_controller
  rclcpp
  std_msgs
  geometry_msgs
)

add_executable(swerve_odom_publisher
  src/swerve_odom_publisher.cpp
)

target_include_directories(swerve_odom_publisher
  PRIVATE
    include
)

ament_target_dependencies(swerve_odom_publisher
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS
  swerve_drive_controller
  swerve_odom_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()