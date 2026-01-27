cmake_minimum_required(VERSION 3.8)
project(tracking)


add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(trackdifferences src/trackdifferences.cpp)

ament_target_dependencies(trackdifferences
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS trackdifferences
  DESTINATION lib/${PROJECT_NAME})

ament_package()