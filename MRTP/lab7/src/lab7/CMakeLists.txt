cmake_minimum_required(VERSION 3.8)
project(lab7)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(navigation REQUIRED)

add_executable(testnavigator src/testnavigator.cpp)
ament_target_dependencies(testnavigator
  rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs navigation)

install(TARGETS testnavigator DESTINATION lib/${PROJECT_NAME})
ament_package()