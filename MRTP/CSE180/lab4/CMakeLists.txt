cmake_minimum_required(VERSION 3.8)
project(lab4)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(chaser src/chaser.cpp)
ament_target_dependencies(chaser rclcpp geometry_msgs turtlesim)

install(TARGETS chaser
  DESTINATION lib/${PROJECT_NAME})

ament_package()