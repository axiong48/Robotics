cmake_minimum_required(VERSION 3.8)
project(robctl)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(controlturtlebot src/controlturtlebot.cpp)
ament_target_dependencies(controlturtlebot rclcpp geometry_msgs)

add_executable(turtlebotposesensor src/turtlebotposesensor.cpp)
ament_target_dependencies(turtlebotposesensor rclcpp nav_msgs)

install(TARGETS controlturtlebot turtlebotposesensor
  DESTINATION lib/${PROJECT_NAME})

ament_package()