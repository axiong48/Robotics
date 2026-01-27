
cmake_minimum_required(VERSION 3.8)
project(remap)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(remap src/remap.cpp)
ament_target_dependencies(remap rclcpp geometry_msgs)

install(TARGETS remap
  DESTINATION lib/${PROJECT_NAME})

ament_package()