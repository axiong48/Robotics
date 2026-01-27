cmake_minimum_required(VERSION 3.8)
project(moverobo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add your executable
add_executable(moverobo src/moverobo.cpp)

# Link dependencies
ament_target_dependencies(moverobo
  rclcpp
  rclcpp_action
  std_msgs
  nav_msgs
  nav2_msgs
  geometry_msgs
)

add_executable(human_detector_node src/human_detector_node.cpp)

ament_target_dependencies(human_detector_node
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
)


# Install the executable
install(TARGETS
  moverobo
  human_detector_node
  DESTINATION lib/${PROJECT_NAME}
)


# Install launch files (if you have them)
# install(DIRECTORY
#   launch
#   DESTINATION share/${PROJECT_NAME}
# )



ament_package()