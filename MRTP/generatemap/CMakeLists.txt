cmake_minimum_required(VERSION 3.8)
project(generatemap)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ROS 2 Dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# find_package(ament_index_cpp REQUIRED)

# find_package(amiga_navigation_interfaces REQUIRED)
# find_package(kortex_interfaces REQUIRED)
# find_package(amiga_interfaces REQUIRED)

# System Libraries
find_package(nlohmann_json REQUIRED)
# find_package(LibXml2 REQUIRED)


# NOTE: This assumes 'tcp_demux_node.cpp' is in the root of the folder.
# If you move it to a 'src' folder, change this to "src/tcp_demux_node.cpp"
add_executable(tcp_demux_node src/tcp_demux_node.cpp)

# Include directories for LibXml2
target_include_directories(tcp_demux_node PUBLIC ${LIBXML2_INCLUDE_DIR})


# Link ROS 2 dependencies
ament_target_dependencies(tcp_demux_node
  rclcpp
  rclcpp_action
  nav2_msgs
  geometry_msgs
  geographic_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
#  amiga_navigation_interfaces
#  kortex_interfaces
#  amiga_interfaces
)

# Link System dependencies
target_link_libraries(tcp_demux_node
  nlohmann_json::nlohmann_json
  ${LIBXML2_LIBRARIES}
)

install(TARGETS tcp_demux_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()