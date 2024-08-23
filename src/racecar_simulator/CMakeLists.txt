cmake_minimum_required(VERSION 3.8)
project(racecar_simulator)

# Compiler-specific options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(control_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executables and libraries
add_executable(simulator src/simulator.cpp)
add_executable(map_publisher src/map_publisher.cpp)

add_library(scan_simulator_2d src/scan_simulator_2d.cpp)
add_library(distance_transform src/distance_transform.cpp)

# Link libraries
target_link_libraries(simulator scan_simulator_2d)
target_link_libraries(scan_simulator_2d distance_transform)
target_link_libraries(map_publisher yaml-cpp)

# Link executables to required libraries
ament_target_dependencies(simulator
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  ackermann_msgs
  control_msgs
  nav_msgs
  sensor_msgs
)

ament_target_dependencies(map_publisher
  rclcpp
  nav_msgs
  geometry_msgs
  yaml-cpp
)

# Install executables
install(TARGETS
  simulator
  map_publisher
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory and include files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)

ament_package()
