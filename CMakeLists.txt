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
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(f1_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executables and libraries
add_executable(simulator src/simulator.cpp)

add_library(scan_simulator_2d src/scan_simulator_2d.cpp)
add_library(distance_transform src/distance_transform.cpp)
add_library(param_loader src/param_loader.cpp)

# Link libraries
target_link_libraries(simulator scan_simulator_2d param_loader yaml-cpp)
target_link_libraries(scan_simulator_2d distance_transform)


# Link executables to required libraries
ament_target_dependencies(simulator
  rclcpp
  std_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  ackermann_msgs
  f1_msgs
  nav_msgs
  sensor_msgs
  yaml-cpp
)

ament_target_dependencies(param_loader
  rclcpp
)


# Install executables
install(TARGETS
  simulator
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch directory and include files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY params DESTINATION share/${PROJECT_NAME})

ament_package()
