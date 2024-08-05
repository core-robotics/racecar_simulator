cmake_minimum_required(VERSION 3.8)
project(racecar_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

# include directories
include_directories(include)

# add executables
add_executable(simulator src/simulator.cpp)
add_executable(map_publisher src/map_publisher.cpp)

# link the executables to the required libraries
ament_target_dependencies(simulator rclcpp std_msgs geometry_msgs tf2_ros)
ament_target_dependencies(map_publisher rclcpp nav_msgs geometry_msgs yaml-cpp)

#maually link the libraries
target_link_libraries(map_publisher yaml-cpp)

install(TARGETS
  simulator
  map_publisher
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/
#   DESTINATION include/
# )

install (DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
