cmake_minimum_required(VERSION 3.8)
project(racecar_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

set(map_server_executable map_server)
set(library_name ${map_server_executable}_core)

add_library(${library_name} SHARED
  src/map_server.cpp)
target_include_directories(${library_name}
  PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${library_name} PUBLIC
  ${nav_msgs_TARGETS}
  ${nav2_msgs_TARGETS}
  rclcpp::rclcpp
)

# include directories
include_directories(include)

# add executables
add_executable(simulator src/simulator.cpp)
add_executable(${map_server_executable}
  src/map_server.cpp)
target_include_directories(${map_server_executable}
  PRIVATE
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${map_server_executable} PRIVATE
  ${library_name}
  rclcpp::rclcpp
)

rclcpp_components_register_nodes(${library_name} "map_server::MapServer")

# link the executables to the required libraries
ament_target_dependencies(simulator rclcpp std_msgs)

install(TARGETS
  simulator
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install (DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
