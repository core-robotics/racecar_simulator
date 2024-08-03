cmake_minimum_required(VERSION 3.8)
project(racecar_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# include directories
include_directories(include)

# add executables
add_executable(simulator src/simulator.cpp)

# link the executables to the required libraries
ament_target_dependencies(simulator rclcpp std_msgs)

install(TARGETS
  simulator
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/
#   DESTINATION include/
# )

install (DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
