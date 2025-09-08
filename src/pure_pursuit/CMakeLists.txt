cmake_minimum_required(VERSION 3.8)
project(pure_pursuit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(pure_pursuit src/pure_pursuit.cpp)

ament_target_dependencies(pure_pursuit
  rclcpp
  control_msgs
  ackermann_msgs
  nav_msgs
  tf2
)

install(TARGETS
  pure_pursuit
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
