cmake_minimum_required(VERSION 3.8)
project(nn_odom)

# -------------------------
# Compile options
# -------------------------
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------
# Dependencies
# -------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Message packages
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

# Torch
find_package(Torch REQUIRED)

# -------------------------
# Target
# -------------------------
add_executable(nn_aukf
  src/nn_aukf.cpp 
)

target_include_directories(nn_aukf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(nn_aukf PUBLIC c_std_99 cxx_std_17)

# ROS2 dependencies
ament_target_dependencies(nn_aukf
  rclcpp
  ament_index_cpp
  nav_msgs
  sensor_msgs
  ackermann_msgs
  vesc_msgs
)

# Torch flags & libs
target_compile_options(nn_aukf PRIVATE ${TORCH_CXX_FLAGS})
target_link_libraries(nn_aukf "${TORCH_LIBRARIES}")

# -------------------------
# Install
# -------------------------
install(TARGETS nn_aukf
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
