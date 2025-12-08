cmake_minimum_required(VERSION 3.8)
project(nn_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(vesc_msgs REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(Torch REQUIRED)

# -------------------------
# nn_aukf node
# -------------------------
add_executable(nn_aukf
  src/nn_aukf.cpp
)

target_include_directories(nn_aukf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(nn_aukf PUBLIC cxx_std_17)

ament_target_dependencies(nn_aukf
  rclcpp
  ament_index_cpp
  nav_msgs
  sensor_msgs
  ackermann_msgs
  vesc_msgs
)

target_compile_options(nn_aukf PRIVATE ${TORCH_CXX_FLAGS})
target_link_libraries(nn_aukf "${TORCH_LIBRARIES}")

# -------------------------
# mocap_pose_to_odom node
# -------------------------
add_executable(mocap_pose_to_odom
  src/mocap_pose_to_odom.cpp
)

target_include_directories(mocap_pose_to_odom PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(mocap_pose_to_odom PUBLIC cxx_std_17)

ament_target_dependencies(mocap_pose_to_odom
  rclcpp
  geometry_msgs
  nav_msgs
)

target_link_libraries(mocap_pose_to_odom Eigen3::Eigen)

# -------------------------
# Install
# -------------------------
install(TARGETS
  nn_aukf
  mocap_pose_to_odom
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
