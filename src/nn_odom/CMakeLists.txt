cmake_minimum_required(VERSION 3.8)
project(nn_odom)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Torch REQUIRED)

add_executable(nn_aukf src/nn_aukf.cpp)
target_include_directories(nn_aukf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(nn_aukf PUBLIC c_std_99 cxx_std_17)

# ROS2 의존성
ament_target_dependencies(nn_aukf
  rclcpp
  ament_index_cpp
)

# Torch 플래그 & 링크
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")
target_link_libraries(nn_aukf "${TORCH_LIBRARIES}")

# 실행 파일 설치
install(TARGETS nn_aukf
  DESTINATION lib/${PROJECT_NAME}
)

# 헤더 설치
install(DIRECTORY include/
  DESTINATION include/
)

# launch, config, models 설치
install(DIRECTORY
  launch
  config
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
