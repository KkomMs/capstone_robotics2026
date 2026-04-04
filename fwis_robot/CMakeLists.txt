cmake_minimum_required(VERSION 3.8)
project(fwis_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ====================================================
# 의존 package
# ====================================================
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ====================================================
# 경로
# ====================================================
set(INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/config)
include_directories(${INCLUDE})

# ====================================================
# core library
# ====================================================
add_library(fwis_robot_core
  #src/camera.cpp
  src/controller.cpp
  src/kinematics.cpp
)
target_include_directories(fwis_robot_core PUBLIC
  $<BUILD_INTERFACE:${INCLUDE}>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(fwis_robot_core rclcpp)

# ====================================================
# [Node] mobile_robot_node
# ====================================================
add_executable(mobile_robot_node
  src/mobile_robot_node.cpp
)
target_include_directories(mobile_robot_node PUBLIC
  $<BUILD_INTERFACE:${INCLUDE}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mobile_robot_node
  fwis_robot_core
)
ament_target_dependencies(mobile_robot_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
)

# ====================================================
# [Node] camera_node
# ====================================================
# add_executable(camera_node
#   src/camera_node.cpp
# )
# target_include_directories(camera_node PUBLIC
#   $<BUILD_INTERFACE:${INCLUDE}>
#   $<INSTALL_INTERFACE:include>  
# )
# target_link_libraries(camera_node
#   fwis_robot_core
# )
# # [todo] 실제 필요한 의존성으로 수정하기
# ament_target_dependencies(camera_node
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   sensor_msgs
#   std_msgs
#   tf2
#   tf2_ros
# )

# ====================================================
# install - C++
# ====================================================
install(TARGETS
  fwis_robot_core
  mobile_robot_node
  #camera_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)

# ====================================================
# install - Python
# ====================================================
install(PROGRAMS
  scripts/stm32_bridge.py
  RENAME stm32_bridge
  DESTINATION lib/${PROJECT_NAME}
)

# ====================================================
# install - 리소스
# ====================================================
install(DIRECTORY
  config
  launch
  models
  scripts
  src
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()