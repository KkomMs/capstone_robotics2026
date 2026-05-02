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
# [추가] 정렬/스캐너/다이나믹셀 노드 의존성
find_package(ros2_aruco_interfaces REQUIRED)
find_package(OpenCV REQUIRED)
find_package(scanner_interfaces REQUIRED)

# ====================================================
# [추가] Zebra SDK 경로
# ====================================================
set(ZEBRA_SDK_ROOT "/home/hmh/scanner_sdk/usr")
set(ZEBRA_INCLUDE_DIR "${ZEBRA_SDK_ROOT}/include")
set(ZEBRA_LIB_DIR "${ZEBRA_SDK_ROOT}/lib/zebra-scanner/corescanner")
set(ZEBRA_LIBS cs-client cs-common)

# ====================================================
# [추가] Dynamixel SDK 경로
# ====================================================
set(DYNAMIXEL_INCLUDE_DIR "/usr/local/include/dynamixel_sdk")
set(DYNAMIXEL_LIB_DIR     "/usr/local/lib")
set(DYNAMIXEL_LIB         "dxl_x64_cpp")

# ====================================================
# 경로
# ====================================================
set(INCLUDE ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(CONFIG ${CMAKE_CURRENT_SOURCE_DIR}/config)
include_directories(
  ${INCLUDE}
  ${ZEBRA_INCLUDE_DIR}        # [추가]
  ${OpenCV_INCLUDE_DIRS}      # [추가]
  ${DYNAMIXEL_INCLUDE_DIR}    # [추가]
)
link_directories(${ZEBRA_LIB_DIR})  # [추가]

# ====================================================
# core library
# ====================================================
add_library(fwis_robot_core
  src/controller.cpp
  src/kinematics.cpp
  src/imu_driver.cpp
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
# [Node] imu_node
# ====================================================
add_executable(imu_node
  src/imu_node.cpp
)
target_include_directories(imu_node PUBLIC
  $<BUILD_INTERFACE:${INCLUDE}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(imu_node
  fwis_robot_core
)
ament_target_dependencies(imu_node
  rclcpp
  sensor_msgs
  tf2
)

# ====================================================
# [추가] [Node] aruco_aligner_node
# ====================================================
add_executable(aruco_aligner_node
  src/aruco_aligner_node.cpp
)
target_include_directories(aruco_aligner_node PUBLIC
  $<BUILD_INTERFACE:${INCLUDE}>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aruco_aligner_node
  fwis_robot_core
)
ament_target_dependencies(aruco_aligner_node
  rclcpp
  std_msgs
  ros2_aruco_interfaces
)

# ====================================================
# [추가] [Node] zebra_scanner_node
# ====================================================
add_executable(zebra_scanner_node
  src/zebra_scanner_node.cpp
)
target_include_directories(zebra_scanner_node PUBLIC
  $<BUILD_INTERFACE:${INCLUDE}>
  $<INSTALL_INTERFACE:include>
  ${ZEBRA_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(zebra_scanner_node
  ${ZEBRA_LIBS}
  ${OpenCV_LIBS}
)
ament_target_dependencies(zebra_scanner_node
  rclcpp
  std_msgs
  geometry_msgs
  ros2_aruco_interfaces
  scanner_interfaces
)

# ====================================================
# [추가] [Node] dynamixel_scan_node
# ====================================================
add_executable(dynamixel_scan_node
  src/dynamixel_scan_node.cpp
)
target_include_directories(dynamixel_scan_node PUBLIC
  $<BUILD_INTERFACE:${INCLUDE}>
  $<INSTALL_INTERFACE:include>
  ${DYNAMIXEL_INCLUDE_DIR}
)
target_link_libraries(dynamixel_scan_node
  ${DYNAMIXEL_LIB}
)
ament_target_dependencies(dynamixel_scan_node
  rclcpp
  std_msgs
  scanner_interfaces
)

# ====================================================
# install - C++
# ====================================================
install(TARGETS
  fwis_robot_core
  mobile_robot_node
  imu_node
  aruco_aligner_node    # [추가]
  zebra_scanner_node    # [추가]
  dynamixel_scan_node   # [추가]
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