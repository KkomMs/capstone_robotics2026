cmake_minimum_required(VERSION 3.8)
project(fwis_simulation)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY
  launch
  scripts
  config
  models
  urdf
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

## Install
install(
  PROGRAMS
    scripts/cmd_vel_controller.py
    scripts/odom_publisher.py
    scripts/lyapunov_kinematics_controller.py
    scripts/kinematics_smc_controller.py
    scripts/aruco_aligner.py
    scripts/mpc_ufrws_controller.py
    scripts/scanner_motor_test.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()