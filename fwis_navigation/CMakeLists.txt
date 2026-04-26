cmake_minimum_required(VERSION 3.8)
project(fwis_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===================================================
# 플러그인 필수 의존성
#====================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(NLOPT REQUIRED nlopt)

# ===================================================
# include 디렉토리 설정
#====================================================
include_directories(
  include
  ${NLOPT_INCLUDE_DIRS}
)

# ===================================================
# 플래너, 컨트롤러 이름 설정
#====================================================
set(UFRWS_CONTROLLER_PLUGIN_NAME
  mpc_ufrws_controller)

# ===================================================
# 의존성 설정
#====================================================
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  std_msgs
  visualization_msgs
  nav2_util
  nav2_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
  tf2
  tf2_ros
  nav2_costmap_2d
  nav2_core
  pluginlib
)

# ===================================================
# 플래너, 컨트롤러 소스코드를 공유 라이브러리(.so)로 빌드
#==================================================== 
add_library(${UFRWS_CONTROLLER_PLUGIN_NAME} SHARED
  src/mpc_ufrws_controller.cpp)
target_link_libraries(${UFRWS_CONTROLLER_PLUGIN_NAME}
  ${NLOPT_LIBRARIES}
)

# prevent pluginlib from using boost
target_compile_definitions(${UFRWS_CONTROLLER_PLUGIN_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")
# ===================================================
# 플러그인 의존성 링크
#====================================================
ament_target_dependencies(${UFRWS_CONTROLLER_PLUGIN_NAME}
  ${dependencies}
)

# ===================================================
# 플러그인 xml 파일 시스템에 등록
#====================================================
pluginlib_export_plugin_description_file(nav2_core mpc_ufrws_controller.xml)

# ===================================================
# 플러그인 설치
#====================================================
install(TARGETS ${UFRWS_CONTROLLER_PLUGIN_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# ===================================================
# 패키지 설치
#====================================================
install(
  DIRECTORY launch map param rviz src
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${UFRWS_CONTROLLER_PLUGIN_NAME})
ament_export_dependencies(${dependencies})
ament_package()