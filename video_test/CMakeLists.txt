cmake_minimum_required(VERSION 3.10.0)
project(ZebraScanner LANGUAGES C CXX)

# c++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Zebra SDK 경로 설정
set(ZEBRA_SDK_ROOT "/home/jinu0211/scanner_sdk/usr")
set(ZEBRA_INCLUDE_DIR "${ZEBRA_SDK_ROOT}/include")
set(ZEBRA_LIB_DIR "${ZEBRA_SDK_ROOT}/lib/zebra-scanner/corescanner")

# OpenCV 패키지
find_package(OpenCV REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake/opencv4 NO_DEFAULT_PATH)

message(STATUS "========================================")
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "========================================")

# 공통 포함 디렉토리
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${ZEBRA_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/src
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(${ZEBRA_LIB_DIR})

# 공통 lib 파일
set(COMMON_LIBS
    cs-client
    cs-common
)

# ===================================================
# 1. 스캐너 비디오 스트리밍 실행 파일
# ===================================================
add_executable(video_stream
    src/main_video.cpp
)
target_link_libraries(video_stream PRIVATE
    ${COMMON_LIBS}
    ${OpenCV_LIBS}
)

message(STATUS "1.video_stream.exe")

# ===================================================
# 2. ArUco 마커 인식 실행 파일
# ===================================================
add_executable(aruco_detection
    src/main_aruco.cpp
)
target_link_libraries(aruco_detection PRIVATE
    ${COMMON_LIBS}
    ${OpenCV_LIBS}
)
message(STATUS "2.aruco_detection.exe")

# ===================================================
# 3. 카메라 캘리브레이션 실행 파일
# ===================================================
add_executable(camera_calibration
    src/camera_calibration.cpp
)
target_link_libraries(camera_calibration PRIVATE
    ${COMMON_LIBS}
    ${OpenCV_LIBS}
)
message(STATUS "3.camera_calibration.exe")

include(CTest)
enable_testing()
