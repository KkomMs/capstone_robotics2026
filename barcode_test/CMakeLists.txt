cmake_minimum_required(VERSION 3.10.0)
project(ZebraScanner LANGUAGES C CXX)

# c++ 표준 설정
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Zebra SDK 경로 설정
set(ZEBRA_SDK_ROOT "/home/jinu0211/scanner_sdk/usr")
set(ZEBRA_INCLUDE_DIR "${ZEBRA_SDK_ROOT}/include")
set(ZEBRA_LIB_DIR "${ZEBRA_SDK_ROOT}/lib/zebra-scanner/corescanner")

# 공통 포함 디렉토리
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${ZEBRA_INCLUDE_DIR}
    ${CMAKE_SOURCE_DIR}/src
)
link_directories(${ZEBRA_LIB_DIR})

# 공통 lib 파일
set(COMMON_LIBS
    cs-client
    cs-common
)

# ===================================================
# 1. 바코드 인식 실행 파일
# ===================================================
add_executable(barcode_scanner
    src/main_barcode.cpp
)
target_link_libraries(barcode_scanner PRIVATE
    ${COMMON_LIBS}
)

message(STATUS "1.barcode_scanner.exe")

include(CTest)
enable_testing()
