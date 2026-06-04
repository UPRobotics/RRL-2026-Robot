cmake_minimum_required(VERSION 3.8)
project(osmo_camera_pkg)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------
# ROS2
# -------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# -------------------------------------------------------
# System deps via pkg-config
# -------------------------------------------------------
find_package(PkgConfig REQUIRED)

pkg_check_modules(SDL2          REQUIRED sdl2)
pkg_check_modules(LIBAVCODEC    REQUIRED libavcodec)
pkg_check_modules(LIBAVFORMAT   REQUIRED libavformat)
pkg_check_modules(LIBAVDEVICE   REQUIRED libavdevice)
pkg_check_modules(LIBSWSCALE    REQUIRED libswscale)
pkg_check_modules(LIBAVUTIL     REQUIRED libavutil)
pkg_check_modules(SPDLOG        REQUIRED spdlog)

find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  pkg_check_modules(NLOHMANN_JSON REQUIRED nlohmann_json)
endif()

# -------------------------------------------------------
# Optional NVJPEG (part of CUDA toolkit)
# -------------------------------------------------------
find_package(CUDAToolkit QUIET)
if(CUDAToolkit_FOUND)
  find_path(NVJPEG_INCLUDE_DIR nvjpeg.h
    HINTS ${CUDAToolkit_INCLUDE_DIRS}
    PATHS /usr/local/cuda/targets/x86_64-linux/include
          /usr/local/cuda/targets/aarch64-linux/include
          /usr/local/cuda/include)
  find_library(NVJPEG_LIBRARY nvjpeg
    HINTS ${CUDAToolkit_LIBRARY_DIR}
    PATHS /usr/local/cuda/targets/x86_64-linux/lib
          /usr/local/cuda/targets/aarch64-linux/lib
          /usr/local/cuda/lib64)
  if(NVJPEG_INCLUDE_DIR AND NVJPEG_LIBRARY)
    set(HAS_NVJPEG TRUE)
    message(STATUS "NVJPEG found: ${NVJPEG_LIBRARY}")
  else()
    message(STATUS "NVJPEG header/library not found — using FFmpeg MJPEG software decoder")
  endif()
else()
  message(STATUS "CUDAToolkit not found — using FFmpeg MJPEG software decoder")
endif()

# ARM / Jetson detection
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(IS_JETSON TRUE)
  message(STATUS "Building for aarch64 (Jetson Orin)")
endif()

# -------------------------------------------------------
# Common include dirs / libs shared by both executables
# -------------------------------------------------------
set(COMMON_INCLUDES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SDL2_INCLUDE_DIRS}
  ${LIBAVCODEC_INCLUDE_DIRS}
  ${LIBAVFORMAT_INCLUDE_DIRS}
  ${LIBAVDEVICE_INCLUDE_DIRS}
  ${LIBSWSCALE_INCLUDE_DIRS}
  ${LIBAVUTIL_INCLUDE_DIRS}
  ${SPDLOG_INCLUDE_DIRS}
)

set(COMMON_LIBS
  ${SDL2_LIBRARIES}
  ${LIBAVCODEC_LIBRARIES}
  ${LIBAVFORMAT_LIBRARIES}
  ${LIBAVDEVICE_LIBRARIES}
  ${LIBSWSCALE_LIBRARIES}
  ${LIBAVUTIL_LIBRARIES}
  ${SPDLOG_LIBRARIES}
  pthread
)

# -------------------------------------------------------
# osmo_stream_node
# -------------------------------------------------------
add_executable(osmo_stream_node
  src/usb_camera.cpp
  src/osmo_stream_node.cpp
)

target_include_directories(osmo_stream_node PUBLIC ${COMMON_INCLUDES})
target_link_libraries(osmo_stream_node ${COMMON_LIBS})

if(nlohmann_json_FOUND)
  target_link_libraries(osmo_stream_node nlohmann_json::nlohmann_json)
else()
  target_include_directories(osmo_stream_node PUBLIC ${NLOHMANN_JSON_INCLUDE_DIRS})
endif()

ament_target_dependencies(osmo_stream_node rclcpp sensor_msgs ament_index_cpp)

# -------------------------------------------------------
# osmo_display_node
# -------------------------------------------------------
add_executable(osmo_display_node
  src/jpeg_decoder.cpp
  src/display_window.cpp
  src/osmo_display_node.cpp
)

target_include_directories(osmo_display_node PUBLIC ${COMMON_INCLUDES})
target_link_libraries(osmo_display_node ${COMMON_LIBS})

if(nlohmann_json_FOUND)
  target_link_libraries(osmo_display_node nlohmann_json::nlohmann_json)
else()
  target_include_directories(osmo_display_node PUBLIC ${NLOHMANN_JSON_INCLUDE_DIRS})
endif()

ament_target_dependencies(osmo_display_node rclcpp sensor_msgs ament_index_cpp)

if(HAS_NVJPEG)
  target_compile_definitions(osmo_display_node PRIVATE HAS_NVJPEG=1)
  target_include_directories(osmo_display_node PRIVATE ${NVJPEG_INCLUDE_DIR})
  target_link_libraries(osmo_display_node ${NVJPEG_LIBRARY} CUDA::cudart)
  if(IS_JETSON)
    target_compile_definitions(osmo_display_node PRIVATE IS_JETSON=1)
  endif()
endif()

# -------------------------------------------------------
# Install
# -------------------------------------------------------
install(TARGETS osmo_stream_node osmo_display_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# -------------------------------------------------------
# Testing
# -------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
