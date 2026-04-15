cmake_minimum_required(VERSION 3.8)
project(detections_pkg)

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
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# -------------------------------------------------------
# System deps via pkg-config
# -------------------------------------------------------
find_package(PkgConfig REQUIRED)

pkg_check_modules(SDL2     REQUIRED sdl2)
pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)

pkg_check_modules(LIBAVCODEC  REQUIRED libavcodec)
pkg_check_modules(LIBAVFORMAT REQUIRED libavformat)
pkg_check_modules(LIBSWSCALE  REQUIRED libswscale)
pkg_check_modules(LIBAVUTIL   REQUIRED libavutil)

pkg_check_modules(SPDLOG REQUIRED spdlog)

find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  pkg_check_modules(NLOHMANN_JSON REQUIRED nlohmann_json)
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs dnn)
pkg_check_modules(ZBAR REQUIRED zbar)

# ONNX Runtime (for hazmat detection)
find_library(ONNXRUNTIME_LIB onnxruntime PATHS /usr/local/lib)
find_path(ONNXRUNTIME_INCLUDE onnxruntime_cxx_api.h PATHS /usr/local/include)
find_library(ONNXRUNTIME_CUDA_LIB onnxruntime_providers_cuda PATHS /usr/local/lib)
find_library(ONNXRUNTIME_SHARED_LIB onnxruntime_providers_shared PATHS /usr/local/lib)

# -------------------------------------------------------
# Sources
# -------------------------------------------------------
set(SOURCES
  src/main.cpp
  src/detection_stream.cpp
  src/detection_window.cpp
  src/qr_detector.cpp
  src/motion_detector.cpp
  src/magnetometer_panel.cpp
  src/hazmat_detector.cpp
  src/thermal_panel.cpp
)

add_executable(detections_pkg_node ${SOURCES})

target_include_directories(detections_pkg_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SDL2_INCLUDE_DIRS}
  ${SDL2_TTF_INCLUDE_DIRS}
  ${LIBAVCODEC_INCLUDE_DIRS}
  ${LIBAVFORMAT_INCLUDE_DIRS}
  ${LIBSWSCALE_INCLUDE_DIRS}
  ${LIBAVUTIL_INCLUDE_DIRS}
  ${SPDLOG_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${ZBAR_INCLUDE_DIRS}
  ${ONNXRUNTIME_INCLUDE}
)

if(nlohmann_json_FOUND)
  target_link_libraries(detections_pkg_node nlohmann_json::nlohmann_json)
else()
  target_include_directories(detections_pkg_node PUBLIC ${NLOHMANN_JSON_INCLUDE_DIRS})
endif()

target_link_libraries(detections_pkg_node
  ${SDL2_LIBRARIES}
  ${SDL2_TTF_LIBRARIES}
  ${LIBAVCODEC_LIBRARIES}
  ${LIBAVFORMAT_LIBRARIES}
  ${LIBSWSCALE_LIBRARIES}
  ${LIBAVUTIL_LIBRARIES}
  ${SPDLOG_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${ZBAR_LIBRARIES}
  ${ONNXRUNTIME_LIB}
  pthread
)

# Link CUDA EP if available
if(ONNXRUNTIME_CUDA_LIB AND ONNXRUNTIME_SHARED_LIB)
  target_link_libraries(detections_pkg_node
    ${ONNXRUNTIME_CUDA_LIB}
    ${ONNXRUNTIME_SHARED_LIB}
  )
  target_compile_definitions(detections_pkg_node PRIVATE HAS_ORT_CUDA=1)
  message(STATUS "ONNX Runtime CUDA provider found")
else()
  message(STATUS "ONNX Runtime CUDA provider not found, CPU only")
endif()

ament_target_dependencies(detections_pkg_node
  rclcpp
  std_msgs
  ament_index_cpp
)

# -------------------------------------------------------
# Install
# -------------------------------------------------------
install(TARGETS
  detections_pkg_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include
)

# -------------------------------------------------------
# Launch files
# -------------------------------------------------------
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

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
