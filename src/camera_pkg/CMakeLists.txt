cmake_minimum_required(VERSION 3.8)
project(camera_pkg)

# Require C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -------------------------------------------------------
# Find ROS2 dependencies
# -------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# -------------------------------------------------------
# Find system dependencies via pkg-config
# -------------------------------------------------------
find_package(PkgConfig REQUIRED)

# SDL2
pkg_check_modules(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)

# FFmpeg
pkg_check_modules(LIBAVCODEC REQUIRED libavcodec)
pkg_check_modules(LIBAVFORMAT REQUIRED libavformat)
pkg_check_modules(LIBSWSCALE REQUIRED libswscale)
pkg_check_modules(LIBAVUTIL REQUIRED libavutil)

# spdlog
pkg_check_modules(SPDLOG REQUIRED spdlog)

# nlohmann-json
find_package(nlohmann_json QUIET)
if(NOT nlohmann_json_FOUND)
  pkg_check_modules(NLOHMANN_JSON REQUIRED nlohmann_json)
endif()

# -------------------------------------------------------
# Build the camera_pkg node
# -------------------------------------------------------
set(SOURCES
  src/main.cpp
  src/main_window.cpp
  src/camera_manager.cpp
  src/camera_stream.cpp
  src/thermal_stream.cpp
  src/camera_grid.cpp
  src/ui_helpers.cpp
  src/settings_manager.cpp
  src/console_window.cpp
  src/console_sink.cpp
  src/stats_panel.cpp
)

add_executable(camera_pkg_node ${SOURCES})

target_include_directories(camera_pkg_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SDL2_INCLUDE_DIRS}
  ${SDL2_TTF_INCLUDE_DIRS}
  ${LIBAVCODEC_INCLUDE_DIRS}
  ${LIBAVFORMAT_INCLUDE_DIRS}
  ${LIBSWSCALE_INCLUDE_DIRS}
  ${LIBAVUTIL_INCLUDE_DIRS}
  ${SPDLOG_INCLUDE_DIRS}
)

# Link against nlohmann_json (cmake target or pkg-config)
if(nlohmann_json_FOUND)
  target_link_libraries(camera_pkg_node nlohmann_json::nlohmann_json)
else()
  target_include_directories(camera_pkg_node PUBLIC ${NLOHMANN_JSON_INCLUDE_DIRS})
endif()

target_link_libraries(camera_pkg_node
  ${SDL2_LIBRARIES}
  ${SDL2_TTF_LIBRARIES}
  ${LIBAVCODEC_LIBRARIES}
  ${LIBAVFORMAT_LIBRARIES}
  ${LIBSWSCALE_LIBRARIES}
  ${LIBAVUTIL_LIBRARIES}
  ${SPDLOG_LIBRARIES}
  pthread
)

ament_target_dependencies(camera_pkg_node
  rclcpp
  std_msgs
)

# -------------------------------------------------------
# Install
# -------------------------------------------------------
install(TARGETS
  camera_pkg_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
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
# Testing
# -------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
