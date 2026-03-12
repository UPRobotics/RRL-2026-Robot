cmake_minimum_required(VERSION 3.10)
project(telemetry_ui_pkg)

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
find_package(robot_msgs REQUIRED)

# -------------------------------------------------------
# System deps via pkg-config
# -------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2     REQUIRED sdl2)
pkg_check_modules(SDL2_TTF REQUIRED SDL2_ttf)
pkg_check_modules(SPDLOG   REQUIRED spdlog)

# -------------------------------------------------------
# Sources
# -------------------------------------------------------
set(SOURCES
  src/main.cpp
  src/main_window.cpp
)

add_executable(telemetry_ui_node ${SOURCES})

target_include_directories(telemetry_ui_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${SDL2_INCLUDE_DIRS}
  ${SDL2_TTF_INCLUDE_DIRS}
  ${SPDLOG_INCLUDE_DIRS}
)

target_link_libraries(telemetry_ui_node
  ${SDL2_LIBRARIES}
  ${SDL2_TTF_LIBRARIES}
  ${SPDLOG_LIBRARIES}
  pthread
)

ament_target_dependencies(telemetry_ui_node
  rclcpp
  std_msgs
  robot_msgs
)

# -------------------------------------------------------
# Install
# -------------------------------------------------------
install(TARGETS
  telemetry_ui_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
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
