cmake_minimum_required(VERSION 3.10)
project(mic_pkg)

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
# Find ALSA via pkg-config
# -------------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_check_modules(ALSA REQUIRED alsa)

# -------------------------------------------------------
# mic_transmitter node
# -------------------------------------------------------
add_executable(mic_transmitter_node
  src/mic_transmitter_main.cpp
  src/mic_transmitter.cpp
)

target_include_directories(mic_transmitter_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ALSA_INCLUDE_DIRS}
)

target_link_libraries(mic_transmitter_node
  ${ALSA_LIBRARIES}
)

ament_target_dependencies(mic_transmitter_node
  rclcpp
  std_msgs
)

# -------------------------------------------------------
# Install
# -------------------------------------------------------
install(TARGETS mic_transmitter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
