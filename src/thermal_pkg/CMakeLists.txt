cmake_minimum_required(VERSION 3.8)
project(thermal_pkg)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# -------------------------------------------------------
# Thermal camera node (serial reader + publisher)
# -------------------------------------------------------
add_executable(thermal_camera_node
  src/thermal_camera_node.cpp
)
target_include_directories(thermal_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(thermal_camera_node
  rclcpp
  std_msgs
)

# -------------------------------------------------------
# Thermal display node (subscriber + OpenCV visualization)
# -------------------------------------------------------
add_executable(thermal_display_node
  src/thermal_display_node.cpp
)
target_include_directories(thermal_display_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(thermal_display_node
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(thermal_display_node
  rclcpp
  std_msgs
)

# -------------------------------------------------------
# Install
# -------------------------------------------------------
install(TARGETS
  thermal_camera_node
  thermal_display_node
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
