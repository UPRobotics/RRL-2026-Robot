cmake_minimum_required(VERSION 3.8)
project(telemetry_pkg)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Common include dirs
set(COMMON_INCLUDES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /usr/include
)
find_package(ament_index_cpp REQUIRED)
# Common ROS/External deps
set(COMMON_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  ament_index_cpp
)

# ---------- TELEMETRY NODE ----------
add_executable(telemetryPublisher_node
  src/TelemetryPublisher.cpp
)

target_include_directories(telemetryPublisher_node PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(telemetryPublisher_node ${COMMON_DEPS})
install(TARGETS
  telemetryPublisher_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/telemetry_pkg/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING PATTERN "*.json"
)

# ---------- LINT ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Skipping specific lints if they are too strict for your json setup
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()