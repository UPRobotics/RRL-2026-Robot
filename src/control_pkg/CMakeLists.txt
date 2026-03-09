cmake_minimum_required(VERSION 3.8)
project(control_pkg)

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

# ---------- JOYSTICK NODE ----------
add_executable(joystick_node
  src/JoyStickListener.cpp
)

target_include_directories(joystick_node PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(joystick_node ${COMMON_DEPS})

install(TARGETS joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(joystick_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS joystick_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
