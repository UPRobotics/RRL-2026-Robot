cmake_minimum_required(VERSION 3.8)
project(robot_pkg)

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

# Common ROS deps
set(COMMON_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# ---------- ARM NODE ----------
add_executable(arm_node
  src/arm.cpp
  src/VESC.cpp
)

target_include_directories(arm_node PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(arm_node ${COMMON_DEPS})
target_link_libraries(arm_node serial)

install(TARGETS arm_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- BODY NODE ----------
add_executable(body_node
  src/body.cpp
  src/VESC.cpp
)

target_include_directories(body_node PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(body_node ${COMMON_DEPS})
target_link_libraries(body_node serial)

install(TARGETS body_node
  DESTINATION lib/${PROJECT_NAME}
)

# ---------- JOYSTICK NODE ----------
add_executable(joystick_node
  src/JoyStickListener.cpp
)

target_include_directories(joystick_node PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(joystick_node ${COMMON_DEPS})

install(TARGETS joystick_node
  DESTINATION lib/${PROJECT_NAME}
)


# ---------- LINT ----------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
