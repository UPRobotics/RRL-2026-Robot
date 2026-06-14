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
find_package(robot_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Common include dirs
set(COMMON_INCLUDES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Common ROS deps
set(COMMON_DEPS
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  robot_msgs
  ament_index_cpp
)

# ---------- ARM NODE ----------
add_executable(arm_node
  src/arm.cpp
  src/VESC.cpp
  src/utils/ros_vesc_motor.cpp
)

target_include_directories(arm_node PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(arm_node ${COMMON_DEPS})

# typesupport targets for message packages are not required to be retrieved here;
# ament_target_dependencies already links the necessary message headers and libs.

target_link_libraries(arm_node serial)

install(TARGETS arm_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# ---------- BODY NODE ----------
add_executable(body_node
  src/body.cpp
  src/VESC.cpp
  src/utils/ros_vesc_motor.cpp
)

target_include_directories(body_node PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(body_node ${COMMON_DEPS})


target_link_libraries(body_node serial)

install(TARGETS body_node
  DESTINATION lib/${PROJECT_NAME}
)


# ---------- MOTOR SIMULATOR ----------
add_executable(motor_sim_node
  src/motor_sim.cpp
)

target_include_directories(motor_sim_node PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(motor_sim_node rclcpp robot_msgs)
install(TARGETS motor_sim_node
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
