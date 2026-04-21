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
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

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
  robot_msgs
  ament_index_cpp
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

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
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



# ---------- FAST-LIO REMAPPER ----------
add_executable(fast_lio_remapper_node
  src/fast_lio_remapper.cpp
)

target_include_directories(fast_lio_remapper_node PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(fast_lio_remapper_node
  rclcpp
  nav_msgs
  tf2_ros
  geometry_msgs
)

install(TARGETS fast_lio_remapper_node
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
