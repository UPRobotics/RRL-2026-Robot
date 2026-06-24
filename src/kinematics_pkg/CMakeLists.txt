cmake_minimum_required(VERSION 3.8)
project(kinematics_pkg)

# Compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
set(COMMON_INCLUDES
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Dependencies list
set(COMMON_DEPS
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
)

# ===== KINEMATICS LIBRARY =====
add_library(arm_kinematics_lib
  src/forward_kinematics.cpp
  src/inverse_kinematics.cpp
)

target_include_directories(arm_kinematics_lib PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(arm_kinematics_lib ${COMMON_DEPS})
target_link_libraries(arm_kinematics_lib Eigen3::Eigen)

# ===== KINEMATICS NODE =====
add_executable(arm_kinematics_node
  src/arm_kinematics.cpp
)

target_include_directories(arm_kinematics_node PUBLIC ${COMMON_INCLUDES})
ament_target_dependencies(arm_kinematics_node ${COMMON_DEPS})
target_link_libraries(arm_kinematics_node arm_kinematics_lib Eigen3::Eigen)

# Installation
install(TARGETS arm_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS arm_kinematics_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
