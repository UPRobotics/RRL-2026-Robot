cmake_minimum_required(VERSION 3.8)
project(robot_pkg)

# 1. OPCIONES DE COMPILACIÓN
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 2. BUSCAR DEPENDENCIAS
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 3. DECLARAR EJECUTABLE
# Aquí unimos todos tus archivos en un solo nodo
add_executable(robot_node
  src/main.cpp
  src/arm.cpp
  src/body.cpp
  src/VESC.cpp
)

# 4. CONECTAR LIBRERÍAS AL EJECUTABLE
ament_target_dependencies(robot_node
  rclcpp
  std_msgs
)

# 5. INSTALACIÓN (Para que 'ros2 run' funcione)
install(TARGETS
  robot_node
  DESTINATION lib/${PROJECT_NAME}
)

# 6. PRUEBAS (Linter)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 7. FINALIZACIÓN (Solo una vez y al final)
ament_package()