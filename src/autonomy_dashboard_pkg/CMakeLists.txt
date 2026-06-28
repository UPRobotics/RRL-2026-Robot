cmake_minimum_required(VERSION 3.8)
project(autonomy_dashboard_pkg)

find_package(ament_cmake REQUIRED)

# Instala módulos Python
install(
  DIRECTORY autonomy_dashboard
  DESTINATION lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages
  PATTERN "__pycache__" EXCLUDE
)

# Instala scripts ejecutables
install(PROGRAMS
  autonomy_dashboard/autonomy_dashboard_node.py
  autonomy_dashboard/autonomy_ui.py
  autonomy_dashboard/test_ui.py
  DESTINATION lib/${PROJECT_NAME}
)

# Instala directorios de configuración
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
