cmake_minimum_required(VERSION 3.8)
project(eternalmorph_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install executable scripts for ros2 run
install(PROGRAMS
  launch/katla.py
  launch/mode.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  launch/katla.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME katla
)

install(PROGRAMS
  launch/mode.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mode
)

# Install URDF/xacro files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install mesh files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

ament_package()
