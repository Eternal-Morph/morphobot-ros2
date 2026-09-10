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

# Install scripts directory
install(DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
)

# Install executable scripts for ros2 run
install(PROGRAMS
  scripts/flight_teleop.py
  scripts/katla.py
  scripts/mode.py
  scripts/circle.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/flight_teleop.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME flight_teleop
)

install(PROGRAMS
  scripts/katla.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME katla
)

install(PROGRAMS
  scripts/mode.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mode
)

install(PROGRAMS
  scripts/circle.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME circle
)

# Install URDF/xacro files
install(DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

# Install mesh files
install(DIRECTORY meshes/
  DESTINATION share/${PROJECT_NAME}/meshes
)

# Install world files
install(DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

ament_package()
