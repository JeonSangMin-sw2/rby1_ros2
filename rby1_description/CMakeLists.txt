cmake_minimum_required(VERSION 3.8)
project(rby1_description)

# Suppress CMP0148 warning about deprecated FindPythonInterp (used by rosidl in Humble)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

install(
  DIRECTORY urdf meshes initial_position launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES initial_position/initial_positions.yaml
  DESTINATION share/${PROJECT_NAME}/initial_position
)


ament_package()
