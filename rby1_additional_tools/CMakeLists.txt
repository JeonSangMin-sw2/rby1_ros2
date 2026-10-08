cmake_minimum_required(VERSION 3.10)
project(rby1_additional_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)
# librealsense2: ROS's build (ros-humble-librealsense2) first. Intel's own package carries
# a Fast DDS of its own and exports it, which clashes with ROS's in one process; with
# that one the launch has to run the node on Cyclone DDS (REALSENSE_FROM, read by it).
# Looked for afresh every time: a build configured before ros-humble-librealsense2 was
# installed remembers Intel's (realsense2_DIR in the cache) and would keep linking it.
unset(realsense2_DIR CACHE)
find_package(realsense2 QUIET PATHS /opt/ros/$ENV{ROS_DISTRO} NO_DEFAULT_PATH)
if(NOT realsense2_FOUND)
  find_package(realsense2 QUIET)
endif()
if(realsense2_DIR MATCHES "^/opt/ros/")
  set(REALSENSE_FROM ros)
else()
  set(REALSENSE_FROM system)
endif()
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs videoio imgproc)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_library(${PROJECT_NAME}_core STATIC src/camera_model.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
ament_target_dependencies(${PROJECT_NAME}_core sensor_msgs)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

add_executable(camera_publisher src/camera_publisher.cpp)
target_link_libraries(camera_publisher ${PROJECT_NAME}_core ${OpenCV_LIBS})
ament_target_dependencies(camera_publisher rclcpp sensor_msgs cv_bridge)

# The RealSense node needs librealsense2 (ros-humble-librealsense2); without it the rest builds.
if(realsense2_FOUND)
  add_executable(realsense_publisher src/realsense_publisher.cpp)
  target_include_directories(realsense_publisher PRIVATE ${EIGEN3_INCLUDE_DIR})
  target_link_libraries(realsense_publisher ${PROJECT_NAME}_core realsense2::realsense2)
  ament_target_dependencies(realsense_publisher rclcpp sensor_msgs geometry_msgs tf2_ros)
  install(TARGETS realsense_publisher DESTINATION lib/${PROJECT_NAME})
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/realsense_from "${REALSENSE_FROM}\n")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/realsense_from DESTINATION share/${PROJECT_NAME})
  message(STATUS "realsense_publisher: librealsense2 ${realsense2_VERSION} from ${realsense2_DIR} (${REALSENSE_FROM})")
else()
  message(WARNING "librealsense2 not found: realsense_publisher is not built (sudo apt install ros-humble-librealsense2)")
endif()

install(TARGETS camera_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_camera_model test/test_camera_model.cpp)
  target_link_libraries(test_camera_model ${PROJECT_NAME}_core)
endif()

ament_package()
