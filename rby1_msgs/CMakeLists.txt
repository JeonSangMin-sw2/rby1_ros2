cmake_minimum_required(VERSION 3.10)
project(rby1_msgs)

# Suppress CMP0148 warning about deprecated FindPythonInterp (used by rosidl in Humble)
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/StateOnOff.srv"
  "srv/ControlMode.srv"
  "action/MultiJointCommand.action"
  "action/SingleJointCommand.action"
  "action/StreamPosition.action"
  "msg/AllMotorState.msg"
  "msg/PowerState.msg"
  "msg/ToolFlangeState.msg"
  "msg/TorqueVelocityState.msg"
  "msg/CartesianPose.msg"
  DEPENDENCIES action_msgs trajectory_msgs geometry_msgs
)

ament_package()
