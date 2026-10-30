# ros2_control with the RB-Y1 hardware plugin (rby1_hardware/RBY1SystemHardware), without MoveIt.
#
# It starts the controller manager (ros2_control_node) with the robot's URDF, robot_state_publisher,
# the joint_state_broadcaster and the trajectory controllers listed below. The driver (rby1_driver)
# must already be running: the plugin takes the control right from it (<driver_namespace>/hardware_control)
# and then streams the joint positions to the robot itself.
#
#   ros2 launch rby1_hardware rby1_hardware.launch.py
#   ros2 launch rby1_hardware rby1_hardware.launch.py robot_ip:=192.168.30.1:50051
#   ros2 launch rby1_hardware rby1_hardware.launch.py use_fake_hardware:=true   # no robot, no driver
#
# Which robot this is for is set HERE, not on the command line. The URDF with its <ros2_control> tag and
# the controllers come from the MoveIt configuration package of that robot (only its files are used;
# nothing of MoveIt runs). Edit these lines for your RB-Y1:
MOVEIT_CONFIG_PACKAGE = 'rby1_moveit_m_1_2'     # rby1_moveit_{a,m}_{1_0,1_1,1_2,1_3}: model and version
ROBOT_DESCRIPTION = 'RBY1_M_v1_2.urdf.xacro'    # its config/*.urdf.xacro (RBY1_A_v1_0, RBY1_M_v1_3, ...)
CONTROLLERS = ['right_arm_controller', 'left_arm_controller', 'torso_controller', 'head_controller']
# ^ started active, from its config/ros2_controllers.yaml: right_arm_controller, left_arm_controller,
#   both_arms_controller, torso_controller, head_controller, body_controller (v1.3 also gripper_r/l_controller).
#   Two controllers cannot share a joint while active (both_arms_controller and right_arm_controller):
#   list one of them here and switch at run time with `ros2 control switch_controllers`.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory(MOVEIT_CONFIG_PACKAGE), 'config')
    model = MOVEIT_CONFIG_PACKAGE.split('_')[2]     # rby1_moveit_m_1_2 -> m

    robot_ip = LaunchConfiguration('robot_ip')
    driver_namespace = LaunchConfiguration('driver_namespace')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ', os.path.join(config_dir, ROBOT_DESCRIPTION),
            ' use_fake_hardware:=', use_fake_hardware,
            ' robot_ip:=', robot_ip,
            ' model:=', model,
            ' driver_namespace:=', driver_namespace,
        ]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # the controller manager: loads the hardware plugin named in the URDF and runs the controllers
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[{'robot_description': robot_description},
                    os.path.join(config_dir, 'ros2_controllers.yaml')],
        on_exit=Shutdown(),   # without it nothing else here is of use
    )

    spawners = [
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager'],
            output='screen',
        )
        for name in ['joint_state_broadcaster'] + CONTROLLERS
    ]

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='127.0.0.1:50051',
                              description='robot address and gRPC port (the driver connects to the same one)'),
        DeclareLaunchArgument('driver_namespace', default_value='rby1',
                              description='namespace of the running rby1_ros2_driver'),
        DeclareLaunchArgument('use_fake_hardware', default_value='false',
                              description='true: mock_components/GenericSystem instead of the robot (no driver needed)'),
        robot_state_publisher,
        controller_manager,
        *spawners,
    ])
