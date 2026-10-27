"""A virtual gripper per hand: commands in, finger state out (nothing on the robot moves).

  ros2 launch rby1_additional_tools virtual_gripper.launch.py [config:=/path/to/virtual_gripper.yaml]
  ros2 topic pub --once /rby1/right_gripper/command std_msgs/msg/Float64 "{data: 0.0}"   # close
  ros2 topic echo /rby1/right_gripper/state

The RB-Y1 driver does not drive the grippers. This node stands in for one: it takes each
finger's opening (m, 0 closed .. 0.05 open) on /rby1/<hand>_gripper/command and publishes
the fingers moving there on /rby1/<hand>_gripper/state. The gripper modules of
rby1_moveit_objects (objects.launch.py config:=gripper.yaml) follow that state in the
planning scene. Settings: config/virtual_gripper.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('rby1_additional_tools'), 'config', 'virtual_gripper.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default, description='virtual_gripper settings'),
        Node(package='rby1_additional_tools', executable='virtual_gripper', name='virtual_gripper',
             parameters=[LaunchConfiguration('config')], output='screen'),
    ])
