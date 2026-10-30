"""A virtual gripper per hand: commands in, finger state out (nothing on the robot moves).
gripper.launch.py starts the same node together with the gripper's model and scene; this is the node alone.

  ros2 launch rby1_tools virtual_gripper.launch.py [config:=/path/to/virtual_gripper.yaml]
  ros2 launch rby1_tools virtual_gripper.launch.py joint_states_topic:=/joint_states   # RB-Y1 v1.3
  ros2 topic pub --once /rby1/right_gripper/command std_msgs/msg/Float64 "{data: 0.0}"   # close
  ros2 topic echo /rby1/right_gripper/state

The RB-Y1 driver does not drive the grippers. This node stands in for one: it takes each
finger's opening (m, 0 closed .. 0.05 open) on /rby1/<hand>_gripper/command and publishes
the fingers moving there on /rby1/<hand>_gripper/state. The gripper modules of
rby1_tools (gripper.launch.py scene:=true) follow that state in the
planning scene. Settings: config/virtual_gripper.yaml. An RB-Y1 v1.3 has the gripper in its
URDF instead; with joint_states_topic its prismatic finger joints get their states from here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default = os.path.join(get_package_share_directory('rby1_tools'), 'config', 'virtual_gripper.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default, description='virtual_gripper settings'),
        DeclareLaunchArgument('joint_states_topic', default_value='',
                              description="where the URDF finger joints' states go (RB-Y1 v1.3: /joint_states); empty: nowhere"),
        Node(package='rby1_tools', executable='virtual_gripper', name='virtual_gripper',
             parameters=[LaunchConfiguration('config'),
                         {'joint_states_topic': LaunchConfiguration('joint_states_topic')}],
             output='screen'),
    ])
