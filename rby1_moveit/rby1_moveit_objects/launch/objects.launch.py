"""Modules on the robot and fixtures around it, into the MoveIt planning scene.

  ros2 launch rby1_moveit_objects objects.launch.py [config:=/path/to/objects.yaml]
  ros2 launch rby1_moveit_objects objects.launch.py config:=gripper.yaml    # a file of this package's config/

Needs a running move_group (MoveIt or the cuMotion launch). The objects stay while this
runs -- put back if move_group restarts -- and are taken away on Ctrl+C. The file
format is described at the top of config/objects.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def publisher(context):
    """The node, with `config` as given or -- a bare file name -- from this package's config/."""
    share = os.path.join(get_package_share_directory('rby1_moveit_objects'), 'config')
    config = LaunchConfiguration('config').perform(context)
    if not os.path.isfile(config) and os.path.isfile(os.path.join(share, config)):
        config = os.path.join(share, config)
    return [Node(package='rby1_moveit_objects', executable='publish_objects', output='screen',
                 parameters=[{'config': config}])]


def generate_launch_description():
    default = os.path.join(get_package_share_directory('rby1_moveit_objects'), 'config', 'objects.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default,
                              description='YAML file of objects: a path, or a file name in this package\'s config/'),
        OpaqueFunction(function=publisher),
    ])
