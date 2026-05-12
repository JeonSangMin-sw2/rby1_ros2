import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

parameter_file = os.path.join(
    get_package_share_directory('rby1_driver'),
    'config',
    'driver_parameters.yaml'
)

rby1_ros2_driver = Node(
    package='rby1_driver',
    executable='rby1_ros2_driver',
    name='rby1_ros2_driver',
    parameters=[parameter_file],
    output='screen',
    # arguments=['--ros-args', '--log-level', 'DEBUG'] # ros2 의 디버깅 모드 설정
)


def generate_launch_description():
    return LaunchDescription([
        rby1_ros2_driver
    ])