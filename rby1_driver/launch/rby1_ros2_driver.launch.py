import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory('rby1_driver'),
        'config',
        'driver_parameters.yaml'
    )
    
    # Declare the namespace launch argument
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='rby1',
        description='Top-level namespace for the robot node, topics, and services'
    )
    
    namespace = LaunchConfiguration('namespace')

    rby1_ros2_driver = Node(
        package='rby1_driver',
        executable='rby1_ros2_driver',
        name='rby1_ros2_driver',
        namespace=namespace,
        parameters=[parameter_file],
        output='screen',
        prefix=['chrt -f 1'],
    # arguments=['--ros-args', '--log-level', 'DEBUG'] # ros2 의 디버깅 모드 설정
    )

    # The driver's odometry moves base_footprint in odom; the robot model's root is base, which
    # is on the floor (the SDK's wheel radius equals the axles' height: M 0.076 m, A 0.1 m).
    # This joins the two TF trees. M (mecanum): base_footprint is base. A (differential drive):
    # base_footprint is the middle of the wheel axle, which the SDK model puts 0.228 m in front
    # of base (<mobile xyz="0.228 0 0">), so base is 0.228 m behind it.
    with open(parameter_file) as stream:
        sections = yaml.safe_load(stream) or {}
    models = [section['ros__parameters']['model'] for section in sections.values()
              if 'model' in (section or {}).get('ros__parameters', {})]
    model = str(models[0]).strip().lower() if len(models) == 1 else None
    if model not in ('a', 'm'):
        raise ValueError(f'{parameter_file}: model must be a or m, got {model!r}')
    base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base',
        namespace=namespace,
        arguments=['--x', '-0.228' if model == 'a' else '0.0',
                   '--frame-id', 'base_footprint', '--child-frame-id', 'base'],
        output='screen',
    )

    return LaunchDescription([
        declare_namespace_cmd,
        rby1_ros2_driver,
        base_footprint,
    ])
