from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rby1_examples',
            executable='power_control_example',
            name='power_control_example',
            output='screen'
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='rby1_examples',
                    executable='single_joint_example',
                    name='single_joint_example',
                    output='screen'
                )
            ]
        ),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='rby1_examples',
                    executable='multi_joint_example',
                    name='multi_joint_example',
                    output='screen'
                )
            ]
        ),
    ])
