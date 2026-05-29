import os
from glob import glob
from setuptools import setup

package_name = 'rby1_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jsm',
    maintainer_email='sangmin.jeon@rainbow-robotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            '01_power_control = rby1_examples.01_power_control:main',
            '02_brake_control = rby1_examples.02_brake_control_example:main',
            '03_robot_status_monitor = rby1_examples.03_robot_status_monitor:main',
            '04_tool_flange_monitoring = rby1_examples.04_tool_flange_monitoring:main',
            '05_joint_state_monitoring = rby1_examples.05_joint_state_monitoring:main',
            '06_gravity_compensation = rby1_examples.06_gravity_compensation:main',
            '08_zero_pose = rby1_examples.08_zero_pose:main',
            '09_joint_command = rby1_examples.09_joint_command:main',
            '10_cartesian_command = rby1_examples.10_cartesian_command:main',
            '11_multi_controls = rby1_examples.11_multi_controls:main',
            '12_trajectory_joint_command = rby1_examples.12_trajectory_joint_command:main',
            '13_cancel_control = rby1_examples.13_cancel_control:main',
            '14_mobile_base_control = rby1_examples.14_mobile_base_control:main',
            '15_stream_command = rby1_examples.15_stream_command:main',
        ],
    },
)
 
