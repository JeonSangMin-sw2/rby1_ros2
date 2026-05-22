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
            'power_control = rby1_examples.power_control:main',
            'joint_command = rby1_examples.joint_command:main',
            'zero_pose = rby1_examples.zero_pose:main',
            'cancel_control = rby1_examples.cancel_control:main',
            'cartesian_command = rby1_examples.cartesian_command:main',
            'multi_controls = rby1_examples.multi_controls:main',
            'stream_joint_control = rby1_examples.stream_joint_control:main',
            'tool_flange_test = rby1_examples.tool_flange_test:main',
            'joint_state_monitoring = rby1_examples.joint_state_monitoring:main',
            'tool_flange_monitoring = rby1_examples.tool_flange_monitoring:main',
            'robot_status_monitor = rby1_examples.robot_status_monitor:main',
            'gravity_compensation = rby1_examples.gravity_compensation:main',
            'brake_control = rby1_examples.brake_control_example:main',
        ],
    },
)
