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
            'single_joint = rby1_examples.single_joint:main',
            'multi_joint = rby1_examples.multi_joint:main',
            'zero_pose = rby1_examples.zero_pose:main',
            'cancel_control = rby1_examples.cancel_control:main',
            'cartesian_impedance = rby1_examples.cartesian_impedance:main',
            'joint_impedance = rby1_examples.joint_impedance:main',
            'joint_group = rby1_examples.joint_group:main',
            'multi_controls = rby1_examples.multi_controls:main',
            'stream_joint_control = rby1_examples.stream_joint_control:main',
            'tool_flange_test = rby1_examples.tool_flange_test:main',
            'cartesian_control = rby1_examples.cartesian_control:main',
            'state_monitoring = rby1_examples.state_monitoring:main',
            'robot_status_monitor = rby1_examples.robot_status_monitor:main',
        ],
    },
)
