"""A camera on the robot as ROS topics, for Isaac ROS AprilTag in the container.

  ros2 launch rby1_additional_tools camera.launch.py                         # webcam /dev/video0
  ros2 launch rby1_additional_tools camera.launch.py source:=/dev/video2
  ros2 launch rby1_additional_tools camera.launch.py camera:=file source:=/path/tag.png
  ros2 launch rby1_additional_tools camera.launch.py camera:=realsense       # librealsense, no realsense-ros
  ros2 launch rby1_additional_tools camera.launch.py intrinsics:=/path/camera_intrinsics.yaml
  ros2 launch rby1_additional_tools camera.launch.py rviz:=true              # with RViz: the image and the markers

Settings are in config/webcam.yaml (webcam and file) and config/realsense.yaml:
topics, size, frame rate, exposure, which RealSense streams, the camera model.
`config:=` names another file; `source:=` and `intrinsics:=` override those two
settings (a given intrinsics file switches use_custom_intrinsics on).

Whatever the camera, the container sees the same colour topics -- by default
/camera/image_raw and /camera/camera_info -- and the same frames:
config/camera_mount.yaml places camera_link on the robot (static TF), and
camera_optical_frame (z forward, the images' frame) sits under it. Cameras stay on
the host: no device mounts or camera drivers in the container.
"""

import math
import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

ARGUMENTS = {
    'camera': ('webcam', 'webcam, file or realsense'),
    'config': ('', 'settings YAML; empty: config/webcam.yaml or config/realsense.yaml'),
    'source': ('', 'webcam: device (0, /dev/video0); file: image or video path; empty: the config\'s'),
    'intrinsics': ('', 'calibration YAML (camera_ws or ROS layout); empty: the config\'s'),
    'mount': ('', 'camera mount YAML; empty: config/camera_mount.yaml'),
    'rviz': ('false', 'true: RViz with the image, the camera frames and the markers found (config/camera.rviz)'),
}
NODES = {'webcam': 'camera_publisher', 'file': 'camera_publisher', 'realsense': 'realsense_publisher'}


def mount_transform(path):
    """static_transform_publisher arguments for the mount file."""
    with open(path) as stream:
        mount = yaml.safe_load(stream)
    for key in ('parent', 'frame', 'xyz', 'rpy'):
        if key not in mount:
            raise ValueError(f'{path}: needs {key}')
    x, y, z = (float(v) for v in mount['xyz'])
    roll, pitch, yaw = (float(v) for v in mount['rpy'])
    return ['--x', str(x), '--y', str(y), '--z', str(z), '--roll', str(roll), '--pitch', str(pitch),
            '--yaw', str(yaw), '--frame-id', mount['parent'], '--child-frame-id', mount['frame']], mount['frame']


def failed(reason):
    return [LogInfo(msg=f'CAMERA_FAILED: {reason}'), Shutdown(reason=reason)]


def setup(context):
    value = {name: LaunchConfiguration(name).perform(context) for name in ARGUMENTS}
    share = get_package_share_directory('rby1_additional_tools')
    camera = value['camera']
    if camera not in NODES:
        return failed(f'camera must be webcam, file or realsense, got {camera!r}')
    executable = NODES[camera]
    if not os.path.isfile(os.path.join(get_package_prefix('rby1_additional_tools'), 'lib', 'rby1_additional_tools',
                                       executable)):
        return failed(f'{executable} is not built: sudo apt install ros-humble-librealsense2, then rebuild '
                      'rby1_additional_tools')
    config = value['config'] or os.path.join(share, 'config', 'realsense.yaml' if camera == 'realsense'
                                             else 'webcam.yaml')
    if not os.path.isfile(config):
        return failed(f'no settings file {config}')
    try:
        mount_args, body = mount_transform(value['mount'] or os.path.join(share, 'config', 'camera_mount.yaml'))
    except (OSError, ValueError, TypeError) as error:
        return failed(str(error))
    overrides = {}
    if value['source'] and camera != 'realsense':
        overrides['source'] = value['source']
    if value['intrinsics']:
        overrides.update(use_custom_intrinsics=True, intrinsics_file=value['intrinsics'])
    # Built against Intel's own librealsense2 package (not ros-humble-librealsense2), the
    # node dies on start next to ROS's Fast DDS -- that package carries a Fast DDS of its
    # own. Then it runs on Cyclone DDS, which talks to the Fast DDS nodes over the network.
    environment = {}
    origin = os.path.join(share, 'realsense_from')
    if camera == 'realsense' and os.path.isfile(origin) and open(origin).read().strip() != 'ros':
        environment = {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'}
    viewer = []
    if value['rviz'].lower() in ('true', '1'):
        viewer = [Node(package='rviz2', executable='rviz2', name='camera_rviz', output='log',
                       arguments=['-d', os.path.join(share, 'config', 'camera.rviz'), '-f', body])]
    return [
        *viewer,
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_mount',
             arguments=mount_args, output='log'),
        # camera_link (x forward, z up) -> camera_optical_frame (z forward, x right, y down).
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_optical',
             arguments=['--roll', str(-math.pi / 2), '--yaw', str(-math.pi / 2),
                        '--frame-id', body, '--child-frame-id', 'camera_optical_frame'], output='log'),
        Node(package='rby1_additional_tools', executable=executable, output='screen',
             parameters=[config, overrides], additional_env=environment, on_exit=Shutdown()),
    ]


def generate_launch_description():
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=default, description=text)
          for name, (default, text) in ARGUMENTS.items()],
        OpaqueFunction(function=setup),
    ])
