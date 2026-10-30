# Tools (`rby1_tools`)

[Overview](01_overview.md) · [Driver](02_driver.md) · [Examples](03_examples.md) · [Description](04_description.md) · [Hardware Interface](05_hardware_interface.md) · [MoveIt](06_moveit.md) · [Tools](07_tools.md)

What sits on the robot besides the robot: cameras and their place on it, the gripper, and (planned) a LiDAR.

## Camera

Publishes a camera's images, its model and its place on the robot, so a marker detector or any other vision node —
on this machine or another — only needs topics. Two kinds: a webcam through OpenCV, and an Intel RealSense through
librealsense directly (no realsense-ros; only the streams switched on are opened).

```bash
ros2 launch rby1_tools camera.launch.py                          # webcam (config/webcam.yaml)
ros2 launch rby1_tools camera.launch.py source:=/dev/video2      # another webcam
ros2 launch rby1_tools camera.launch.py camera:=file source:=/path/tag.png   # an image or a video file
ros2 launch rby1_tools camera.launch.py camera:=realsense        # RealSense (config/realsense.yaml)
ros2 launch rby1_tools camera.launch.py intrinsics:=/path/camera_intrinsics.yaml   # a calibration file
ros2 launch rby1_tools camera.launch.py rviz:=true               # with RViz (config/camera.rviz)
```

| Launch argument | Default | Meaning |
|-----------------|---------|---------|
| `camera` | `webcam` | `webcam`, `file`, `realsense` |
| `config` | `config/webcam.yaml` / `config/realsense.yaml` | Settings file |
| `source` | from the settings file | Webcam: `0`, `/dev/video0`. File: an image (published over and over) or a video (looped) |
| `intrinsics` | from the settings file | Calibration file; giving one switches `use_custom_intrinsics` on |
| `mount` | `config/camera_mount.yaml` | Where the camera sits on the robot |
| `rviz` | `false` | RViz with the image, the frames drawn over it, and the frames in 3D |

`config/webcam.yaml` (`camera_publisher`):

| Setting | Default | Meaning |
|---------|---------|---------|
| `source` | `"0"` | Device number, `/dev/videoN`, or a file |
| `width`, `height`, `fps` | `1280`, `720`, `30.0` | Asked of the camera (the size it gives is used) |
| `auto_exposure`, `exposure` | `true`, `100.0` | Manual exposure in the driver's unit (V4L2: 100 µs) |
| `image_topic`, `info_topic` | `/camera/image_raw`, `/camera/camera_info` | Published topics |
| `frame_id` | `camera_optical_frame` | Frame of the image |
| `use_custom_intrinsics`, `intrinsics_file` | `false`, `""` | Camera model from a calibration file |
| `horizontal_fov` | `69.0` | Horizontal field of view (deg) of the pinhole model used without a calibration; distances are then approximate |

`config/realsense.yaml` (`realsense_publisher`):

| Setting | Default | Meaning |
|---------|---------|---------|
| `serial` | `""` | Which device when several are connected (`rs-enumerate-devices -s`); empty: the first |
| `width`, `height`, `fps` | `1280`, `720`, `30` | The same for every stream switched on |
| `use_rgb`, `use_depth`, `use_ir_left`, `use_ir_right` | `true`, `false`, `false`, `false` | Streams to open. Colour `bgr8`, depth `16UC1` (mm), infrared `mono8` |
| `align_depth_to_color` | `false` | Depth aligned to the colour image's pixels and frame |
| `rgb_topic` … `ir_right_info_topic` | `/camera/image_raw`, `/camera/camera_info`, `/camera/depth/…`, `/camera/ir_left/…`, `/camera/ir_right/…` | Topics per stream |
| `frame_id`, `depth_frame_id`, `ir_right_frame_id` | `camera_optical_frame`, `camera_depth_optical_frame`, `camera_ir_right_optical_frame` | Depth and infrared frames hang under the colour frame by the device's extrinsics (static TF) |
| `auto_exposure`, `exposure`, `gain` | `true`, `6000.0` µs, `-1` | Colour exposure |
| `ir_auto_exposure`, `ir_exposure` | `true`, `8500.0` µs | Infrared and depth exposure (a D405 shares one sensor: the colour values apply) |
| `emitter` | `true` | Infrared projector: on for depth, off for a clean infrared image |
| `skip_frames` | `10` | Frames dropped at start while the exposure settles |
| `use_custom_intrinsics`, `intrinsics_file` | `false`, `""` | Colour camera model from a calibration file instead of the factory calibration |

| Topic / frame | Content |
|---------------|---------|
| `/camera/image_raw`, `/camera/camera_info` | Colour image (`sensor_msgs/Image`, `bgr8`) and camera model |
| TF `link_head_2 → camera_link` | Where the camera sits: `config/camera_mount.yaml` (`parent`, `frame`, `xyz`, `rpy`; the default is the head mount, 22 mm forward and 40 mm up of `link_head_2`) |
| TF `camera_link → camera_optical_frame` | Frame of the image (z forward, x right, y down) |
| TF `camera_optical_frame → camera_depth_optical_frame`, … | RealSense depth and infrared frames |

- **librealsense**: the RealSense node builds against `ros-$ROS_DISTRO-librealsense2` (installed by `rosdep`);
  without it only this node is left out of the build. Intel's own `librealsense2` package carries a Fast DDS that
  clashes with ROS's in one process; when only that one is installed the launch runs the node on CycloneDDS. A normal
  user needs the RealSense udev rules to open the camera:
  ```bash
  sudo curl -sSL https://raw.githubusercontent.com/IntelRealSense/librealsense/v2.56.5/config/99-realsense-libusb.rules -o /etc/udev/rules.d/99-realsense-libusb.rules
  sudo udevadm control --reload-rules && sudo udevadm trigger
  ```
- **Frame rate**: auto exposure is kept from lowering it; a USB 2 connection and frames arriving slower than `fps` are
  warned about. Measured: D405, USB 3.2, 1280×720 colour at 30 Hz. A program that opened the camera through libusb
  leaves it invisible until it is plugged in again; the node's message says so.
- **Calibration** (`intrinsics_file`), OpenCV style (`camera_matrix: [[fx,0,cx],[0,fy,cy],[0,0,1]]`, `dist_coeffs`,
  `width`, `height`) or ROS `camera_calibration` style. Another image size scales focal length and centre (same aspect
  ratio only). One can be made with `ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.025
  image:=/camera/image_raw`. A marker's position in robot frames is only as good as the mount and the calibration.
- **Marker detector**: not part of this repository. Any node works that publishes each marker as
  `/rby1/marker_<id>/pose` (`geometry_msgs/PoseStamped` in the camera's optical frame) and as TF `target_marker_<id>`;
  the RB-Y1 Isaac ROS repository has one (`rby1_apriltag`).

Source: `src/camera_publisher.cpp`, `src/realsense_publisher.cpp`, `src/camera_model.cpp` (tested).

## Gripper

The driver does not drive the grippers, and the robot model up to v1.2 ends at the tool frames. `rby1_tools` carries
the RB-Y1 gripper as a model, as parts for the planning scene, and as a controller — virtual, or the real one:

| | File |
|---|---|
| Model | `model/rby1_gripper/gripper.urdf.xacro`: the body and two prismatic fingers (0..0.05 m) per hand, rooted at the tool frame `ee_<hand>`, with the mount offset as a fixed joint; meshes in `model/rby1_gripper/meshes/` |
| Planning scene | `model/rby1_gripper/gripper.yaml`: the same body and fingers as modules on `ee_right` / `ee_left` for `rby1_moveit_objects`, each finger following its joint on `/rby1/<hand>_gripper/state` ([MoveIt — Objects](06_moveit.md#objects-rby1_moveit_objects)) |
| Mount | `config/gripper_mount.yaml`: `right` and `left`, each `parent` (`ee_right` / `ee_left`), `xyz`, `rpy`. The RB-Y1 gripper mounts with no offset; measure yours (an adapter plate, another gripper) and change it. The model and the scene parts are placed with it |
| Virtual controller | `virtual_gripper`, `config/virtual_gripper.yaml`: nothing moves, the fingers are reported going where they are told at `speed` |
| Real controller | `gripper_driver`, `config/gripper_driver.yaml`: the gripper itself through the SDK's Dynamixel bus |

```bash
ros2 launch rby1_tools gripper.launch.py                                 # model on TF + virtual controller
ros2 launch rby1_tools gripper.launch.py scene:=true                     # and its parts in the planning scene (needs a move_group)
ros2 launch rby1_tools gripper.launch.py controller:=real                # the gripper itself (on the UPC)
ros2 launch rby1_tools gripper.launch.py hands:=right                    # one hand
ros2 launch rby1_tools gripper.launch.py model:=false joint_states_topic:=/joint_states   # RB-Y1 v1.3
ros2 topic pub --once -w 1 /rby1/right_gripper/grasp std_msgs/msg/Bool "{data: true}"         # close
ros2 topic pub --once -w 1 /rby1/right_gripper/command std_msgs/msg/Float64 "{data: 0.02}"   # each finger 2 cm open
ros2 topic echo /rby1/right_gripper/state
```

| Launch argument | Default | Meaning |
|---|---|---|
| `model` | `true` | A `robot_state_publisher` per hand with the gripper's URDF, mounted on the tool frame: the frames `gripper_<hand>`, `gripper_finger_<r\|l>1`, `_2` on TF under `ee_<hand>`, and the meshes for RViz (RobotModel display, topic `/gripper/<hand>/robot_description`). **Not for an RB-Y1 v1.3**, whose URDF carries the gripper already |
| `scene` | `false` | The gripper's parts in the planning scene, with the mount applied (through `objects.launch.py`; needs a `move_group`) |
| `controller` | `virtual` | `virtual`, `real` or `none` (a controller runs elsewhere) |
| `hands` | `right,left` | `right`, `left`, or both |
| `joint_states_topic` | `/gripper/joint_states` when `model` is on, else none | Where the controller publishes the finger joints' states; `/joint_states` on a v1.3 |
| `urdf`, `mount`, `objects`, `config` | the package's files above | Paths; `config` is the controller's settings |

Either controller takes, per hand, `/rby1/<hand>_gripper/command` (`std_msgs/Float64`, each finger's opening in m:
0 closed .. `max_opening` open; outside that range it is refused with a warning) and `/rby1/<hand>_gripper/grasp`
(`std_msgs/Bool`: `true` closes, `false` opens), and publishes `/rby1/<hand>_gripper/state`
(`sensor_msgs/JointState`: `gripper_finger_<r|l>1` and `_2`, both at the opening, at `rate` Hz) and, on
`joint_states_topic`, the URDF's finger joints (`gripper_finger_<r|l>1_joint` at +opening, `_2_joint` at −opening).

`config/virtual_gripper.yaml`: `hands`, `topic_prefix` (`/rby1`), `max_opening` (0.05 m), `speed` (0.05 m/s),
`rate` (50 Hz), `initial_opening` (0.05, open).

`config/gripper_driver.yaml` (**real**; built only where `RBY1_SDK_PATH` is set, run on the PC the gripper's USB is
plugged into, the UPC): `device` (`/dev/rby1_gripper`), `right_id` 0, `left_id` 1 (the Dynamixel id per hand),
`max_opening`, `rate`, `homing` (`true`), `homing_current` (0.3 A), `hold_current` (5), `inverted` (`false`). As SDK
example 35 drives it: at start each motor is **homed** — pushed with `homing_current` to one end and the other until
the encoder stops, which gives its travel; **the fingers move** — then current-based position control with
`hold_current` as the current limit; the state is read back from the encoders. `inverted` is for a motor whose
encoder rises when the gripper closes. The driver was compiled and not run on a gripper (2026-10-30).

- **v1.3**: the robot model has the gripper, so `model:=false` and no scene; the controller feeds the model's finger
  joints with `joint_states_topic:=/joint_states`. `rby1_bringup` does this when `moveit_executor.version` is `1.3`
  and refuses `gripper.model` or `gripper.scene` switched on.
- **In the planning scene** the fingers follow the controller's state (moved at most every 0.1 s, when they moved
  more than 1 mm), so a grasp closes the planner's model of the hand too. The parts of one gripper may touch each
  other and the wrist (`touch_links`); a tool held in the gripper needs the gripper's parts in its own `touch_links`.
- **Mount offset**: the model's fixed joint takes it from the file; for the scene the launch rewrites the parts'
  poses and finger axes by it (a temporary file), so both agree. Simulator (2026-10-30): with the right gripper 2 cm
  down and turned 90°, TF and the scene both put its open finger at (0, 0.053, −0.095) in `ee_right`.
- Simulator, virtual controller: TF of the open finger at x 0.053, closed 0.003, at 2 cm 0.023; the scene's six
  parts attached and following; RViz showed the model opening and closing. In `rby1_bringup` this is `GRIPPER`.
- `ros2 topic pub --once` without `-w 1` can lose the message to a subscriber that is not matched yet.

Source: `src/virtual_gripper.cpp` (the step and the command check in `include/rby1_tools/virtual_gripper.hpp`,
tested), `src/gripper_driver.cpp`, `launch/gripper.launch.py`; `launch/virtual_gripper.launch.py` starts the virtual
controller alone.

## LiDAR

Planned; nothing of it is in the repository yet. It will follow the camera's pattern: a publisher launch, a mount
file placing the sensor's frame on the robot, and its topics under `/rby1/lidar/…` or the sensor's own namespace.
