# RBY1 ROS 2 Driver Package

> [!CAUTION]
> ## For safe use, please test the features in a simulation first.

Please click the link below for more details.

[RBY1 ROS 2 Driver Documentation](https://rainbowrobotics.github.io/rby1-dev/ros2/ros2_driver.html)

## Overview

`rby1_ros2` is a unified ROS 2 driver package for controlling the Rainbow Robotics RBY1 robot.  
It wraps the RBY1 C++ SDK into a ROS 2 node, providing state monitoring and multiple control modes (Joint Position, Cartesian Position, Impedance, Gravity Compensation, and Trajectory Streaming) through a clean action/service/topic interface.

- **ROS 2 version**: Humble(recommend), Jazzy
- **OS**: Ubuntu 22.04
- **SDK compatibility**: rby1-sdk `0.10.x` and later

### System Architecture Overview

![System Architecture](Doc/img/system_architecture.png)

The system operates using two primary pipelines to interface with the robot:
1. **Direct Control Mode (`rby1_ros2_driver`)**: A standalone C++ ROS 2 node that communicates directly with the RBY1 robot or simulator via gRPC. User applications/scripts send standard ROS 2 topics, services, and actions (e.g. `robot_joint`, `robot_cartesian`) to command movements and monitor status.
2. **MoveIt 2 Integration (`rby1_hardware`)**: Bridges MoveIt 2 and `ros2_control` to the robot via a custom `RBY1SystemHardware` interface plugin. The hardware plugin streams command inputs to the robot via a direct gRPC connection, while querying status and coordinating power, servo states, and control rights with the `rby1_ros2_driver` node via internal ROS 2 service calls (like `/hardware_control`).

---

## 1. Quick Start

- **If you install in an environment such as conda or miniforge, issues may arise due to Python and CMake path conflicts, so please install it in a local environment.**

### 1-1. Install ROS 2 Humble

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

### 1-2. Install RB-Y1 SDK

<https://github.com/RainbowRobotics/rby1-sdk>

### 1-3. Install RBY1 Simulator (Docker)

<https://hub.docker.com/r/rainbowroboticsofficial/rby1-sim>

### 1-4. Install MoveIt 2

#### Option A(binary install. recommended in robot UPC)

- it can use robot's UPC(jetson).

```bash
sudo apt update
sudo apt install ros-humble-moveit
sudo apt install ros-humble-moveit-visual-tools ros-humble-interactive-markers

# install check
source /opt/ros/humble/setup.bash
ros2 pkg list | grep moveit
```
#### Option B(install package)
- Please proceed up to  `~ Optional: add the previous command to your .bashrc`

<https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html>

#### 1-5. Install additional tool

```bash
sudo apt install ros-humble-gripper-controllers
sudo apt install ros-humble-joint-trajectory-controller
```

### 1-6. Install Nav2(optional)

> [!CAUTION]
> ## rby1_nav didn't work yet.

<https://docs.nav2.org/development_guides/build_docs/index.html#build-instructions>



### 1-7. Environment Setup

Add the following lines to `~/.bashrc`:

```bash
sudo nano ~/.bashrc

# Add at the bottom:
export PATH=/opt/cmake/bin:$PATH
source /opt/ros/humble/setup.bash

# Apply changes
source ~/.bashrc
```

### 1-8. Real-time (RT) Priority Setup (Required for 100Hz Real-Time Control)

To enable real-time scheduling (`SCHED_FIFO`) and eliminate gRPC/ROS 2 thread latency under high CPU loads, configure real-time priority limits for your user account (e.g. `nvidia`):

```bash
# 1. Create a realtime limit configuration file (example for user 'nvidia'):
sudo bash -c 'cat << EOF > /etc/security/limits.d/99-realtime.conf
<user> - rtprio 99
<user> - memlock unlimited
EOF'

# Note: Replace 'nvidia' with your actual username (e.g. $USER) if different.

# 2. Apply settings by opening a new terminal session or logging out and back in.

# 3. Verify that real-time priority is enabled:
ulimit -r
# Expected output: 99
```
> [!IMPORTANT]
> Apply settings, it is necessary to reboot or log out/in.



### 1-9. Build

```bash
mkdir -p rby1_ros2_ws/src
cd rby1_ros2_ws/src
git clone https://github.com/RainbowRobotics/rby1-ros2.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

### 1-10. Configure `driver_parameters.yaml`

Located at `rby1_driver/config/driver_parameters.yaml`.  
Edit this file to match your robot before launching the driver.  
Because the workspace was built with `--symlink-install`, **no rebuild is needed** after editing.

> [!IMPORTANT]
> If you use simulation for testing, keep `robot_ip: "127.0.0.1:50051"`.  
> Some state values (battery, tool flange FT/IMU) will show zeros in simulation because no physical sensors are attached.

- Main Parameters(for the detail of driver_parameters.yaml, see config/driver_parametersyaml)

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `robot_ip` | `"127.0.0.1:50051"` | - | Robot IP address and gRPC port |
| `model` | `"m"` | - | Robot model — `"a"` (RBY1-A) or `"m"` (RBY1-M) |
| `get_state_period` | `0.01` | s | State publish interval — default 0.01(100 Hz) |
| `publish_battery_state` | `true` | - | Enable battery state topic |
| `publish_tool_flange_state` | `true` | - | Enable tool flange state topics (left + right) |
| `state_loss_timeout` | `1.0` | s | The driver stops only after state reads kept failing this long (a single failed read is retried) |
| `fjt_start_velocity_scale` | `1.0` | - | `follow_joint_trajectory` is rejected when reaching its first waypoint in time would need more than this share of a joint's velocity limit |

---

> [!NOTE]
> **`get_state_period` and communication frequency:**  
> `get_state_period` sets the interval (in seconds) at which the driver reads the robot state via `GetState()` and publishes all state topics.  
> Actual throughput may be slightly lower (97–100 Hz) depending on PC environment and CPU load.

![get_state_period_1](Doc/img/topic_hz.png)

### 1-11. Run Simulator (optional)

If you do not have a physical robot, run the Docker simulator.  
The robot IP in this case is `"127.0.0.1:50051"` or `"localhost:50051"`.  
Change the tag at the end to select a model/version (e.g. `a_v1.2`, `m_v1.3`).

```bash
# Example: Model A,  v1.2
sudo docker run --rm \
  -e DISPLAY=${DISPLAY} \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -p 50051:50051 \
  rainbowroboticsofficial/rby1-sim:0.10.6-a_v1.2
```

> [!IMPORTANT]
> ## Model `a` only supports  up to v1.2. Model `m` supports v1.0–v1.3.

---

### 1-12. Launch the Driver

```bash
# In your workspace root
source install/setup.bash

ros2 launch rby1_driver rby1_ros2_driver.launch.py

```

## Launch or Run

### Examples
Each example can be run in a **separate terminal** while the driver is active:
```bash
source install/setup.bash
ros2 run rby1_examples <example_name>
# ex ) ros2 run rby1_examples 01_power_control
```

### Visualization & Robot Description
You can use the robot's basic TF structure and state publisher through the commands below. When implementing features related to rby1, please use the model files from the corresponding package.

```bash
source install/setup.bash
ros2 launch rby1_description rby1_state_publisher.launch.py model:=a version:=1_1
```

### MoveIt 2

```bash
# open another terminal
source install/setup.bash

# Real hardware (default: use_fake_hardware:=false)
ros2 launch rby1_moveit_m_1_2 demo.launch.py

# With a custom robot IP
ros2 launch rby1_moveit_m_1_2 demo.launch.py robot_ip:=192.168.30.1:50051

# Fake hardware / simulation (no real robot required)
ros2 launch rby1_moveit_m_1_2 demo.launch.py use_fake_hardware:=true
```

## Additional Tools

> ⚠️ **Topic names changed.** The topics the tools below use between them were renamed so that everything of the robot
> is under `/rby1/` (target topics per arm, the marker topics, the tracking switch), and `rby1_moveit_scene` was merged
> into `rby1_moveit_objects`. If you have programs written against the earlier names, see the table in
> [Dev_page.md — Topic and service names](Dev_page.md#topic-and-service-names).

Tools that run next to the driver: a MoveIt planner that moves the arms through the driver, obstacles and modules for
its planning scene, a camera publisher, and the examples that use them. What each one is comes first, then a
step-by-step guide to try them. Parameters, behaviour and internals of every package are in [Dev_page.md](Dev_page.md).

| Package | What it does |
|---|---|
| `rby1_moveit_executor` | Plans with MoveIt (OMPL) to a tool pose target and executes through the driver's `follow_joint_trajectory` — no `ros2_control` |
| `rby1_moveit_objects` | Objects in the planning scene. The `scene` command adds, moves and removes obstacles (box, sphere, cylinder) and loads a file of them; `objects.launch.py` puts modules (gripper, sensors, tools) on robot links, or fixtures in the world, from a YAML file; `point_object.launch.py` keeps one obstacle where a point topic says |
| `rby1_additional_tools` | Camera publisher (webcam or Intel RealSense): images, the camera model, and the camera's mount on TF |
| `rby1_examples` `16_target_shuttle_publisher` | Publishes targets: two hand poses in turn, one every few seconds, for whatever listens on the target topic — the planner, or example 15. It only publishes |
| `rby1_examples` `15_cartesian_target_move` | The simplest target executor: takes targets from the same topic and moves the hand with the driver alone (one Cartesian command, or a stream of them) — no planner, nothing is checked for collisions |

A target is one message on `/rby1/right_arm/target_pose` (`/rby1/left_arm/target_pose` for the left arm): a
`std_msgs/Float64MultiArray` of 16 values, the row-major 4x4 pose of the tool frame in `base`. The executor answers on
`/rby1/right_arm/target_status` (`std_msgs/String`): `READY`, `PLANNING`,
`EXECUTING …`, `DONE`, or `FAILED: <reason and what to do>`. Any node on the same ROS domain can send one.
The planner serves both arms: a target for one arm moves that arm and leaves the other where it is, and targets sent
to both arms together move them in one motion.

### Step by step

Try it in the simulator first. Each step is a separate terminal with `source install/setup.bash`, while the driver runs.

**Step 1 — Start the planner.** It checks the robot, switches power and servos on, and takes whatever is straight to
the ready pose — both arms (elbow 90 deg) and the torso (knee bent a little, 0.2 rad), together. **The robot moves**,
on both sides. Then it shows `READY`. The ready pose is a file of joint angles,
`rby1_moveit_executor/config/ready_pose.yaml`: edit it, or give a copy with `ready_pose:=/path/to/file.yaml`.

```bash
ros2 launch rby1_driver rby1_ros2_driver.launch.py
ros2 launch rby1_moveit_executor moveit_executor.launch.py              # with RViz; rviz:=false without
```

**Step 2 — Move to a target.** With `cycles:=1` the publisher example sends two targets, `period` (5 s) apart: here
the hand goes 5 cm forward and up, then back to where the ready pose has it. A point is x, y, z, roll, pitch, yaw —
always six values; `-1.571, -1.071, 1.571` is how the right hand is turned at the ready pose. The example only
publishes, so watch the planner's answers in another terminal.

```bash
ros2 topic echo /rby1/right_arm/target_status std_msgs/msg/String       # another terminal: the planner's answers
ros2 run rby1_examples 16_target_shuttle_publisher --ros-args -p cycles:=1 \
    -p "point_a:=[0.463,-0.367,1.166,-1.571,-1.071,1.571]" -p "point_b:=[0.413,-0.367,1.116,-1.571,-1.071,1.571]"
```

Each target is answered with `PLANNING`, `EXECUTING …` and `DONE`; the example logs the same lines under the target
it sent. It does not wait for the hand: `period` must be longer than a move takes (`-p period:=8.0`). A target that
arrives while the arm is still moving runs when that move ends.

The left arm is moved the same way, with nothing restarted: give its two topics and points on its side.

```bash
ros2 run rby1_examples 16_target_shuttle_publisher --ros-args -p cycles:=1 \
    -p target_topic:=/rby1/left_arm/target_pose -p status_topic:=/rby1/left_arm/target_status \
    -p "point_a:=[0.463,0.367,1.166,1.571,-1.071,-1.571]" -p "point_b:=[0.413,0.367,1.116,1.571,-1.071,-1.571]"
```

**Step 3 — Put an obstacle in.** A 4 cm box where the wrist would be with the hand 15 cm further forward: the target
is refused and the robot does not move. Without the box the same target works.

```bash
ros2 run rby1_moveit_objects scene add box wall --xyz 0.437 -0.367 1.116 --size 0.04 0.04 0.04
ros2 run rby1_examples 16_target_shuttle_publisher --ros-args -p cycles:=1 \
    -p "point_a:=[0.563,-0.367,1.116,-1.571,-1.071,1.571]" \
    -p "point_b:=[0.413,-0.367,1.116,-1.571,-1.071,1.571]"                  # point_a is answered FAILED: planning failed ...
ros2 run rby1_moveit_objects scene remove wall
ros2 run rby1_examples 16_target_shuttle_publisher --ros-args -p cycles:=1 \
    -p "point_a:=[0.563,-0.367,1.116,-1.571,-1.071,1.571]" \
    -p "point_b:=[0.413,-0.367,1.116,-1.571,-1.071,1.571]"                  # point_a is answered DONE
```

`scene list` shows what is in the scene, `scene clear` empties it, and `scene add sphere|cylinder`, `scene move` and
`scene load <file>` are described in Dev_page.md.

A prepared set of obstacles goes in with one launch: it loads a YAML file of them and ends, and they stay in the scene.
The default file has a table, a post and a ball.

```bash
ros2 launch rby1_moveit_objects scene.launch.py                         # config/example_scene.yaml
ros2 launch rby1_moveit_objects scene.launch.py config:=/path/to/my_scene.yaml
ros2 run rby1_moveit_objects scene clear
```

An obstacle can also be kept where a topic says. `point_object.launch.py` starts a node that puts one object — a 6 cm
box as shipped, described in `config/point_object.yaml` — at every point it receives on `/rby1/scene/object_point`
(`geometry_msgs/PointStamped`; an empty `frame_id` means `base`), in place of where it was before.

```bash
ros2 launch rby1_moveit_objects point_object.launch.py                  # config/point_object.yaml
ros2 topic pub --once /rby1/scene/object_point geometry_msgs/msg/PointStamped \
    "{header: {frame_id: base}, point: {x: 0.413, y: -0.367, z: 1.266}}"
ros2 run rby1_moveit_objects scene remove point_object                  # it stays in the scene until removed
```

**Step 4 — Shuttle between two points, around an obstacle.** The hand goes back and forth between `point_a` (the
ready pose) and `point_b` (30 cm above it), three round trips; give your own as x, y, z, roll, pitch, yaw. Put a box
midway between the two points: with it in the scene, every leg is planned around it.

```bash
ros2 run rby1_examples 16_target_shuttle_publisher
ros2 run rby1_moveit_objects scene add box shuttle_box --xyz 0.413 -0.367 1.266 --size 0.06 0.06 0.06
ros2 run rby1_examples 16_target_shuttle_publisher --ros-args -p cycles:=2 -p period:=8.0   # longer legs: around the box
ros2 run rby1_moveit_objects scene remove shuttle_box
ros2 run rby1_examples 16_target_shuttle_publisher --ros-args \
    -p "point_a:=[0.413,-0.367,1.116,-1.571,-1.071,1.571]" -p "point_b:=[0.413,-0.167,1.216,-1.571,-1.071,1.571]"
```

Every target is answered `DONE` or `FAILED: <reason>` on `/rby1/right_arm/target_status`; the example logs the answers
and sends the next target either way. It does not wait for `DONE` and does not check where the hand ended.

**Step 5 — Put modules on the robot.** What is mounted on the robot is given in a YAML file — the frame (a link such
as `ee_right`, or `base`), the pose, and a mesh file or a box, sphere or cylinder — and is planned with while the node
runs; Ctrl+C takes it away. The default file has a camera on the head, a rod in the right hand and a workbench.

```bash
ros2 launch rby1_moveit_objects objects.launch.py                       # config/objects.yaml
ros2 launch rby1_moveit_objects objects.launch.py config:=/path/to/my_objects.yaml
ros2 run rby1_examples 16_target_shuttle_publisher                      # the rod is kept clear of things, too
```

**Step 6 — Use markers a camera sees.** The camera publisher gives the images and the camera's place on the robot. A
marker detector is not part of this repository: any node works that publishes each marker as
`/rby1/marker_<id>/pose` (`geometry_msgs/PoseStamped` in the camera's optical frame) and as TF `target_marker_<id>`.

```bash
ros2 launch rby1_additional_tools camera.launch.py                      # webcam, config/webcam.yaml
ros2 launch rby1_additional_tools camera.launch.py camera:=realsense    # needs: sudo apt install ros-humble-librealsense2
ros2 launch rby1_additional_tools camera.launch.py rviz:=true           # with RViz showing the image
```

The RB-Y1 Isaac ROS repository has such a detector (`rby1_apriltag`), and with it the two things that keep following a
marker: `marker_target.launch.py` sends the point under a marker as targets on `/rby1/right_arm/target_pose` (marker 7)
and `/rby1/left_arm/target_pose` (marker 8), so the planner started in step 1 moves the hand there, and
`follow_head:=true` turns the head to keep the marker in the middle of the image.

**Without a planner — `15_cartesian_target_move`.** This example is a target executor too: it takes the same targets
from the same topic and answers on the same status topic. But it has no planner: the driver solves for the arm and
takes the hand straight to each target, and **nothing is checked for collisions**. It needs the driver only, with the
robot powered, its servos on and the arm bent. The planner's launch leaves the robot so, but **stop that launch
(Ctrl+C) before starting the example**: two executors on one topic would both move the arm, and the example refuses to
run while the target topic already has a subscriber.

```bash
ros2 run rby1_examples 15_cartesian_target_move                              # shows READY, then serves targets until Ctrl+C
ros2 run rby1_examples 16_target_shuttle_publisher                           # another terminal: the ready pose and 30 cm above it, in turn
ros2 topic echo /rby1/right_arm/target_status std_msgs/msg/String            # a third terminal: EXECUTING …, DONE

ros2 run rby1_examples 15_cartesian_target_move --ros-args -p mode:=stream   # instead: a stream of small steps, up to 10 cm/s
ros2 run rby1_examples 16_target_shuttle_publisher --ros-args -p period:=8.0
```

Each target is answered `EXECUTING …`, then `DONE` when the hand ends within `tolerance` (5 mm) of it, else
`FAILED: <reason and what to do>`. `mode:=command` sends one `robot_cartesian` command that takes `duration` seconds
(3); `mode:=stream` opens the driver's `arm` stream channel for a move, sends `stream_cartesian` steps that speed up to
`max_speed`, and closes the channel after it. A target that arrives during a move runs when the move ends, and only the
latest one is kept. For the left arm, give both examples its topics
(`-p target_topic:=/rby1/left_arm/target_pose -p status_topic:=/rby1/left_arm/target_status`) and the publisher points
on the left side.

Every parameter of an example is listed at the top of its file. So are the values of the other examples (06–14):
the postures, times and speeds an example uses are constants in one block under its imports
(`# Values to change: …`), to be edited there.
