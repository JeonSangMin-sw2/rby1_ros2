# RBY1 ROS 2 Driver Package

> [!CAUTION]
> ## The current driver is in beta. For safe use, please test the features in a simulation first.

## Overview

`rby1_ros2` is a unified ROS 2 driver package for controlling the Rainbow Robotics RBY1 robot.  
It wraps the RBY1 C++ SDK into a ROS 2 node, providing state monitoring and multiple control modes (Joint Position, Cartesian Position, Impedance, Gravity Compensation, and Trajectory Streaming) through a clean action/service/topic interface.

- **ROS 2 version**: Humble
- **OS**: Ubuntu 22.04
- **SDK compatibility**: rby1-sdk `0.10.x` and later

---

## 1. Quick Start

- **If you install in an environment such as conda or miniforge, issues may arise due to Python and CMake path conflicts, so please install it in a local environment.**

### 1-1. Install ROS 2 Humble

<https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

### 1-2. Install RBY1 Simulator (Docker)

<https://hub.docker.com/r/rainbowroboticsofficial/rby1-sim>

### 1-3. Environment Setup

Add the following lines to `~/.bashrc`:

```bash
sudo nano ~/.bashrc

# Add at the bottom:
export PATH=/opt/cmake/bin:$PATH
source /opt/ros/humble/setup.bash

# Apply changes
source ~/.bashrc
```

### 1-4. Build

```bash
mkdir -p rby1_ros2_ws/src
cd rby1_ros2_ws/src
git clone https://github.com/RainbowRobotics/rby1_ros2.git
cd ..
colcon build --symlink-install
source install/setup.bash
```

### 1-5. Configure `driver_parameters.yaml`

Located at `rby1_driver/config/driver_parameters.yaml`.  
Edit this file to match your robot before launching the driver.  
Because the workspace was built with `--symlink-install`, **no rebuild is needed** after editing.

> [!IMPORTANT]
> For simulation testing, keep `robot_ip: "127.0.0.1:50051"`.  
> Some state values (battery, tool flange FT/IMU) will show zeros in simulation because no physical sensors are attached.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_ip` | `"127.0.0.1:50051"` | Robot IP address and gRPC port |
| `model` | `"m"` | Robot model — `"a"` (RBY1-A) or `"m"` (RBY1-M) |
| `state_topic_name` | `"joint_states"` | Fixed default namespace prefix for all state topics |
| `joint_position_topic_name` | `"robot_joint"` | Fixed default action server name for joint position commands |
| `cartesian_position_topic_name` | `"robot_cartesian"` | Fixed default action server name for Cartesian commands |
| `get_state_period` | `0.01` | State publish interval (seconds) — default 100 Hz |
| `minimum_time` | `2.0` | Default minimum execution time for motion commands (seconds) |
| `angular_velocity_limit` | `4.712` | Joint angular velocity limit (rad/s) |
| `linear_velocity_limit` | `1.5` | Cartesian linear velocity limit (m/s) |
| `acceleration_limit` | `1.0` | Acceleration scaling factor |
| `se2_minimum_time` | `1.0` | Minimum execution time (interpolation ramp) for SE2 velocity commands (seconds) |
| `se2_linear_acceleration_limit` | `0.5` | Linear acceleration limit for SE2 velocity commands |
| `se2_angular_acceleration_limit` | `0.5` | Angular acceleration limit for SE2 velocity commands |
| `fault_reset_trigger` | `true` | Auto-reset MAJOR/MINOR fault on driver startup |
| `node_power_off_trigger` | `false` | Power off robot automatically when driver node exits |
| `collision_enable` | `false` | Enable self-collision detection |
| `collision_threshold` | `0.01` | Collision detection distance threshold (meters) |
| `publish_battery_state` | `true` | Enable battery state topic |
| `publish_tool_flange_state` | `true` | Enable tool flange state topics (left + right) |

---

### 1-6. Run Simulator (optional)

If you do not have a physical robot, run the Docker simulator.  
The robot IP in this case is `"127.0.0.1:50051"` or `"localhost:50051"`.  
Change the tag at the end to select a model/version (e.g. `a_v1.2`, `m_v1.3`).

```bash
# Example: Model A, firmware v1.2
sudo docker run --rm \
  -e DISPLAY=${DISPLAY} \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -p 50051:50051 \
  rainbowroboticsofficial/rby1-sim:0.10.6-a_v1.2
```

> [!IMPORTANT]
> ## Model `a` only supports firmware up to v1.2. Model `m` supports v1.0–v1.3.

---

### 1-7. Launch the Driver

```bash
# In your workspace root
source install/setup.bash

# Option A: Launch normally
ros2 launch rby1_driver rby1_ros2_driver.launch.py

# Option B: Launch with a custom namespace (topic names remain relative)
ros2 launch rby1_driver rby1_ros2_driver.launch.py namespace:=my_robot
```

### 1-8. Run Examples

Each example can be run in a **separate terminal** while the driver is active:
```bash
source install/setup.bash
ros2 run rby1_examples <example_name>
```

| Example | Command | Description |
|---------|---------|-------------|
| `01_power_control` | `ros2 run rby1_examples 01_power_control` | Full power lifecycle: Power ON/OFF, Servo ON/OFF |
| `02_brake_control` | `ros2 run rby1_examples 02_brake_control` | Releases and re-engages arm brakes via IDLE state |
| `03_robot_status_monitor` | `ros2 run rby1_examples 03_robot_status_monitor` | Comprehensive state monitor (CM state, brakes, battery, FT) |
| `04_tool_flange_monitoring` | `ros2 run rby1_examples 04_tool_flange_monitoring` | Continuously prints tool flange FT/IMU/IO data |
| `05_joint_state_monitoring` | `ros2 run rby1_examples 05_joint_state_monitoring` | Prints per-component joint positions in real time |
| `06_gravity_compensation` | `ros2 run rby1_examples 06_gravity_compensation` | Enables gravity compensation (direct teaching) mode |
| `08_zero_pose` | `ros2 run rby1_examples 08_zero_pose` | Moves all joints to 0 rad simultaneously |
| `09_joint_command` | `ros2 run rby1_examples 09_joint_command` | Sends Ready Pose → Zero Pose via joint position action |
| `10_cartesian_command` | `ros2 run rby1_examples 10_cartesian_command` | Moves the right arm to a target Cartesian pose |
| `11_multi_controls` | `ros2 run rby1_examples 11_multi_controls` | Simultaneous joint + Cartesian control per body part |
| `12_trajectory_joint_command` | `ros2 run rby1_examples 12_trajectory_joint_command` | Streams a pre-computed trajectory via persistent command streams |
| `13_cancel_control` | `ros2 run rby1_examples 13_cancel_control` | Demonstrates action cancel and Trigger service cancel |
| `14_mobile_base_control` | `ros2 run rby1_examples 14_mobile_base_control` | Drives robot wheels via relative cmd_vel Twists |
| `15_stream_command` | `ros2 run rby1_examples 15_stream_command` | Alternates Zero/Ready poses using regular joint commands over persistent stream with varying wait intervals |

---

## 2. Package Structure

| Package | Role |
|---------|------|
| `rby1_driver` | C++ main driver node. Wraps the RBY1 SDK and exposes a ROS 2 interface. |
| `rby1_msgs` | Custom message, service, and action definitions for robot control and state. |
| `rby1_examples` | Python example scripts demonstrating all major driver features. |
|`rby1_description` | Robot description for ROS, demonstrating URDF and Mesh files, and simple visualization launch file. |

---

## 3. System Architecture

![driver](Doc/img/driver.png)

```
[User Node / Example]
        │
   ROS 2 Topics / Services / Actions
        │
  ┌─────▼────────────────────┐
  │   rby1_ros2_driver (C++) │
  │  ┌────────────────────┐  │
  │  │   State Publisher  │  │  ← reads robot state via SDK, publishes ROS topics
  │  ├────────────────────┤  │
  │  │  Service Handlers  │  │  ← power, servo, brake, gravity comp, CM commands
  │  ├────────────────────┤  │
  │  │  Action Servers    │  │  ← joint commands, Cartesian commands, streaming
  │  └────────────────────┘  │
  └─────────────────┬────────┘
                    │ gRPC
             ┌──────▼──────┐
             │  RBY1 Robot │
             │  (or Sim)   │
             └─────────────┘
```

**Key internal components:**
- **State Loop**: Calls `GetState()` and `GetControlManagerState()` at `get_state_period` intervals and publishes all state topics.
- **Action Executors**: Each action goal is translated into an SDK `CommandBuilder` command and executed synchronously. Minor faults during execution trigger automatic reset and recovery.
- **Safety Guard**: All motion commands are rejected unless the Control Manager is in `ENABLE` or `EXECUTING` state.
- **Collision Detection**: When `collision_enable: true`, self-collision is monitored and `CancelControl()` is called automatically if distance falls below `collision_threshold`.

---

## 4. Control Manager States

The `RobotState.control_manager_state` field (and the `robot_state` topic) uses the following integer constants, also accessible as `RobotState.STATE_*`:

| Value | Constant | Description |
|-------|----------|-------------|
| `0` | `STATE_NONE` | Driver not initialized or disconnected |
| `1` | `STATE_IDLE` | Control Manager is disabled (IDLE). Safe for brake operations. |
| `2` | `STATE_ENABLE` | Control Manager is active and holding position |
| `3` | `STATE_EXECUTING` | A motion command is currently being executed |
| `4` | `STATE_MAJOR_FAULT` | Unrecoverable hardware fault — requires reset |
| `5` | `STATE_MINOR_FAULT` | Recoverable fault — driver auto-resets by default |

---

## 5. Communication Interfaces

### 5-1. Topics (Publishers)

| Topic | Type | Always Active | Description |
|-------|------|:---:|-------------|
| `joint_states/torso` | `sensor_msgs/JointState` | ✅ | Torso joint positions, velocities, torques |
| `joint_states/right_arm` | `sensor_msgs/JointState` | ✅ | Right arm joint state |
| `joint_states/left_arm` | `sensor_msgs/JointState` | ✅ | Left arm joint state |
| `joint_states/head` | `sensor_msgs/JointState` | ✅ | Head joint state |
| `robot_state` | `rby1_msgs/RobotState` | ✅ | Control Manager state, brakes, EMO, CoM, tool flange connection |
| `battery_state` | `sensor_msgs/BatteryState` | ⚙️ `publish_battery_state` | Battery voltage, current, percentage |
| `tool_flange/left` | `rby1_msgs/ToolFlangeState` | ⚙️ `publish_tool_flange_state` | Left flange: FT sensor, IMU, switch, voltage, digital I/O |
| `tool_flange/right` | `rby1_msgs/ToolFlangeState` | ⚙️ `publish_tool_flange_state` | Right flange: FT sensor, IMU, switch, voltage, digital I/O |
| `odom` | `nav_msgs/Odometry` | ✅ | High-rate robot odometry and TF broadcast relative to node namespace |

### 5-2. Topics (Subscribers)

| Topic | Type | Description |
|-------|------|-------------|
| `cmd_vel` | `geometry_msgs/Twist` | Velocity command for driving base wheels (linear x, y and angular z) |

> [!IMPORTANT]
> **Mobile Base Control (`cmd_vel`) streaming requirement:**
> Since `cmd_vel` acts as a high-frequency publisher, **you must enable persistent stream control** before publishing base velocity commands.
> - Call `/stream_control` with `state: true` before sending `cmd_vel` commands.
> - Call `/stream_control` with `state: false` after finishing base control to return to regular position hold.
> - Attempting to activate `/stream_control` when already active is idempotent; the service will safely log a warning and return success.
>
> ⚙️ = controlled by the corresponding flag in `driver_parameters.yaml`

---

### 5-3. Services

| Service | Type | Description |
|---------|------|-------------|
| `robot_power` | `rby1_msgs/StateOnOff` | Power ON/OFF. `parameters`: `"all"`, `"48v"`, `"5v"`, etc. |
| `robot_servo` | `rby1_msgs/StateOnOff` | Servo ON/OFF. `parameters`: `"all"`, joint/part names |
| `tool_flange_power` | `rby1_msgs/StateOnOff` | Set tool flange voltage. `parameters`: `"12v"`, `"24v"`, `"48v"` (ON) or `""` (OFF) |
| `gravity_compensation` | `rby1_msgs/GravityCompensation` | Enable/disable gravity compensation per body part |
| `cancel_control` | `std_srvs/Trigger` | Cancel all active motion commands immediately |
| `get_cartesian_pose` | `rby1_msgs/GetCartesianPose` | Query Cartesian transform between two links |
| `control_manager_command` | `rby1_msgs/ControlManagerCommand` | Send `CMD_ENABLE` / `CMD_DISABLE` / `CMD_RESET` to the Control Manager |
| `set_motor_brake` | `rby1_msgs/StateOnOff` | Engage (`state=true`) or release (`state=false`) a joint brake. `parameters`: joint name (e.g. `"right_arm_3"`). Only available in `STATE_IDLE`. |
| `stream_control` | `rby1_msgs/StateOnOff` | Enable/disable persistent streaming mode with 10-minute hold times (`state=true` to enable, `state=false` to disable) |

#### `ControlManagerCommand` constants

| Constant | Value | Description |
|----------|-------|-------------|
| `CMD_NONE` | `0` | No operation |
| `CMD_ENABLE` | `1` | Enable the Control Manager (start position hold) |
| `CMD_DISABLE` | `2` | Disable the Control Manager (transition to IDLE) |
| `CMD_RESET` | `3` | Reset MAJOR/MINOR fault and return to IDLE |

---

### 5-3. Action Servers

| Action Server | Type | Description |
|---------------|------|-------------|
| `robot_joint` | `rby1_msgs/Rby1JointCommand` | Whole-body joint position command. Each body part (torso, right_arm, left_arm, head) can be commanded independently in a single goal. |
| `robot_cartesian` | `rby1_msgs/Rby1CartesianCommand` | Whole-body Cartesian command. Each arm and torso can be assigned an . geometry_msgs/msg/Transform.msg (position, quaternion)|
| `stream_position_command` | `rby1_msgs/StreamPosition` | Streams a full `JointTrajectory` (multi-waypoint) to the robot. |

> [!IMPORTANT]
> The action server names (`robot_joint`, `robot_cartesian`) are configured via `joint_position_topic_name` and `cartesian_position_topic_name` in `driver_parameters.yaml`.

---

## 6. Custom Message Types

### `rby1_msgs/JointCommand` (used inside `Rby1JointCommand` goals)

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `joint_names` | `string[]` | — | Optional joint name list |
| `position` | `float64[]` | — | Target joint positions (rad) |
| `minimum_time` | `float64` | `2.0` | Minimum execution time (s) |
| `velocity_limit` | `float64` | `4.7` | Joint velocity limit (rad/s) |
| `acceleration_limit` | `float64` | `1.0` | Acceleration scaling |
| `use_impedance` | `bool` | `false` | Use joint impedance instead of position control |
| `stiffness` | `float64[]` | — | Impedance stiffness coefficients |
| `damping_ratio` | `float64` | `1.0` | Impedance damping ratio |
| `torque_limit` | `float64` | `10.0` | Impedance torque safety limit (N·m) |

### `rby1_msgs/CartesianCommand` (used inside `Rby1CartesianCommand` goals)

| Field | Type | Description |
|-------|------|-------------|
| `target_link` | `string` | Name of the end-effector link to control |
| `ref_link` | `string` | Reference coordinate frame link |
| `transform` | `float64[16]` | Row-major 4×4 homogeneous transform matrix |
| `minimum_time` | `float64` | Minimum execution time (s) |
| `use_impedance` | `bool` | Use Cartesian impedance instead of position control |

### `rby1_msgs/RobotState`

| Field | Type | Description |
|-------|------|-------------|
| `control_manager_state` | `int32` | Current Control Manager state (see constants above) |
| `brake_state` | `BrakeState` | Brake engagement per joint (left_arm[], right_arm[], torso[], head[]) |
| `tool_flange_state` | `bool[]` | Tool flange connection status `[left, right]` |
| `emo_state` | `bool` | Emergency Stop pressed status |
| `center_of_mass` | `float64[3]` | Calculated CoM position `[x, y, z]` in meters |

### `rby1_msgs/ToolFlangeState`

| Field | Type | Description |
|-------|------|-------------|
| `ft_force` | `float64[3]` | Force `[Fx, Fy, Fz]` in Newtons |
| `ft_torque` | `float64[3]` | Torque `[Tx, Ty, Tz]` in N·m |
| `gyro` | `float64[3]` | Gyroscope `[roll, pitch, yaw]` in rad/s |
| `acceleration` | `float64[3]` | Accelerometer `[ax, ay, az]` in m/s² |
| `switch_a` | `bool` | Physical switch A state |
| `output_voltage` | `int32` | Output voltage in millivolts |
| `digital_input_a/b` | `bool` | Digital input A/B state |
| `digital_output_a/b` | `bool` | Digital output A/B state |

---

## 7. Key Features

### Robot Control
- **Joint Position Control**: Command each body part (Torso, Right/Left Arm, Head) to target joint angles (rad) via the `robot_joint` action. All parts can be commanded simultaneously in one goal.
- **Cartesian Position Control**: Command end-effector pose as a 4×4 SE3 transform via the `robot_cartesian` action.
- **Impedance Control**: Both joint and Cartesian modes support impedance control with configurable stiffness and damping.
- **Gravity Compensation**: Enables back-drivable joints for direct teaching; the driver continuously compensates gravity.
- **Trajectory Streaming**: Send a pre-computed `JointTrajectory` (multi-waypoint) via the `stream_position_command` action.

### State Monitoring
- Joint states (position, velocity, torque) are published at up to 100 Hz per body part.
- A unified `robot_state` topic provides Control Manager state, brake status, EMO, and CoM in one message.
- Optional battery state and per-flange FT/IMU data can be enabled in `driver_parameters.yaml`.

### Safety & Fault Management
- Motion commands are rejected if the Control Manager is not in `ENABLE` or `EXECUTING` state.
- Minor faults encountered during execution are automatically reset and control is resumed.
- Self-collision detection (optional): automatically calls `CancelControl()` when link distance falls below `collision_threshold`.
- Brake operations are only allowed in `STATE_IDLE` to prevent mechanical damage.

---

## 8. Notes & Known Limitations

- **Simulator**: Battery voltage, FT sensor, and IMU data read as `0.0` in simulation (no physical hardware).
- **Tool flange topics** require `publish_tool_flange_state: true` in `driver_parameters.yaml`.
- **Brake control** requires the Control Manager to be in `STATE_IDLE`. As an extra mechanical safety measure, brake commands are strictly rejected if 48V power is active (to prevent disengaging/engaging mechanical brakes while joints are powered). The `brake_control` example handles these states automatically.
- The `trajectory_joint_command` example currently uses `StreamPosition` only.

## 9. rby1_description

-  You can use the robot's basic TF structure and state publisher through the commands below. When implementing features related to rby1, please use the model files from the corresponding package.

- parameter
  - model_name :rby1a , rby1m 
  - model_version
    - rby1a : 1.0, 1.1, 1.2
    - rby1m : 1.0, 1.1, 1.2, 1.3

```bash
source install/setup.bash
ros2 launch rby1_description rby1_state_publisher.launch.py model:=a version:=1_1

```
1. if you launch this command, you can see the following window

![rby1_state_publisher_1](Doc/img/state_checker_guide_1.png)

2. click 'Add', and add plugin `TF`,`RobotModel`

![rby1_state_publisher_2](Doc/img/state_checker_guide_2.png)

3. click Fixed Frame and set to `base`

![rby1_state_publisher_3](Doc/img/state_checker_guide_3.png)

4. click RobotModel, and select Topics->`/robot_description`

![rby1_state_publisher_4](Doc/img/state_checker_guide_4.png)

5. you can now control robot model by use joint state publisher gui

![rby1_state_publisher_5](Doc/img/state_checker_guide_5.png)

---

## 10. Troubleshooting & Known Issues

### Issue: Control Commands Rejected After Trajectory Stream Interruptions (스트림 제어 노드 급작 종료 시 제어 불가 현상)

* **Symptom (증상)**: 
  If a stream-based trajectory control node (e.g., using persistent trajectory streams) is suddenly terminated or killed mid-operation, the driver's stream state remains active. Until this stream mode is explicitly closed, the driver will reject all other incoming joint or Cartesian motion commands, resulting in errors.
  
  스트림 통신(궤적 스트리밍 등)을 수행하던 중 노드가 갑자기 강제 종료(중단)된 경우, 드라이버 측에서는 스트림이 계속 동작 중인 것으로 간주하여 스트림이 유지됩니다. 이 스트림 모드를 끄기 전까지는 다른 일반 제어 명령이 모두 거부되며 오류가 발생합니다.

* **Resolution (해결 방법)**: 
  You must manually disable the streaming state by calling the `/stream_control` service with `state: false` in a separate terminal. This terminates the lingering stream and restores normal control capabilities.
  
  별도의 터미널을 열고 아래의 서비스를 호출하여 스트림 제어 모드를 강제로 비활성화(`false` 전송)한 후 정상적으로 제어 명령을 다시 실행해 주시기 바랍니다:

  ```bash
  ros2 service call /stream_control rby1_msgs/srv/StateOnOff "{state: false}"
  ```