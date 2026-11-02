# Architecture: upstream `RainbowRobotics/rby1-ros2` → this repository (2026-10-30)

Compared with the public repository (`main`, read 2026-10-30): its packages are `rby1_driver` (two source files),
`rby1_msgs`, `rby1_examples` (01–14), `rby1_description`, `rby1_hardware`, `rby1_moveit/rby1_moveit_{a,m}_*` (7 MoveIt
configurations with `demo.launch.py`) and `rby1_navigation`; its documentation is the README and one controller guide.

## Before

![before](img/architecture_before.png)

```mermaid
graph TD
    subgraph user ["User space"]
        Ex14["rby1_examples 01-14"]
        Rviz["RViz MotionPlanning panel"]
    end
    subgraph moveit ["rby1_moveit_<model>_<version> demo.launch.py"]
        MG["move_group"]
        CM["ros2_control controller manager"]
        HW["rby1_hardware::RBY1SystemHardware"]
    end
    subgraph drv ["rby1_driver (rby1_ros2_driver.cpp + main.cpp)"]
        D["driver node<br/>17 yaml parameters<br/>joint_states + joint_states/&lt;part&gt; x4<br/>tool_flange/&lt;side&gt;/status, imu, wrench<br/>4 stream channels: arm, torso, mobile, head<br/>one follow_joint_trajectory at a time"]
    end
    Robot["RB-Y1 / simulator"]
    Ex14 -->|services, actions| D
    Rviz --> MG --> CM --> HW
    HW -->|hardware_control, robot_power, robot_servo| D
    D -->|gRPC| Robot
    HW -->|gRPC stream| Robot
```

- The gripper: not in the URDFs (up to v1.2) and not in the scene; v1.3's URDF had it but no controller fed its joints.
- No planner driven by targets, no planning-scene tools, no camera or gripper tools, no integrated launch.
- Driver limits were yaml keys (`minimum_time`, velocity limits, `stop_*_tracking_error`, `get_state_period`, `fault_reset_trigger`).

## Now

![now](img/architecture_after.png)

```mermaid
%%{init: {'flowchart': {'nodeSpacing': 25, 'rankSpacing': 35, 'curve': 'basis'}}}%%
graph TD
    subgraph user ["User space"]
        Ex16["rby1_examples 01-16<br/>15 target executor, 16 target publisher"]
        Tgt["any node publishing /rby1/&lt;arm&gt;/target_pose"]
        Rviz["RViz"]
        subgraph tools ["rby1_tools"]
        CAM["camera.launch.py<br/>webcam / RealSense + camera_mount"]
        GR["gripper.launch.py<br/>model · scene · controller (virtual / real)<br/>gripper_mount"]
        end
    end
    subgraph plan ["MoveIt (one move_group per domain)"]
        EXE["rby1_moveit_executor<br/>targets → OMPL plan → follow_joint_trajectory"]
        OBJ["rby1_moveit_objects<br/>scene, objects.launch.py, object_at_point"]
        DEMO["rby1_moveit_* demo.launch.py"]
    end
    subgraph rc ["ros2_control"]
        CM["controller manager<br/>(demo.launch.py or rby1_hardware.launch.py)"]
        HW["rby1_hardware::RBY1SystemHardware<br/>claim retried · unmapped joints skipped<br/>ordered deactivation"]
    end
    D["rby1_driver: rby1_ros2_driver — source split by feature<br/>(power, state, control, streams, builder, kinematics)<br/>yaml DriverParameters + default_parameters.hpp<br/>joint_overall_states + joint_states (PartJointStates)<br/>RobotState.tool_flange_left/right<br/>5 stream channels: right_arm, left_arm, torso, mobile, head<br/>follow_joint_trajectory per part (concurrent / replace / reject stops all)<br/>static TF base_footprint→base · clean shutdown"]
    Robot["RB-Y1 / simulator"]
    Ex16 -->|services, actions| D
    Tgt --> EXE
    Ex16 --> Tgt
    EXE -->|follow_joint_trajectory| D
    OBJ -->|planning scene| EXE
    OBJ -.->|planning scene| DEMO
    GR -->|gripper parts, finger states| OBJ
    GR -->|/gripper/&lt;hand&gt;/robot_description, TF| Rviz
    CAM -->|markers → object_at_point| OBJ
    Rviz --> DEMO --> CM --> HW
    HW -->|hardware_control, robot_power, robot_servo| D
    D -->|gRPC| Robot
    HW -->|gRPC stream| Robot
```

## What changed, by package

| Area | Upstream | Now |
|---|---|---|
| Driver source | `rby1_ros2_driver.cpp` + `main.cpp` | Split by feature: `power.cpp`, `state_manager.cpp`, `control.cpp`, `control_stream.cpp`, `command_builder.cpp`, `kinematics.cpp`, included by `rby1_ros2_driver.cpp`; fixed values in `default_parameters.hpp` |
| Parameters | 17 yaml keys, limits and timing among them | `DriverParameters` struct, 16 keys: the six robot parameters written at connect, `control_manager_unlimited`; `get_state_period`, `minimum_time`, velocity/acceleration limits, `stop_*_tracking_error`, `fault_reset_trigger`, `joint_wheel` removed (fixed in code or taken from the message) |
| Joint state topics | `joint_states` + `joint_states/<part>` ×4 | `joint_overall_states` (standard, every joint) + `joint_states` (`PartJointStates`, per part) |
| Tool flange | `tool_flange/<side>/status` (`ToolFlangeState`), digital IO fields | In `RobotState.tool_flange_left/right` (`ToolFlange`: connected, output_voltage V, switch_a); IMU/wrench topics unchanged |
| Stream channels | 4: `arm` (both arms), `torso`, `mobile`, `head` | 5: `right_arm`, `left_arm`, `torso`, `mobile`, `head`; open/close by name, empty = all; parallel send per cycle; reopen gap |
| Trajectories | one `follow_joint_trajectory` at a time | per part: different parts concurrent, same part replaced, invalid goal rejected and every running one stopped |
| Shutdown / safety | streams closed | cancel → wait idle → disable control manager at shutdown, `CMD_DISABLE`, servo off; `kShutdown` abort on Ctrl+C |
| TF | `base_footprint→base` in the launch | published by the driver, model-based (A: −0.228 m) |
| Launch | `namespace` | + `params_file`, `robot_ip`, `model`; no `chrt` |
| Messages | `ToolFlangeState` | + `PartJointStates`, `ToolFlange`; `ToolFlangeStatus`/`ToolFlangeState` removed |
| Examples | 01–14 | + 15 (target executor without planner), 16 (target publisher); clean Ctrl+C; `grasp`/stream names updated |
| Description | URDFs end at tool frames (v1.3 included) | v1.3 restored with its gripper; others unchanged |
| Hardware plugin | — | unmapped joints (v1.3 fingers) skipped, claim retried, ordered deactivation, `rby1_hardware.launch.py` (ros2_control without MoveIt) |
| MoveIt | 7 `demo.launch.py` with `chrt` | + `rby1_moveit_executor` (target planner), `rby1_moveit_objects` (scene, modules, point object); `chrt` removed; Jazzy-ready `package.xml`, `joint_limits.yaml` and `rsp.launch.py` (URDF arguments reach the controller manager through `/robot_description`) |
| Tools | — | `rby1_tools`: camera publishers + mount; gripper model/scene/mount + virtual and real controller (`gripper.launch.py`) |
| Bringup | — | `rby1_bringup`: one launch with flags and one yaml, pre-flight checks |
| Docs | README + `Doc/moveit_controller_guide.md` | README + `Doc/manual/` 7 pages (Overview, Driver, Examples, Description, Hardware Interface, MoveIt, Tools) |
| Platforms | Humble | Humble + Jazzy (`cv_bridge` include, `rosdep` keys, `RBY1_SDK_PATH` check, acceleration limits) |
