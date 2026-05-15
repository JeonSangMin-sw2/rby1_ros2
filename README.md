# RBY1 ROS 2 Driver Package

## 개요
`rby1_ros2`는 Rainbow Robotics의 RBY1 로봇을 ROS 2 환경에서 제어하기 위한 통합 드라이버 패키지입니다. 이 패키지는 로봇의 상태 모니터링부터 다양한 제어 모드(Joint, Cartesian, Impedance 등)를 추상화된 인터페이스를 통해 제공합니다.


## 1.Quick start

### install ros2 humble

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

### install rb-y1 docker

https://hub.docker.com/r/rainbowroboticsofficial/rby1-sim

### additional setting[추가중]
```bash
sudo nano ~/.bashrc
# 내용 아래에 해당 커맨드 추가
export PATH=/opt/cmake/bin:$PATH
source /opt/ros/humble/setup.bash
alias cb='colcon build --symlink-install'
alias cbp='colcon build --symlink-install --packages-select'
alias si='source install/setup.bash'
```

### how to build
```bash
mkdir rby1_ros2_ws/src
cd rby1_ros2_ws/src
git clone https://github.com/RainbowRobotics/rby1_ros2.git
cd ..
cb
```

### set driver_parameters.yaml
- rby1_driver/config 에 있는 yaml 파일을 통해 로봇의 기본세팅을 할 수 있습니다.
- 해당 파일에서 연결할 로봇의 ip 와 model 을 선택할 수 있으며, 부부가적인 토픽구성, 기능 트리거를 사용할 수 있습니다.
- 처음 패키지 빌드당시 symlink 옵션으로 빌드했기에 해당 파일을 수정할 때마다 패키지 빌드를 할 필요는 없습니다.

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `robot_ip` | `"127.0.0.1:50051"` | 로봇 통신 IP 주소 및 포트 |
| `model` | `"a"` | 로봇 모델 (`"a"` 또는 `"m"`) |
| `state_topic_name` | `"joint_states"` | 상태 Publisher 및 Action Server의 기본 네임스페이스 |
| `get_state_period` | `0.01` | 로봇 상태 Publish 주기 (초) |
| `minimum_time` | `2.0` | 명령 수행 시 기본 최소 실행 시간 (초) |
| `acceleration_limit` | `1.0` | 로봇 가속도 제한 스케일링 |
| `collision_threshold` | `0.03` | 충돌 감지 임계값 (미터) |
| `publish_power_state` | `false` | 전원 상태 토픽 활성화 여부 |
| `publish_tool_flange` | `false` | 툴 플랜지 상태 토픽 활성화 여부 |
| `publish_torque_velocity` | `false` | 토크 및 속도 상태 토픽 활성화 여부 |
---
  
### Examples
`rby1_examples` 패키지에는 다양한 시나리오의 예제 코드가 포함되어 있습니다.
- 구동방식
```bash
ros2 run rby1_examples joint_control_example
```
| example | explain |
|---|---|
| cancel_control | 액션 cancel 명령으로 로봇제어 중지 + control_cancel 서비스 통신명령전송 |
| cartesian_control | 카타시안 제어기 |
| cartesian_impedance | 카타시안 임피던스 제어기 |
| joint_group| torso의 joint group 제어기 명령 |
| joint_impedance | 조인트 임피던스 명령 |
| multi_control | 각 파트마다 다른 제어기를 활용하여 제어 |
| multi_joint | arms,torso,head 모두를 포함한 명령전송 |
| power_control | power,servo on/off |
| robot_status_monitor | robot_info에서 확인 가능한 파라미터들을 모니터링 |
| single_joint | 단일 파트에 대한 조인트명령 |
| stream_joint_control | 스트림 형식의 제어명령. 명령보내면 joint trajectory를 기반으로 스트림 명령후 스트림 닫음 |
| tool_flange_test | 툴플렌지에서 확인되는 f/t센서 데이터 및 정보 모니터링 |
| zero_pose | 로봇 zero_pose로 명령 |
---

## 2.패키지 구성 및 역할
| 패키지 | 역할 |
|---|---|
| `rby1_driver` | C++ 기반의 메인 드라이버 노드. RBY1 SDK를 통해 실제 로봇과 통신하며 ROS 2 인터페이스를 제공합니다. |
| `rby1_msgs` | 로봇 제어 및 상태 확인에 필요한 커스텀 메시지, 서비스, 액션 정의를 포함합니다. |
| `rby1_examples` | 드라이버 기능을 활용하는 파이썬 기반의 다양한 제어 예제 코드를 제공합니다. |
---

### 2-1. rby1_driver
- 통신액션,서비스 선언
- 로봇 전원, 서보키는 서비스통신 진행 : 이미 켜져있으면 드라이버 내부에서 알아서 처리함. 안켜진거 있으면 그것만 on/off
- 사용할 제어기 설정 후 서비스통신을 통해 전송 : 기본값으로는 joint position제어로 되어있음
    - 제어기 설정시, 필요한 파라미터는 기입해줘야 함. 값이 누락되면 기본값으로 사용되도록 설정되어있음
    - [제어기종류 및 필요변수 매뉴얼에 추가할 예정]
- 하나의 파트(left_arm, right_arm, torso, head)만 액션으로 보내거나, 모든 데이터를 한번에 보내기 가능
   - 조인트 : 각도 rad 값
   - 카타시안 : 4*4행렬의 요소를 16크기의 배열로 나열한 행렬
- 제어기 사용 시 일반적으로 left_arm,right_arm,torso,head 로 나눠서 명명
- 따라서 각 조인트의 일부만을 제어하는 것은 불가능(torso 제외)

|제어기|사용방식|
|---|---|
|Joint position, Joint impedance|각 조인트의 rad 값|
|Cartesian position, Cartesian impedance|타겟 링크의 4*4행렬값(이를 16크기의 배열로 변환하여 전송)|
|Joint group|torso 의 선택된 joint 이름과 rad 값|
|Gravity compensation|파트의 on/off|
|stream|ros2의 trajectory 변수를 활용한 조인트 position의 경로값|
---
### 1. 로봇 제어 (Advanced Control)
- **Joint Control**: 각 부위별(Torso, Arm, Head) 또는 전신(Multi-joint)의 관절 위치를 개별적으로 제어할 수 있습니다.
- **Cartesian Control**: 4x4 변환 행렬을 16크기의 배열로 통신하며, 사용자가 정한 타켓 링크의 위치와 방향을 제어합니다.
- **Impedance Control**: 관절 및 카테시안 공간에서의 임피던스 제어를 지원합니다.
- **Gravity Compensation**: 중력 보상 모드를 통해 사용자가 로봇을 직접 손으로 움직여 가르칠 수 있는 티칭(Direct Teaching) 기능을 제공합니다.
- **Trajectory Streaming**: ros2에서 자주 사용되는 관절 궤적 데이터를 받아서 rb-y1의 스트림 형식으로 제어합니다.

### 2. 통합 상태 모니터링 (Comprehensive Monitoring)
- **States**: 100Hz 주기(사용자가 수정 가능)로 로봇이 제공하는 상태를 발행합니다.
- **Sensor Integration**: 6축 FT 센서 데이터, 툴 플랜지 IMU 데이터, 배터리 및 전원 상태를 실시간으로 확인할 수 있습니다.
- **Link Poses**: 내부 Dynamics 엔진을 활용하여 주요 링크의 카테시안 좌표를 실시간으로 계산하여 발행합니다.
- **Joint State**: 로봇의 각 관절들의 각도값을 받고, 옵션을 통해 전체 데이터가 통합된 하나의 토픽을 받을 수 있습니다.

### 3. 유연한 장치 관리 (Device Management)
- **Power & Servo Control**: 개별 부위 또는 전체 로봇의 전원 및 서보 상태를 서비스를 통해 제어합니다.
- **Tool Flange I/O**: 툴 플랜지의 출력 전압(12V/24V) 설정 및 디지털 입출력 상태 확인 기능을 제공합니다.
- **Fault Management**: 하드웨어 또는 제어 폴트 발생 시 소프트웨어적인 리셋 및 복구 기능을 지원합니다.



## 시스템 아키텍쳐
본 드라이버는 RBY1 SDK를 ROS 2 노드로 래핑(Wrapping)하는 구조로 설계되었습니다.

1.  **Driver Node**: SDK를 통해 실시간 로봇 상태(Joint State, Cartesian Pose, Sensor Data)를 읽어와 ROS Topic으로 발행합니다.
2.  **Command Builder**: SDK의 `CommandBuilder`를 활용하여 ROS Action 요청을 SDK 명령어로 변환하고 주입합니다.
3.  **State Machine**: 서비스(`set_control_mode`)를 통해 각 부위별 제어 방식(Position, Impedance 등)을 실시간으로 전환하며 동적인 제어를 수행합니다.

---

## 시스템 구성 및 주요 기능

-   **모듈형 상태 발행**: 각 부위(Torso, Right Arm, Left Arm, Head, Mobility)의 상태를 개별 토픽 또는 통합 토픽(`AllMotorState`)으로 선택적 발행 가능.
-   **다양한 제어 모드 지원**:
    -   Joint Position / Joint Impedance
    -   Cartesian Position / Cartesian Impedance
    -   Gravity Compensation (중력 보상)
    -   Joint Group Position (특정 관절 그룹 제어)
-   **고급 상태 모니터링**: 전원 상태, FT 센서 데이터, 툴 플랜지 IMU 및 스위치 입력 등을 실시간으로 Publish.
-   **안전 기능**: 자가 충돌 감지(Collision Detection) 시 자동 제어 취소 기능 지원.

---

## 통신 인터페이스 (Communication Interfaces)

## 5.토픽 (Topics - Publishers)

| 토픽 이름 | 자료형 (Message Type) | 설명 |
|---|---|---|
| `joint_states/torso` | `sensor_msgs/msg/JointState` | Torso 관절의 상태 (위치, 속도, 토크) |
| `joint_states/right_arm` | `sensor_msgs/msg/JointState` | 우측 팔 관절의 상태 |
| `joint_states/left_arm` | `sensor_msgs/msg/JointState` | 좌측 팔 관절의 상태 |
| `joint_states/head` | `sensor_msgs/msg/JointState` | 머리 관절의 상태 |
| `joint_states/torso_cartesian` | `rby1_msgs/msg/CartesianPose` | Torso 말단 장치의 카테시안 좌표 (4x4 Matrix) |
| `joint_states/right_cartesian` | `rby1_msgs/msg/CartesianPose` | 우측 팔 말단 장치의 카테시안 좌표 |
| `joint_states/left_cartesian` | `rby1_msgs/msg/CartesianPose` | 좌측 팔 말단 장치의 카테시안 좌표 |
| `joint_states/control_state` | `std_msgs/msg/Int32` | 로봇의 현재 제어 상태 (0:NONE, 1:ENABLE, 2:EXECUTING, 4:MINOR_FAULT, 5:MAJOR_FAULT, 6:IDLE) |
| `joint_states/power_state` | `rby1_msgs/msg/PowerState` | 배터리 전압, 전류, EMO 상태 등 전원 관련 정보 |
| `joint_states/tool_flange_state`| `rby1_msgs/msg/ToolFlangeState` | FT 센서 데이터, 툴 IMU, 스위치 및 IO 상태 |
| `joint_states/torque_velocity` | `rby1_msgs/msg/TorqueVelocityState` | 전체 관절의 속도 및 토크, COM 정보 |

### 서비스 (Services)

| 서비스 이름 | 자료형 (Service Type) | 설명 |
|---|---|---|
| `/robot_power` | `rby1_msgs/srv/StateOnOff` | 로봇 전원 On/Off (파라미터로 특정 부위 지정 가능) |
| `/robot_servo` | `rby1_msgs/srv/StateOnOff` | 관절 서보 On/Off |
| `/tool_flange_power` | `rby1_msgs/srv/StateOnOff` | 툴 플랜지 출력 전압 설정 (12V, 24V, 0V) |
| `/set_control_mode` | `rby1_msgs/srv/ControlMode` | 각 부위별 제어 모드 및 파라미터(Stiffness 등) 설정 |
| `/cancel_control` | `std_srvs/srv/Trigger` | 현재 수행 중인 모든 제어 명령 취소 및 정지 |

### 액션 서버 (Action Servers)

| 액션 이름 | 자료형 (Action Type) | 설명 |
|---|---|---|
| `joint_states/single_position_command` | `rby1_msgs/action/SingleJointCommand` | 특정 한 부위에 대한 위치/카테시안 명령 전달 |
| `joint_states/multi_position_command` | `rby1_msgs/action/MultiJointCommand` | 여러 부위(전신)에 대한 동기화된 명령 전달 |
| `joint_states/stream_position_command`| `rby1_msgs/action/StreamPosition` | 관절 궤적(Joint Trajectory) 스트리밍 제어 |

---

