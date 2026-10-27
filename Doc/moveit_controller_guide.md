# Rainbow Robotics RBY1 MoveIt 2 및 ros2_control 구성 가이드

이 문서는 `rby1_hardware` 및 `rby1_moveit_test` 패키지를 참고하여, Rainbow Robotics RBY1 로봇을 위한 MoveIt 2 패키지 설계, 컨트롤러(ros2_control/MoveIt) 작성, 엔드이펙터 정의, 그리고 패키지 로드 시 문제가 없도록 충돌 방지(SRDF)를 사전 정의하는 방법을 상세히 설명합니다.

---

## 1. MoveIt 2 패키지 구조 및 Xacro 설정
MoveIt 2 패키지(`rby1_moveit_test`)는 MoveIt Setup Assistant를 통해 뼈대를 생성한 후, 실제 하드웨어와의 연동을 위해 수정/보완합니다.

### 1.1 최상위 로봇 Xacro 구성 (`rby1.urdf.xacro`)
MoveIt은 로봇 설명(URDF)을 로드할 때 Xacro 매핑 및 인자를 처리합니다.
- **역할**: 로봇 원본 URDF 파일(e.g., `rby1_description`)을 포함하고, `ros2_control` 설정을 주입합니다.
- **예시 구성**:
  ```xml
  <?xml version="1.0"?>
  <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="rby1">
      <!-- 실행 인자 정의 -->
      <xacro:arg name="initial_positions_file" default="initial_positions.yaml" />
      <xacro:arg name="use_fake_hardware" default="true" />
      <xacro:arg name="robot_ip" default="127.0.0.1:50051" />
      <xacro:arg name="model" default="m" />
      <xacro:arg name="collision_check_enable" default="false" />
      <xacro:arg name="collision_threshold" default="0.01" />
      <xacro:arg name="model_name" default="rby1m" />
      <xacro:arg name="model_version" default="1_3" />

      <!-- 1. 로봇 원본 URDF 모델 포함 -->
      <xacro:include filename="$(find rby1_description)/urdf/$(arg model_name)/model_v$(arg model_version).urdf" />

      <!-- 2. ros2_control Xacro 파일 포함 -->
      <xacro:include filename="rby1.ros2_control.xacro" />

      <!-- 3. ros2_control 매크로 호출 및 파라미터 주입 -->
      <xacro:rby1_ros2_control
          name="RBY1System"
          initial_positions_file="$(arg initial_positions_file)"
          use_fake_hardware="$(arg use_fake_hardware)"
          robot_ip="$(arg robot_ip)"
          model="$(arg model)"
          collision_check_enable="$(arg collision_check_enable)"
          collision_threshold="$(arg collision_threshold)"
      />
  </robot>
  ```

### 1.2 ros2_control 설정 (`rby1.ros2_control.xacro`)
하드웨어 인터페이스와 컨트롤러 사이의 인터페이스를 정의합니다.
- **하드웨어 플러그인 분기**: `use_fake_hardware` 값에 따라 시뮬레이션용 가상 플러그인(`mock_components/GenericSystem`) 또는 실물 로봇 연동용 플러그인(`rby1_hardware/RBY1SystemHardware`)을 선택합니다.
- **인터페이스 정의**: 각 조인트별로 제어 명령 타입(`command_interface`) 및 상태 읽기 타입(`state_interface`)을 선언합니다. (주로 `position`과 `velocity`)
  ```xml
  <ros2_control name="${name}" type="system">
      <hardware>
          <xacro:if value="${use_fake_hardware}">
              <plugin>mock_components/GenericSystem</plugin>
          </xacro:if>
          <xacro:unless value="${use_fake_hardware}">
              <plugin>rby1_hardware/RBY1SystemHardware</plugin>
              <param name="robot_ip">${robot_ip}</param>
              <param name="model">${model}</param>
              <param name="collision_check_enable">${collision_check_enable}</param>
              <param name="collision_threshold">${collision_threshold}</param>
          </xacro:unless>
      </hardware>
      
      <!-- 조인트별 인터페이스 및 초기값 설정 -->
      <joint name="left_arm_0">
          <command_interface name="position"/>
          <command_interface name="velocity"/>
          <state_interface name="position">
              <param name="initial_value">${initial_positions['left_arm_0']}</param>
          </state_interface>
          <state_interface name="velocity"/>
      </joint>
      <!-- ... 다른 조인트들 정의 ... -->
  </ros2_control>
  ```

---

## 2. 컨트롤러 설정 (ros2_control vs MoveIt 2)
컨트롤러는 크게 **ros2_control 측 컨트롤러(실제 조인트를 구동)**와 **MoveIt 2 측 컨트롤러 관리자(경로 계획 결과를 ros2_control에 전달)**로 나뉩니다.

### 2.1 ros2_control 컨트롤러 (`ros2_controllers.yaml`)
실제 조인트 모터를 제어하는 드라이버 레벨의 컨트롤러 목록을 정의합니다.
- **JointStateBroadcaster**: 로봇의 현재 조인트 상태를 `/joint_states` 토픽으로 발행합니다.
- **JointTrajectoryController**: MoveIt이 보낸 궤적(Trajectory) 명령을 받아 부드럽게 조인트를 구동합니다.
- **설정 예시**:
  ```yaml
  controller_manager:
    ros__parameters:
      update_rate: 100  # Hz

      right_arm_controller:
        type: joint_trajectory_controller/JointTrajectoryController
      left_arm_controller:
        type: joint_trajectory_controller/JointTrajectoryController
      torso_controller:
        type: joint_trajectory_controller/JointTrajectoryController
      head_controller:
        type: joint_trajectory_controller/JointTrajectoryController
      joint_state_broadcaster:
        type: joint_state_broadcaster/JointStateBroadcaster

  right_arm_controller:
    ros__parameters:
      joints:
        - right_arm_0
        - right_arm_1
        - right_arm_2
        - right_arm_3
        - right_arm_4
        - right_arm_5
        - right_arm_6
      command_interfaces:
        - position
      state_interfaces:
        - position
  # left_arm_controller, torso_controller, head_controller도 동일하게 설정합니다.
  ```

### 2.2 MoveIt 컨트롤러 (`moveit_controllers.yaml`)
MoveIt이 조인트를 제어할 때 어떤 ros2_control 액션 서버를 호출해야 하는지 매핑합니다.
- **MoveItSimpleControllerManager** 플러그인을 사용하여 `FollowJointTrajectory` 타입의 액션을 지정합니다.
- **설정 예시**:
  ```yaml
  moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

  moveit_simple_controller_manager:
    controller_names:
      - right_arm_controller
      - left_arm_controller
      - torso_controller
      - head_controller

    right_arm_controller:
      type: FollowJointTrajectory
      action_ns: follow_joint_trajectory
      joints:
        - right_arm_0
        - right_arm_1
        - right_arm_2
        - right_arm_3
        - right_arm_4
        - right_arm_5
        - right_arm_6
    # left_arm_controller, torso_controller, head_controller 등도 동일한 포맷으로 매핑합니다.
  ```

---

## 3. 엔드이펙터 정의 (SRDF)
MoveIt에서 엔드이펙터(End-Effector)는 특정 기구학적 체인(Planning Group)의 끝에 장착되어 파지나 작업을 수행하는 부분을 식별하기 위해 정의합니다.

- **SRDF 설정 (`rby1.srdf`)**:
  `<end_effector>` 태그를 사용해 정의하며, 아래의 필수 속성들을 지정해야 합니다.
  - `name`: 엔드이펙터의 고유 식별 명칭.
  - `parent_link`: 엔드이펙터가 물리적으로 부착된 링크 (e.g., 로봇 팔의 마지막 링크).
  - `group`: 엔드이펙터 자체를 제어하기 위한 조인트/링크 그룹 (그리퍼 조인트가 포함된 그룹).
  - `parent_group`: 엔드이펙터를 움직여주는 상위 기구학 그룹 (e.g., 로봇 팔 그룹).

- **예시 구성**:
  ```xml
  <!-- 기구학 그룹(Planning Group) 정의 -->
  <group name="arm">
      <!-- 왼팔, 오른팔 조인트들 정의 -->
      <joint name="left_arm_0"/>
      <!-- ... 생략 ... -->
      <joint name="left_arm_6"/>
      <joint name="right_arm_0"/>
      <!-- ... 생략 ... -->
      <joint name="right_arm_6"/>
  </group>

  <!-- 엔드이펙터 선언 -->
  <end_effector name="arm_l_ef" parent_link="link_left_arm_6" group="arm" parent_group="arm"/>
  <end_effector name="arm_r_ef" parent_link="link_right_arm_6" group="arm" parent_group="arm"/>
  <end_effector name="body_ef" parent_link="link_torso_5" group="body" parent_group="body"/>
  ```

---

## 4. 자가 충돌 검사 제외 설정 (Pre-defined Collisions in SRDF)
MoveIt 2가 실행될 때 모든 링크 쌍 간의 충돌 검사를 수행하면 연산량이 기하급수적으로 늘어납니다. 특히, **물리적으로 붙어있는 인접 링크(Adjacent Links)**나 **절대 닿을 수 없는 링크(Never Collide)** 간에는 충돌 검사를 사전 정의하여 제외(Disable)해 놓아야 합니다.
그렇지 않으면 MoveIt 로드 시 자가 충돌(Self-Collision)로 인식되어 로봇이 시작부터 멈추거나 경로 계획이 무조건 실패하는 치명적인 문제가 발생합니다.

### 4.1 설정 방법 (`rby1.srdf` 내 `<disable_collisions>`)
- **형식**:
  ```xml
  <disable_collisions link1="링크_이름_1" link2="링크_이름_2" reason="제외_이유"/>
  ```
- **제외 이유(`reason`) 종류**:
  - `Adjacent`: 두 링크가 회전 조인트 등으로 직접 맞닿아 있음. (충돌 체크 시 항상 충돌로 오인됨)
  - `Never`: 두 링크의 가동 범위상 물리적으로 절대 만날 수 없음.
  - `Default`: 기본 상태에서 충돌하지 않음.

- **예시 구성**:
  ```xml
  <!-- 인접한 링크 간 충돌 검사 제외 (필수) -->
  <disable_collisions link1="base" link2="link_torso_0" reason="Adjacent"/>
  <disable_collisions link1="link_torso_0" link2="link_torso_1" reason="Adjacent"/>
  
  <!-- 팔의 기저부와 머리가 닿지 않거나 기구학 구조상 충돌할 일이 없는 경우 -->
  <disable_collisions link1="link_head_0" link2="link_left_arm_0" reason="Never"/>
  
  <!-- 센서와 인접 그리퍼 핑거 간 충돌 제외 -->
  <disable_collisions link1="FT_sensor_L" link2="ee_left" reason="Adjacent"/>
  <disable_collisions link1="FT_sensor_L" link2="ee_finger_l1" reason="Never"/>
  ```
- **팁**: MoveIt Setup Assistant의 **Self-Collisions** 탭을 이용하면 샘플 포즈 샘플링을 통해 자동으로 안전한 링크 쌍을 계산하고 `<disable_collisions>` 태그 목록을 생성해 줍니다. 수동으로 커스텀 그리퍼나 센서를 URDF에 추가했을 때는 해당 링크와 인접 링크 간의 `Adjacent` 관계를 SRDF에 수동으로 직접 기입해 주어야 정상 구동됩니다.

---

## 5. ros2_control 하드웨어 인터페이스 플러그인 작성 (`rby1_hardware`)
MoveIt이 계산한 궤적에 맞춰 실물 로봇을 구동하기 위해 `hardware_interface::SystemInterface`를 구현해야 합니다. `rby1_hardware`는 gRPC 스트림 통신을 사용하여 실물 로봇의 SDK와 인터페이스합니다.

### 5.1 핵심 라이프사이클 함수 구성
1. **`on_init()`**:
   - URDF로부터 IP 주소, 모델 타입(`a` 또는 `m`), 예측 충돌 검사 파라미터(`collision_check_enable`, `collision_threshold`) 등을 로드합니다.
   - 내부 상태/명령 버퍼를 할당하고 인터페이스 타입이 `position` 인지 검증합니다.
2. **`export_state_interfaces()` / `export_command_interfaces()`**:
   - ros2_control 내부 메모리 버퍼(`hw_positions_`, `hw_velocities_`, `hw_commands_`)를 외부에 노출하여 컨트롤러가 읽고 쓸 수 있도록 바인딩합니다.
3. **`on_activate()`**:
   - Rainbow Robotics gRPC SDK에 연결합니다 (`robot_->connect(robot_ip_)`).
   - `/hardware_control` 서비스를 통해 드라이버 노드로부터 하드웨어 제어권을 획득(Claim)합니다.
   - URDF 조인트 이름과 SDK 상의 조인트 인덱스를 매핑합니다.
   - 로봇 전원을 인가하고 서보 온(`servo_on`)을 실행한 후, SDK 내부 `ControlManager`를 활성화하고 실시간 제어 스트림(`init_stream`)을 엽니다.
4. **`read()`**:
   - 백그라운드에서 실시간 수신되는 로봇의 최신 조인트 상태 토픽 또는 SDK 상태를 바탕으로 `hw_positions_`와 `hw_velocities_` 버퍼를 최신화합니다.
5. **`write()`**:
   - `hw_commands_`에 채워진 MoveIt/ros2_control의 목표 값을 읽어 SDK 제어 명령으로 변환합니다.
   - **하드웨어 수준 자가 충돌 예방 (Safety)**:
     `collision_check_enable`이 활성화된 경우, SDK의 Dynamics 라이브러리(`ComputeForwardKinematics`, `DetectCollisionsOrNearestLinks`)를 활용해 목표 조인트 각도(`sdk_target_positions`)가 충돌을 일으키는지 사전에 예측 검사합니다. 충돌 위험 거리가 임계값(`collision_threshold`) 이하인 경우 **명령 송신을 차단하고 현재 위치로 긴급 정지**시킵니다.
   - 이상이 없으면 `send_stream_command()`를 통해 실시간 제어 명령을 보냅니다.
6. **`on_deactivate()`**:
   - 하드웨어 제어권을 반납하고, 실시간 스트림을 종료하며 SDK 연결을 안전하게 해제합니다.

### 5.2 플러그인 등록
- C++ 코드 최하단에 `pluginlib` 매크로를 선언해야 합니다.
  ```cpp
  #include "pluginlib/class_list_macros.hpp"
  PLUGINLIB_EXPORT_CLASS(rby1_hardware::RBY1SystemHardware, hardware_interface::SystemInterface)
  ```
- **`rby1_hardware.xml`**에 클래스를 선언하여 ROS 2 시스템이 플러그인으로 로드할 수 있도록 합니다.
  ```xml
  <library path="rby1_hardware">
    <class name="rby1_hardware/RBY1SystemHardware"
           type="rby1_hardware::RBY1SystemHardware"
           base_class_type="hardware_interface::SystemInterface">
      <description>ros2_control hardware interface for Rainbow Robotics RBY1.</description>
    </class>
  </library>
  ```
- **`CMakeLists.txt`**에서 플러그인 XML 파일을 내보내도록 설정합니다.
  ```cmake
  pluginlib_export_plugin_description_file(hardware_interface rby1_hardware.xml)
  ```

---

## 6. MoveIt 설정 패키지 내 주요 수정 대상 파일 및 수정 방법

MoveIt Setup Assistant로 뼈대 설정 패키지(예: `rby1_moveit_m_1_2`)를 생성한 후, 실물 로봇 드라이버와의 정상적인 연동 및 실행 시 발생하는 각종 문제를 해결하기 위해 **반드시 수정해야 하는 주요 설정 파일 목록과 방법**입니다.

### 6.1 `config/ros2_controllers.yaml` (중복 컨트롤러 활성화 충돌 방지)
- **수정 목적**: `both_arms_controller`나 `body_controller` 같이 개별 팔/토르소 조인트가 중복으로 겹치는 결합 컨트롤러가 동시에 활성화되면, `ros2_control`에서 동일 조인트 자원 점유 경쟁이 발생하여 `ros2_control_node`가 강제 종료(Crash)되거나 spawner가 서비스에 접근하지 못하는 오류가 발생합니다.
- **수정 방법**: 
  - `controller_manager` 파라미터 영역 아래의 활성 컨트롤러 목록 중, 중복되지 않는 독립적인 컨트롤러(`left_arm_controller`, `right_arm_controller`, `torso_controller`, `head_controller`)만 활성화하도록 설정합니다.
  - 예시:
    ```yaml
    controller_manager:
      ros__parameters:
        update_rate: 100  # Hz

        right_arm_controller:
          type: joint_trajectory_controller/JointTrajectoryController
        left_arm_controller:
          type: joint_trajectory_controller/JointTrajectoryController
        torso_controller:
          type: joint_trajectory_controller/JointTrajectoryController
        head_controller:
          type: joint_trajectory_controller/JointTrajectoryController
        joint_state_broadcaster:
          type: joint_state_broadcaster/JointStateBroadcaster
        # both_arms_controller나 body_controller 등 중복되는 컨트롤러는 활성 목록에서 제거하거나 주석 처리
    ```

### 6.2 `config/moveit_controllers.yaml` (MoveIt 액션 매핑 제한)
- **수정 목적**: MoveIt이 시작될 때 어떤 액션 컨트롤러를 활성화하고 추적할지 정의하는 파일입니다. 여기서도 중복되는 컨트롤러를 배제해야 `ros2_control`과의 불일치 크래시를 방지할 수 있습니다.
- **수정 방법**:
  - `moveit_simple_controller_manager` -> `controller_names` 리스트에 오직 독립적인 비중복 컨트롤러 이름만 등록합니다.
  - 예시:
    ```yaml
    moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

    moveit_simple_controller_manager:
      controller_names:
        - right_arm_controller
        - left_arm_controller
        - torso_controller
        - head_controller
      # both_arms_controller나 body_controller 등은 아래 세부 설정에는 남겨두어도 되지만 controller_names 목록에서는 제외합니다.
    ```

### 6.3 `config/<robot_model>.ros2_control.xacro` (그리퍼 조인트 제외 및 Xacro 경고 수정)
> [!NOTE]
> 2026-10-16부터 `rby1_description` URDF와 MoveIt 패키지에 그리퍼가 없습니다(그리퍼는 `rby1_moveit_objects`의 `gripper.yaml` 모듈).
> 아래 그리퍼 조인트 삭제는 그 전 URDF로 패키지를 만들 때의 절차입니다.

- **수정 목적**:
  1. **그리퍼 조인트 배제**: RBY1 SDK의 실시간 gRPC 26자유도 스트리밍 제어 대상 조인트에 그리퍼 핑거 조인트(`gripper_finger_l1` 등)는 포함되지 않습니다. 그리퍼는 드라이버 서비스/액션으로 별도 제어되므로, ros2_control 조인트 설정에 그리퍼가 포함되어 있으면 하드웨어 인터페이스 플러그인이 정상 작동하지 않습니다.
  2. **Xacro 경고 수정**: ROS 2 Humble 버전에서 `load_yaml` 사용 시 발생하는 Deprecation 경고를 제거합니다.
- **수정 방법**:
  - `<ros2_control>` 엘리먼트 내부의 모든 조인트 목록에서 `gripper_finger_r1`, `gripper_finger_r2`, `gripper_finger_l1`, `gripper_finger_l2` 와 같은 그리퍼 조인트 정의 블록을 통째로 삭제합니다.
  - 초기값 YAML 로딩 함수 호출부를 `${load_yaml(...)}`에서 `${xacro.load_yaml(...)}`로 수정합니다.
  - 예시:
    ```xml
    <!-- load_yaml 경고 해결 -->
    <xacro:property name="initial_positions" value="${xacro.load_yaml(initial_positions_file)['initial_positions']}"/>
    ```

### 6.4 `config/joint_limits.yaml` (가속도 한계 정의 경고 해결)
- **수정 목적**: MoveIt 궤적 생성기 실행 시 `Joint acceleration limits are not defined. Using the default 1 rad/s^2.` 경고 메세지가 출력되는 문제를 해결하고, 로봇 동작 시 부드럽고 안전하게 구동되도록 가속도 제한을 주입합니다.
- **수정 방법**:
  - `joint_limits` 내의 모든 26개 스트리밍 조인트마다 `has_velocity_limits: true`, `max_velocity` 값을 지정하고, 동시에 `has_acceleration_limits: true`, `max_acceleration` (예: 1.0) 값을 명시적으로 입력합니다.
  - 예시:
    ```yaml
    joint_limits:
      left_arm_0:
        has_velocity_limits: true
        max_velocity: 4.712388
        has_acceleration_limits: true
        max_acceleration: 1.0
      # ... 모든 스트리밍 조인트에 대해 동일하게 작성
    ```

### 6.5 `config/kinematics.yaml` (양팔 결합 그룹의 IK 솔버 설정 제거)
- **수정 목적**: KDL 등 일반적인 IK 솔버 플러그인은 일직선으로 이어진 단일 기구학 체인(Chain)에 대해서만 해석이 가능합니다. 양팔이 갈라지는 복합 기구학 그룹(`dual_arm` 등)에 IK 솔버를 매핑해 두면 Plan 실행 시 수치 해석 오류로 인해 기구학 계산 및 경로 계획이 무조건 실패하는 현상이 발생합니다.
- **수정 방법**:
  - `left_arm`, `right_arm`과 같이 일직선의 단일 체인으로 구성된 그룹에만 IK 솔버 플러그인을 정의하고, 양팔을 묶은 `dual_arm`이나 `body` 등의 그룹에 매핑된 `kinematics_solver` 설정은 완전히 삭제하거나 주석 처리합니다.
  - 예시:
    ```yaml
    left_arm:
      kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
      kinematics_solver_search_resolution: 0.005
      kinematics_solver_timeout: 0.005
    right_arm:
      kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
      kinematics_solver_search_resolution: 0.005
      kinematics_solver_timeout: 0.005
    # dual_arm 이나 body 그룹에 대한 kinematics_solver 설정은 주석 처리하여 비활성화
    ```

---

## 7. 추가 확인 사항 및 디버깅 팁 (Troubleshooting & Debug Notes)

로봇 패키지 구축 및 MoveIt 연동 과정에서 자주 발생할 수 있는 주요 오류 원인과 해결 방안입니다.

### 7.1 SDK 의존성 헤더 및 라이브러리 링크 오류 (`rby1_hardware` 빌드)
`rby1-sdk`의 private 헤더 또는 라이브러리 경로가 누락될 경우 빌드 에러가 발생합니다. `CMakeLists.txt` 파일에 다음 설정을 누락하지 않고 포함시켜야 합니다.
- **헤더 경로 문제 (`version.h` 관련 에러)**:
  `CMakeLists.txt` 내 `include_directories`에 SDK private 빌드 경로를 추가합니다.
  ```cmake
  include_directories(
    # ... 기존 경로 ...
    ${RBY1_SDK_PATH}/build/private
  )
  ```
- **라이브러리 경로 문제**:
  SDK 라이브러리를 정상적으로 링킹하기 위해 `target_link_directories`를 선언해 줍니다.
  ```cmake
  target_link_directories(rby1_hardware PRIVATE "${RBY1_SDK_PATH}/build/src")
  ```

### 7.2 MoveIt 경로 계획(Plan) 실패 오류 (역기구학 솔버 설정 주의)
- **현상**: 조작기(앤드이펙터)를 드래그하여 Plan을 실행했을 때 계획이 계속 실패하는 현상.
- **원인**: `kinematics.yaml` 파일 내에서 양팔이 모인 복합 그룹(`dual_arm` 등)이나 전체 그룹에 역기구학(IK) 솔버가 지정되어 있는 경우 발생합니다. 일반적인 IK 솔버(KDL, TRAC-IK 등)는 기구학 구조가 일직선으로 이어진 단일 체인(Chain)일 때만 수학적 연산이 가능하며, 양팔로 갈라지는 트리(Tree)형 구조는 직접 풀지 못합니다.
- **해결책**: `kinematics.yaml`에서 체인이 아닌 복합/분기형 그룹의 솔버 설정을 주석 처리 또는 제거합니다.
  ```yaml
  # kinematics.yaml 예시
  left_arm:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    # ...
  right_arm:
    kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
    # ...
  # dual_arm:  # 주석 처리 또는 제거
  #   kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  ```
  각각의 개별 팔 그룹에 기구학 솔버가 설정되어 있다면, 상위 컨트롤러 단위에서 이들을 결합해서 사용해도 동작 계획에 아무런 지장이 없습니다.

### 7.3 ros2_control_node의 비정상 종료 (gRPC 스트림 만료 예외)
- **현상**: MoveIt을 통해 로봇을 구동하는 중간 또는 구동이 끝날 무렵 `ros2_control_node`가 `std::runtime_error (This command stream is expired)` 예외를 발생시키며 강제 종료(Crash)되는 현상.
- **원인**: gRPC 통신상의 이유로 SDK 스트림이 끊어지거나 만료되었을 때, `rby1_hardware`의 `write()` 함수가 이를 감지하지 못하고 `SendCommand()`를 계속 호출해 예외가 노드 전체로 던져져서 발생합니다.
- **해결책**: 하드웨어 인터페이스 플러그인 소스 코드의 `write()` 구현부 내에서 스트림 전송 호출을 `try-catch`문으로 감싸 예외가 발생하더라도 노드가 안전하게 생존하도록 조치합니다.
  ```cpp
  try {
    robot_->send_stream_command(sdk_target_positions);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("RBY1SystemHardware"), "SDK Stream Exception: %s", e.what());
    // 필요 시 스트림 재초기화(init_stream()) 또는 재연결 처리
  }
  ```

### 7.4 URDF 모델 및 엔드이펙터 불일치
- URDF 상에서 정의된 가상 조인트(e.g., `tool_left`, `tool_right`) 및 엔드이펙터 링크 이름이 MoveIt SRDF와 완전하게 일치하는지 재점검해야 합니다. (예: 6번 링크 대신 5번 링크가 엔드이펙터의 부모로 잡히면 기구학 계산이 왜곡됩니다.)
- 일부 커스텀 링크에서 `<collision>` 태그가 생략되어 있는 경우 터미널에 경고 로그가 남을 수 있습니다. 작동에 문제는 없으나 깨끗한 로드를 위해 URDF 모델의 충돌 설정을 깔끔하게 작성하는 것을 권장합니다.

### 7.5 Python 튜토리얼 실행 시 Numpy 버전 충돌
- MoveIt 2 또는 SDK의 Python 인터페이스를 사용할 때 Numpy 2.0 이상 버전이 설치되면 기존 Matplotlib 등 시각화 라이브러리와 버전 충돌이 발생할 수 있습니다.
- 테스트 환경 구축 시 `pip install "numpy<2.0"`으로 버전을 다운그레이드하여 설치하면 해결됩니다.

### 7.6 ros2_control 중복/오버랩 컨트롤러 활성화 충돌
- **현상**: MoveIt 데모 실행 시 `ros2_control_node`가 컨트롤러 리소스 충돌로 기동되지 않고 강제 종료되는 현상.
- **원인**: `both_arms_controller`나 `body_controller` 등, 개별 조인트가 겹치는 오버랩 컨트롤러를 동시에 기본 활성화(`ros2_controllers.yaml`에 선언)했기 때문입니다. `ros2_control`에서는 동일한 조인트 자원을 여러 컨트롤러가 동시에 점유하여 제어하는 것을 금지합니다.
- **해결책**: 조인트가 중복되는 상위 결합 컨트롤러는 `controller_manager`의 기본 자동 시작 활성 목록에서 제외하고, MoveIt 제어 시에는 독립적인 비중복 컨트롤러들(`left_arm_controller`, `right_arm_controller`, `torso_controller`, `head_controller`)만 활성화하여 사용하도록 설계합니다.

### 7.7 그리퍼 핑거 조인트 배제 및 상태 유실 경고
> [!NOTE]
> 2026-10-16부터 URDF에 그리퍼 조인트가 없어 이 경고는 나오지 않습니다.

- **현상**: MoveIt 실행 후 `The complete state of the robot is not yet known. Missing gripper_finger_l1, ...` 경고가 주기적으로 출력되는 현상.
- **원인**: RBY1 SDK의 스트림 제어 조인트(26자유도 상체) 목록에 그리퍼 핑거 조인트가 포함되어 있지 않기 때문에, `ros2_control` 하드웨어 인터페이스 설정에서 그리퍼 조인트를 삭제해야 플러그인이 정상 작동합니다. 이로 인해 `joint_state_broadcaster`가 그리퍼 조인트 상태를 토픽으로 발행하지 않아 MoveIt이 미정의 조인트 상태로 인식해 경고를 출력합니다.
- **해결책**: 그리퍼는 MoveIt의 실시간 Trajectory 제어 대상이 아니며, 드라이버가 제공하는 별도의 Action/Service로 제어되므로 해당 경고는 동작 제어에 무해하므로 무시해도 무방합니다.

### 7.8 librby1-sdk.so 런타임 공유 라이브러리 로드 실패 (RPATH 문제)
- **현상**: `ros2_control_node`가 `librby1_hardware.so` 플러그인을 동적 로드하려 할 때, `librby1-sdk.so`를 찾을 수 없다는 에러(`error to load shared library`)와 함께 비정상 종료되는 현상.
- **원인**: 빌드된 플러그인이 런타임 공유 라이브러리 경로(RPATH)를 기억하지 못해 발생합니다.
- **해결책**: `rby1_hardware` 패키지의 `CMakeLists.txt` 내에서 플러그인 타겟 빌드 시 SDK 빌드 폴더를 RPATH로 탐색할 수 있게 설정합니다.
  ```cmake
  set_target_properties(rby1_hardware PROPERTIES
    INSTALL_RPATH "${RBY1_SDK_PATH}/build/src"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
  ```

### 7.9 SDK PowerOn / ServoOn 정규식 사용 주의 ("all" vs ".*")
- **현상**: 하드웨어 인터페이스에서 `robot_->power_on("all")` 또는 `robot_->servo_on("all")`을 시도할 때 `Failed to enable Servo ON for device: all` 에러가 나면서 활성화에 실패하는 현상.
- **원인**: RBY1 SDK의 `PowerOn`/`ServoOn`은 타겟 조인트/전원 매칭을 위한 **정규표현식(Regular Expression)**을 인자로 받습니다. 단순히 `"all"`을 넘겨줄 경우 기기명이 `"all"`과 일치하는 장치가 없어 아무 작업도 수행하지 못하고 에러로 반환됩니다.
- **해결책**: 모든 장치를 대상으로 전원이나 서보를 인가할 때는 정규식 형태인 `.*`로 전달해 주어야 합니다.

### 7.10 하드웨어 인터페이스 내 드라이버 서비스 호출을 통한 기기 기동
- **현상**: `initialize_robot` 플래그를 드라이버와 하드웨어 인터페이스 양쪽에서 다루거나, 각 프로세스가 SDK에 중복 접근하여 전원을 관리할 때 동기화가 깨지거나 기동 순서 타이밍(48V 안정화 지연 부족)으로 서보 온이 실패하는 문제.
- **해결책**:
  1. 드라이버의 parameters yaml에서 `initialize_robot` 설정을 완전히 제거하거나 `false`로 격리합니다.
  2. `rby1_system_hardware.cpp` 기동 시, SDK에 직접 전원을 인가하는 대신 드라이버 노드가 노출하는 `/robot_power` 및 `/robot_servo` 서비스 클라이언트를 생성하여 호출합니다. 드라이버가 정규식 변환과 순차 기동을 안정적으로 처리해 준 뒤 hardware control을 활성화하므로 통신 충돌이 원천 방지됩니다.

### 7.11 xacro 내 load_yaml()의 Deprecation 경고
- **현상**: MoveIt 2 기동 시 `Using load_yaml() directly is deprecated. Use xacro.load_yaml() instead.` 경고 메세지가 반복 출력되는 현상.
- **원인**: ROS 2 Humble 버전 업데이트에 따라 Xacro 파일 내에서 YAML 로드 시 `load_yaml`을 직접 사용하는 방식이 지원 중단될 예정이기 때문입니다.
- **해결책**: `ros2_control.xacro` 파일 내의 initial_positions 로딩 라인을 다음과 같이 수정합니다.
  ```xml
  <!-- 변경 전 -->
  <xacro:property name="initial_positions" value="${load_yaml(initial_positions_file)['initial_positions']}"/>
  <!-- 변경 후 -->
  <xacro:property name="initial_positions" value="${xacro.load_yaml(initial_positions_file)['initial_positions']}"/>
  ```

### 7.12 JointPositionCommandBuilder 복사 생성자 삭제에 따른 빌드 오류
- **현상**: `send_stream_command` 내부에서 `auto cmd_builder = rb::JointPositionCommandBuilder()` 형태로 객체를 생성하여 메서드 체이닝을 구성하려 할 때 빌드 오류 발생.
- **원인**: `rb::JointPositionCommandBuilder` 클래스는 내부적으로 `std::unique_ptr`를 소유하고 있기 때문에 복사 생성자가 명시적으로 삭제(deleted copy constructor)되어 있어, 복사 대입 형식의 객체 초기화가 불가합니다.
- **해결책**: 아래와 같이 스택 상에 명시적 인스턴스로 변수를 선언한 뒤 속성을 적용하고 함수 호출부에는 참조형으로 전달합니다.
  ```cpp
  rb::JointPositionCommandBuilder cmd_builder;
  cmd_builder.SetCommandHeader(rb::CommandHeaderBuilder().SetControlHoldTime(1e6))
             .SetPosition(left_q)
             .SetMinimumTime(0.01);
  left_arm_builder.SetCommand(cmd_builder); // const reference로 전달
  ```
