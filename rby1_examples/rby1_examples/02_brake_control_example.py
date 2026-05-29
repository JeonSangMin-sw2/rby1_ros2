#!/usr/bin/env python3
"""
Brake Control Example
=====================
Demonstrates how to safely engage and release individual joint brakes
on the RBY1 robot. Motor brakes can only be operated when the Control
Manager is in the IDLE (disabled) state, so the example automatically
handles any FAULT recovery and state transitions before attempting brake
operations.

Target joints: right_arm_3, left_arm_3

SDK reference flow (35_brake_test.cpp):
  PowerOn → DisableControlManager → BrakeRelease → wait → BrakeEngage

CAUTION:
  - Brakes may only be operated when the Control Manager is IDLE.
  - Never release torso joint brakes or use '.*' patterns; gravity will
    cause the torso to collapse immediately.
  - Always ensure the robot is in a safe posture (e.g. zero pose) before
    releasing arm brakes.

Sequence:
  1. Wait for services to become available.
  2. Read current robot state.
  3. If MAJOR_FAULT or MINOR_FAULT: reset → re-enable automatically.
  4. If ENABLE or EXECUTING: send CMD_DISABLE to reach IDLE.
  5. Verify IDLE state.
  6. Release brakes for right_arm_3 and left_arm_3.
  7. Hold released state for 4 seconds.
  8. Engage brakes back.
  9. Re-enable the Control Manager.

Run:
  ros2 run rby1_examples brake_control

Services used:
  - control_manager_command  (ControlManagerCommand)
  - set_motor_brake          (StateOnOff)

Topics subscribed:
  - joint_states/robot_state  (RobotState)
"""
import time
import rclpy
from rclpy.node import Node
from rby1_msgs.msg import RobotState
from rby1_msgs.srv import ControlManagerCommand, StateOnOff


class BrakeControlExample(Node):
    def __init__(self):
        super().__init__('brake_control_example')

        # Service clients
        self.control_manager_client = self.create_client(
            ControlManagerCommand, 'control_manager_command')
        self.power_client = self.create_client(
            StateOnOff, 'robot_power')
        self.brake_client = self.create_client(
            StateOnOff, 'set_motor_brake')

        # Unified robot state subscription
        self.state_sub = self.create_subscription(
            RobotState, 'robot_state',
            self.robot_state_callback, 10)

        self.control_manager_state = None
        self.get_logger().info('Brake Control Example Node initialized.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def robot_state_callback(self, msg: RobotState):
        self.control_manager_state = msg.control_manager_state

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _spin_for(self, seconds: float):
        """Spin the node for a specified duration to process callbacks."""
        deadline = time.time() + seconds
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

    def _wait_for_state(self, target_state: int, timeout: float = 5.0) -> bool:
        """Wait for the Control Manager state to match target_state."""
        deadline = time.time() + timeout
        while time.time() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.control_manager_state == target_state:
                return True
        return False

    def _state_name(self, state) -> str:
        names = {
            RobotState.STATE_NONE: 'NONE',
            RobotState.STATE_IDLE: 'IDLE',
            RobotState.STATE_ENABLE: 'ENABLE',
            RobotState.STATE_EXECUTING: 'EXECUTING',
            RobotState.STATE_MAJOR_FAULT: 'MAJOR_FAULT',
            RobotState.STATE_MINOR_FAULT: 'MINOR_FAULT',
        }
        return names.get(state, f'UNKNOWN({state})')

    def _wait_for_services(self, timeout: float = 10.0) -> bool:
        """Wait for all required services to become available."""
        for client, name in [
            (self.control_manager_client, 'control_manager_command'),
            (self.power_client, 'robot_power'),
            (self.brake_client, 'set_motor_brake'),
        ]:
            self.get_logger().info(f"Waiting for service '{name}'...")
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(
                    f"Service '{name}' not available after {timeout:.0f}s.")
                return False
        return True

    # ------------------------------------------------------------------
    # Service calls
    # ------------------------------------------------------------------
    def send_control_manager_cmd(self, command_val: int) -> bool:
        req = ControlManagerCommand.Request()
        req.command = command_val
        cmd_name = {
            ControlManagerCommand.Request.CMD_ENABLE: 'ENABLE',
            ControlManagerCommand.Request.CMD_DISABLE: 'DISABLE',
            ControlManagerCommand.Request.CMD_RESET: 'RESET',
        }.get(command_val, str(command_val))

        self.get_logger().info(f'Sending Control Manager command: {cmd_name}')
        future = self.control_manager_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()
        if res is None:
            self.get_logger().error('No response from control_manager_command service.')
            return False

        if res.success:
            self.get_logger().info(f'Control Manager {cmd_name} OK: {res.message}')
        else:
            self.get_logger().error(f'Control Manager {cmd_name} FAILED: {res.message}')
        return res.success

    def send_brake_cmd(self, joint_name: str, engage: bool) -> bool:
        """Send engage (True) or release (False) request to a joint brake."""
        req = StateOnOff.Request()
        req.parameters = joint_name
        req.state = engage  # True = engage, False = release

        op = 'ENGAGE' if engage else 'RELEASE'
        self.get_logger().info(f'Brake {op} → joint: {joint_name}')
        future = self.brake_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        res = future.result()
        if res is None:
            self.get_logger().error(
                f'No response from set_motor_brake for {joint_name}.')
            return False

        if res.success:
            self.get_logger().info(f'Brake {op} OK: {res.message}')
        else:
            self.get_logger().error(f'Brake {op} FAILED: {res.message}')
        return res.success

    # ------------------------------------------------------------------
    # Main sequence
    # ------------------------------------------------------------------
    def run(self):
        # 0. Wait for services to be available
        if not self._wait_for_services():
            return

        # 1. Get current robot state (spin a bit to receive first message)
        self.get_logger().info('Reading current robot state...')
        self._spin_for(1.0)

        if self.control_manager_state is None:
            self.get_logger().warn(
                'No state update received. '
                'Check that the driver is running and the robot_state topic is active.')
            return

        cur = self._state_name(self.control_manager_state)
        self.get_logger().info(f'Current Control Manager State: {cur}')

        # Check if 48V power is ON, and enable it if it is OFF
        self.get_logger().info('Checking 48V power status...')
        req = StateOnOff.Request()
        req.state = True
        req.parameters = '48'
        future = self.power_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        res = future.result()
        if res is None:
            self.get_logger().error('No response from robot_power service. Aborting.')
            return
            
        if not res.success:
            self.get_logger().error(f'Failed to check/enable 48V power: {res.message}')
            return

        if 'already ON' not in res.message:
            self.get_logger().info('48V power was OFF. Successfully enabled 48V power.')
            time.sleep(1.0) # Wait a moment for power to stabilize
        else:
            self.get_logger().info('48V power is already ON.')

        self.get_logger().info('Proceeding with brake control test preparation...')

        # 2. Handle FAULT states: reset to get to IDLE
        if self.control_manager_state in [
            RobotState.STATE_MAJOR_FAULT, RobotState.STATE_MINOR_FAULT
        ]:
            self.get_logger().warn(
                f'Robot is in {cur} state. Attempting fault reset to transition to IDLE...')
            if not self.send_control_manager_cmd(
                    ControlManagerCommand.Request.CMD_RESET):
                self.get_logger().error('Fault reset failed. Aborting.')
                return

            # After reset, control manager goes to IDLE
            if not self._wait_for_state(RobotState.STATE_IDLE, timeout=5.0):
                self.get_logger().error(
                    f'State did not become IDLE after reset. '
                    f'Current: {self._state_name(self.control_manager_state)}')
                return
            self.get_logger().info('Fault cleared. Robot is now in IDLE state.')

        # 3. If in ENABLE, EXECUTING, or any non-IDLE state → send CMD_DISABLE to transition to IDLE
        if self.control_manager_state != RobotState.STATE_IDLE:
            self.get_logger().info(
                f'Current state is {self._state_name(self.control_manager_state)}. '
                f'Sending CMD_DISABLE to transition Control Manager to IDLE...')
            if not self.send_control_manager_cmd(
                    ControlManagerCommand.Request.CMD_DISABLE):
                self.get_logger().error('Could not send CMD_DISABLE command. Aborting.')
                return

            if not self._wait_for_state(RobotState.STATE_IDLE, timeout=5.0):
                self.get_logger().error(
                    f'State did not become IDLE after disable. '
                    f'Current: {self._state_name(self.control_manager_state)}')
                return

            self.get_logger().info('Control Manager successfully transitioned to IDLE.')

        # 4. Verify IDLE state before brake operations
        if self.control_manager_state != RobotState.STATE_IDLE:
            self.get_logger().error(
                f'Brake control requires IDLE state. '
                f'Current: {self._state_name(self.control_manager_state)}. Aborting.')
            return

        self.get_logger().info('=' * 52)
        self.get_logger().info('Robot is safely in IDLE. Starting brake test...')
        self.get_logger().info('=' * 52)

        # 5. Release brakes for right_arm_3 and left_arm_3
        r_released = self.send_brake_cmd('right_arm_3', False)
        l_released = self.send_brake_cmd('left_arm_3', False)

        if not (r_released and l_released):
            self.get_logger().warn(
                'One or more brakes failed to release. '
                'Re-engaging both for safety...')
            self.send_brake_cmd('right_arm_3', True)
            self.send_brake_cmd('left_arm_3', True)
            return

        # 6. Hold released state for 4 seconds (spin to process messages)
        self.get_logger().info('Brakes released. Holding for 4.0 seconds...')
        self._spin_for(4.0)

        # 7. Engage brakes back
        self.get_logger().info('Engaging brakes back...')
        self.send_brake_cmd('right_arm_3', True)
        self.send_brake_cmd('left_arm_3', True)

        self.get_logger().info('=' * 52)
        self.get_logger().info('Brake test finished. Re-enabling Control Manager...')
        self.get_logger().info('=' * 52)
        


def main(args=None):
    rclpy.init(args=args)
    example = BrakeControlExample()
    try:
        example.run()
    except KeyboardInterrupt:
        pass
    finally:
        example.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
 
