#!/usr/bin/env python3
"""
Tool Flange Test Example
========================
Tests tool flange power control and verifies that sensor data is being
published correctly on the split left/right topics.

The example calls the 'tool_flange_power' service to switch on the 12V
output rail, then monitors both flanges for 10 seconds, printing force,
torque, IMU, and digital I/O values. Power is switched off at the end.

Note:
  The driver must have 'publish_tool_flange_state: true' in
  driver_parameters.yaml for the tool flange topics to be available.
  In simulation, FT/IMU data will read 0 because no sensor is attached.

Sequence:
  1. Call tool_flange_power service (ON, 12V).
  2. Spin for 10 seconds, printing left and right flange state.
  3. Call tool_flange_power service (OFF).

Run:
  ros2 run rby1_examples tool_flange_test

Services used:
  - tool_flange_power (StateOnOff)
    Parameters: '12v', '24v', '48v'  (ON)  or  ''  (OFF)

Topics subscribed:
  - joint_states/tool_flange/left  (ToolFlangeState)
  - joint_states/tool_flange/right (ToolFlangeState)
"""
import time
import rclpy
from rclpy.node import Node
from rby1_msgs.srv import StateOnOff
from rby1_msgs.msg import ToolFlangeState


class ToolFlangeTest(Node):
    def __init__(self):
        super().__init__('tool_flange_test')

        # Service client for tool flange power control
        self.tool_flange_client = self.create_client(StateOnOff, 'tool_flange_power')

        # Subscribe to split left / right tool flange topics (new API)
        self.left_sub = self.create_subscription(
            ToolFlangeState,
            'joint_states/tool_flange/left',
            lambda msg: self._tf_callback(msg, 'Left'),
            10)
        self.right_sub = self.create_subscription(
            ToolFlangeState,
            'joint_states/tool_flange/right',
            lambda msg: self._tf_callback(msg, 'Right'),
            10)

        self.last_tf_left = None
        self.last_tf_right = None

    def _tf_callback(self, msg: ToolFlangeState, side: str):
        if side == 'Left':
            self.last_tf_left = msg
        else:
            self.last_tf_right = msg

    def send_tool_flange_request(self, state: bool, parameters: str) -> bool:
        """tool_flange_power 서비스를 호출합니다.

        Args:
            state: True = power ON, False = power OFF
            parameters: '12v', '24v', '48v' 또는 '' (OFF 시 무시)

        Returns:
            True if the service call succeeded.
        """
        if not self.tool_flange_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                "Service 'tool_flange_power' not available after 5 seconds.")
            return False

        req = StateOnOff.Request()
        req.state = state
        req.parameters = parameters

        self.get_logger().info(
            f"Calling tool_flange_power: state={state}, params='{parameters}'...")
        future = self.tool_flange_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error("No response from tool_flange_power service.")
            return False

        if result.success:
            self.get_logger().info(f"tool_flange_power OK: {result.message}")
        else:
            self.get_logger().error(f"tool_flange_power FAILED: {result.message}")
        return result.success

    def print_tf_state(self, msg: ToolFlangeState, side: str):
        """ToolFlangeState 메시지를 읽기 쉽게 출력합니다."""
        force_str = ', '.join(f'{v:.3f}' for v in msg.ft_force)
        torque_str = ', '.join(f'{v:.3f}' for v in msg.ft_torque)
        gyro_str = ', '.join(f'{v:.4f}' for v in msg.gyro)
        accel_str = ', '.join(f'{v:.4f}' for v in msg.acceleration)
        voltage_v = msg.output_voltage / 1000.0  # mV → V

        self.get_logger().info(
            f'[{side}] FT Force: [{force_str}] N  |  Torque: [{torque_str}] Nm')
        self.get_logger().info(
            f'[{side}] Gyro: [{gyro_str}] rad/s  |  Accel: [{accel_str}] m/s²')
        self.get_logger().info(
            f'[{side}] Switch A: {msg.switch_a}  |  Voltage: {voltage_v:.2f} V')
        self.get_logger().info(
            f'[{side}] DI: A={msg.digital_input_a}, B={msg.digital_input_b}  '
            f'|  DO: A={msg.digital_output_a}, B={msg.digital_output_b}')


def main(args=None):
    rclpy.init(args=args)
    test_node = ToolFlangeTest()

    # 1. Turn on 12V tool flange power
    ok = test_node.send_tool_flange_request(True, '12v')
    if ok:
        test_node.get_logger().info('Tool flange 12V power ON.')
    else:
        test_node.get_logger().warn(
            'Tool flange power ON failed (simulator may not support this). '
            'Continuing to monitor state...')

    # 2. Monitor both flanges for 10 seconds
    test_node.get_logger().info('Monitoring tool flange states for 10 seconds...')
    start_time = time.time()
    received_any = False
    while time.time() - start_time < 10.0 and rclpy.ok():
        rclpy.spin_once(test_node, timeout_sec=0.5)

        got_left = test_node.last_tf_left is not None
        got_right = test_node.last_tf_right is not None

        if got_left or got_right:
            received_any = True
            if got_left:
                test_node.print_tf_state(test_node.last_tf_left, 'Left')
            if got_right:
                test_node.print_tf_state(test_node.last_tf_right, 'Right')
            # Reset to avoid printing duplicate data every loop
            test_node.last_tf_left = None
            test_node.last_tf_right = None
        else:
            test_node.get_logger().info(
                'Waiting for tool flange states '
                '(check driver_parameters.yaml: publish_tool_flange_state: true)...')

    if not received_any:
        test_node.get_logger().warn(
            'No tool flange state data received. '
            'Ensure publish_tool_flange_state is set to true in driver_parameters.yaml.')

    # 3. Turn off power
    test_node.get_logger().info('Turning off tool flange power...')
    test_node.send_tool_flange_request(False, '')

    test_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
