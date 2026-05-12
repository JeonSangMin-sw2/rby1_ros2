#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from rby1_msgs.srv import StateOnOff
from rby1_msgs.msg import ToolFlangeState

class ToolFlangeTest(Node):
    def __init__(self):
        super().__init__('tool_flange_test')
        self.tool_flange_client = self.create_client(StateOnOff, 'tool_flange_power')
        self.tf_sub = self.create_subscription(ToolFlangeState, 'joint_states/tool_flange_state', self.tf_callback, 10)
        self.last_tf_state = None

    def tf_callback(self, msg):
        self.last_tf_state = msg

    def send_tool_flange_request(self, state, parameters):
        req = StateOnOff.Request()
        req.state = state
        req.parameters = parameters
        
        self.get_logger().info(f'Calling tool_flange_power service with state={state}, params={parameters}...')
        future = self.tool_flange_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    test_node = ToolFlangeTest()

    # 1. Turn on 12V tool flange power
    result = test_node.send_tool_flange_request(True, '12v')
    if result and result.success:
        test_node.get_logger().info('Tool flange 12V power ON.')
    else:
        test_node.get_logger().error('Failed to turn on tool flange power.')

    # 2. Collect data for 10 seconds
    test_node.get_logger().info('Monitoring tool flange states for 10 seconds...')
    start_time = time.time()
    while time.time() - start_time < 10.0 and rclpy.ok():
        rclpy.spin_once(test_node, timeout_sec=1.0)
        if test_node.last_tf_state:
            # Print right tool flange status
            tf = test_node.last_tf_state
            test_node.get_logger().info(
                f'[Right] Volt: {tf.tool_flange_right_output_voltage}V, '
                f'SwitchA: {tf.tool_flange_right_switch_a}, '
                f'DI(A/B): {tf.tool_flange_right_digital_input_a}/{tf.tool_flange_right_digital_input_b}'
            )
            # Print FT data
            force = tf.ft_force_right
            test_node.get_logger().info(f'[FT Right] Force: [{force[0]:.2f}, {force[1]:.2f}, {force[2]:.2f}]')
        else:
            test_node.get_logger().info('Waiting for tool flange states...')

    # 3. Turn off power
    test_node.get_logger().info('Turning off tool flange power...')
    test_node.send_tool_flange_request(False, '')

    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
