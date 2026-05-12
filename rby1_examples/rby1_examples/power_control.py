#!/usr/bin/env python3
import sys
import time
import rclpy
from rclpy.node import Node
from rby1_msgs.srv import StateOnOff
from std_msgs.msg import Int32

class PowerControlExample(Node):
    def __init__(self):
        super().__init__('power_control_example')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.state_sub = self.create_subscription(Int32, 'joint_states/control_state', self.state_callback, 10)
        self.control_state = None

        while not self.power_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('power service not available, waiting again...')
        while not self.servo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('servo service not available, waiting again...')

    def state_callback(self, msg):
        self.control_state = msg.data

    def wait_for_state(self):
        while self.control_state is None and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.control_state

    def send_power_request(self, state, parameters):
        req = StateOnOff.Request()
        req.state = state
        req.parameters = parameters
        future = self.power_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_servo_request(self, state, parameters):
        req = StateOnOff.Request()
        req.state = state
        req.parameters = parameters
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

def main(args=None):
    rclpy.init(args=args)
    node = PowerControlExample()

    state = node.wait_for_state()
    node.get_logger().info(f'Current control state: {state}')

    if state not in [2, 3]:
        node.get_logger().info('Robot is not enabled (or in fault). Sending Power and Servo ON (all)...')
        node.send_power_request(True, 'all')
        node.send_servo_request(True, 'all')
        
        while node.wait_for_state() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info('Robot enabled.')
    else:
        node.get_logger().info('Robot is already enabled. Proceeding...')

    node.get_logger().info('Waiting for 3 seconds...')
    time.sleep(3.0)

    node.get_logger().info('Sending Power OFF (all)')
    res = node.send_power_request(False, 'all')
    node.get_logger().info(f'Power OFF Result: {res.success}, msg: {res.message}')

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
