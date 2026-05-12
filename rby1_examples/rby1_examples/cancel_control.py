#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import SingleJointCommand, MultiJointCommand
from rby1_msgs.srv import StateOnOff, ControlMode
from std_srvs.srv import Trigger
from std_msgs.msg import Int32

class CancelControlExample(Node):
    def __init__(self):
        super().__init__('cancel_control_example')
        self._single_action_client = ActionClient(self, SingleJointCommand, 'joint_states/single_position_command')
        self._multi_action_client = ActionClient(self, MultiJointCommand, 'joint_states/multi_position_command')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.control_mode_client = self.create_client(ControlMode, 'set_control_mode')
        self.cancel_control_client = self.create_client(Trigger, 'cancel_control')
        self.state_sub = self.create_subscription(Int32, 'joint_states/control_state', self.state_callback, 10)
        self.control_state = None

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

    def send_control_mode_request(self, part_name, control_type):
        req = ControlMode.Request()
        req.part_name = part_name
        req.control_type = control_type
        future = self.control_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_cancel_service_request(self):
        req = Trigger.Request()
        future = self.cancel_control_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_single_goal(self, target_name, position, minimum_time):
        goal_msg = SingleJointCommand.Goal()
        goal_msg.target_name = target_name
        goal_msg.position = position
        goal_msg.minimum_time = minimum_time

        self._single_action_client.wait_for_server()
        self.get_logger().info(f'Sending Single Goal to {target_name}...')
        return self._single_action_client.send_goal_async(goal_msg)

    def send_multi_goal(self, left_arm_pos, minimum_time):
        goal_msg = MultiJointCommand.Goal()
        goal_msg.left_arm = left_arm_pos
        goal_msg.minimum_time = minimum_time

        self._multi_action_client.wait_for_server()
        self.get_logger().info('Sending Multi Goal (Left Arm)...')
        return self._multi_action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    example = CancelControlExample()

    state = example.wait_for_state()
    if state not in [2, 3]:
        example.get_logger().info('Robot is not enabled. Sending Power and Servo ON...')
        example.send_power_request(True, 'all')
        example.send_servo_request(True, 'all')
        while example.wait_for_state() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(example, timeout_sec=0.1)

    # 1. Action Cancellation (Right Arm)
    target = "right_arm"
    example.get_logger().info('--- Part 1: Action Cancellation ---')
    example.send_control_mode_request(target, ControlMode.Request.JOINT_POSITION)
    
    position = [0.0, -1.0, 0.0, -1.57, 0.0, 0.0, 0.0]
    min_time = 10.0
    
    future = example.send_single_goal(target, position, min_time)
    rclpy.spin_until_future_complete(example, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        example.get_logger().info('Goal accepted. Waiting 2 seconds before action-canceling...')
        time.sleep(2.0)
        cancel_future = goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(example, cancel_future)
        example.get_logger().info('Action Cancel request sent.')
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(example, get_result_future)
        result = get_result_future.result().result
        example.get_logger().info(f'Action Cancel Result: {result.success}, Code: {result.finish_code}')

    time.sleep(1.0)

    # 2. Service Cancellation (Left Arm)
    target = "left_arm"
    example.get_logger().info('--- Part 2: Service Cancellation (Global Stop) ---')
    example.send_control_mode_request(target, ControlMode.Request.JOINT_POSITION)
    
    # Symmetric pose for left arm (outward movement)
    left_position = [0.0, 1.0, 0.0, -1.57, 0.0, 0.0, 0.0] 
    min_time = 5.0
    
    future = example.send_multi_goal(left_position, min_time)
    rclpy.spin_until_future_complete(example, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        example.get_logger().info('Goal accepted. Waiting 2 seconds before service-canceling...')
        time.sleep(2.0)
        
        example.get_logger().info('Calling cancel_control service...')
        service_result = example.send_cancel_service_request()
        example.get_logger().info(f'Service Response: {service_result.success}, {service_result.message}')
        
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(example, get_result_future)
        result = get_result_future.result().result
        example.get_logger().info(f'Action result after service cancel: {result.success}, Code: {result.finish_code}')

    example.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
