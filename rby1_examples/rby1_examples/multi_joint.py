#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import MultiJointCommand
from rby1_msgs.srv import StateOnOff, ControlMode
from std_msgs.msg import Int32

class MultiJointExample(Node):
    def __init__(self):
        super().__init__('multi_joint_example')
        self._action_client = ActionClient(self, MultiJointCommand, 'joint_states/multi_position_command')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.control_mode_client = self.create_client(ControlMode, 'set_control_mode')
        self.state_sub = self.create_subscription(Int32, 'joint_states/control_state', self.state_callback, 10)
        self.control_state = None

    def state_callback(self, msg):
        self.control_state = msg.data

    def wait_for_state(self, target_states, timeout=5.0):
        start_time = self.get_clock().now()
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.control_state in target_states:
                return True
            if (self.get_clock().now() - start_time).nanoseconds / 1e9 > timeout:
                return False
        return False

    def ensure_robot_ready(self):
        self.get_logger().info('Ensuring robot is powered on and servos are active...')
        
        # 1. Check current state first
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.control_state in [2, 3]:
            self.get_logger().info('Robot is already enabled.')
            return True

        # 2. Power On
        req = StateOnOff.Request()
        req.state = True
        req.parameters = "all"
        self.get_logger().info('Sending Power ON request...')
        self.power_client.wait_for_service()
        future = self.power_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to power on: {future.result().message}')
            return False
        
        time.sleep(2.0) # Wait for power to stabilize
            
        # 3. Servo On
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False
            
        # 4. Wait for state to become 2 or 3
        if self.wait_for_state([2, 3], timeout=15.0):
            self.get_logger().info('Robot is ready.')
            time.sleep(1.0) # One more second for control manager to settle
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable. Current state: {self.control_state}')
            return False

    def send_control_mode_request(self, part_name, control_type):
        req = ControlMode.Request()
        req.part_name = part_name
        req.control_type = control_type
        self.control_mode_client.wait_for_service()
        future = self.control_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_goal(self, torso_pos, right_pos, left_pos, minimum_time):
        goal_msg = MultiJointCommand.Goal()
        goal_msg.torso = torso_pos
        goal_msg.right_arm = right_pos
        goal_msg.left_arm = left_pos
        goal_msg.minimum_time = minimum_time

        self._action_client.wait_for_server()
        self.get_logger().info('Sending Multi Joint Command Goal...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = MultiJointExample()

    # 0. Ensure Robot is Ready
    if not action_client.ensure_robot_ready():
        action_client.get_logger().error('Robot initialization failed. Exiting.')
        return

    # 1. Set mode to JOINT_POSITION for all parts
    action_client.get_logger().info('Setting Control Mode to JOINT_POSITION...')
    action_client.send_control_mode_request('torso', ControlMode.Request.JOINT_POSITION)
    action_client.send_control_mode_request('right_arm', ControlMode.Request.JOINT_POSITION)
    action_client.send_control_mode_request('left_arm', ControlMode.Request.JOINT_POSITION)

    time.sleep(0.5)

    # 2. Target positions
    torso_pos = [0.0, 0.1, -0.2, 0.1, 0.0, 0.0]
    right_pos = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    left_pos = [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    min_time = 4.0
    
    future = action_client.send_goal(torso_pos, right_pos, left_pos, min_time)
    rclpy.spin_until_future_complete(action_client, future)

    goal_handle = future.result()
    if not goal_handle.accepted:
        action_client.get_logger().info('Goal rejected :(')
        return

    action_client.get_logger().info('Goal accepted :)')
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(action_client, get_result_future)

    result = get_result_future.result().result
    action_client.get_logger().info(f'Result: {result.success}, Code: {result.finish_code}')

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
