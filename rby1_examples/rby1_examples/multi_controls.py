#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import MultiJointCommand
from rby1_msgs.srv import StateOnOff, ControlMode
from std_msgs.msg import Int32

class MultiControlsExample(Node):
    def __init__(self):
        super().__init__('multi_controls_example')
        self._action_client = ActionClient(self, MultiJointCommand, 'joint_states/multi_position_command')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.control_mode_client = self.create_client(ControlMode, 'set_control_mode')
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

    def send_control_mode_request(self, part_name, control_type, ref_link="", target_link=""):
        req = ControlMode.Request()
        req.part_name = part_name
        req.control_type = control_type
        req.ref_link = ref_link
        req.target_link = target_link
        future = self.control_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_multi_goal(self, torso_pos, right_arm_pos, left_arm_pos, minimum_time):
        goal_msg = MultiJointCommand.Goal()
        goal_msg.torso = torso_pos if torso_pos is not None else []
        goal_msg.right_arm = right_arm_pos if right_arm_pos is not None else []
        goal_msg.left_arm = left_arm_pos if left_arm_pos is not None else []
        goal_msg.minimum_time = minimum_time

        self._action_client.wait_for_server()
        self.get_logger().info('Sending Multi Controls Goal to all parts...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = MultiControlsExample()

    state = action_client.wait_for_state()
    if state not in [2, 3]:
        action_client.send_power_request(True, 'all')
        action_client.send_servo_request(True, 'all')
        while action_client.wait_for_state() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(action_client, timeout_sec=0.1)

    # Torso: Joint Position
    # Right Arm: Cartesian Position
    # Left Arm: Joint Position
    action_client.get_logger().info('Setting Mixed Control Modes for each part...')
    action_client.send_control_mode_request('torso', ControlMode.Request.JOINT_POSITION)
    action_client.send_control_mode_request('right_arm', ControlMode.Request.CARTESIAN_POSITION, "link_torso_5", "link_right_arm_6")
    action_client.send_control_mode_request('left_arm', ControlMode.Request.JOINT_POSITION)

    # Ready positions
    torso_pos = [0.0] * 6
    left_arm_pos = [0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
    
    # Right Arm: Cartesian Ready Pose (Transformation Matrix)
    T = np.eye(4)
    T[0, 3] = 0.4
    T[1, 3] = -0.3
    T[2, 3] = -0.1
    right_arm_pos = T.flatten('F').tolist()
    
    min_time = 3.0
    
    action_client.get_logger().info('Moving all parts to Ready Poses...')
    future = action_client.send_multi_goal(torso_pos, right_arm_pos, left_arm_pos, min_time)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(action_client, get_result_future)
        result = get_result_future.result().result
        action_client.get_logger().info(f'Multi-Control execution finished. Result: {result.success}, Code: {result.finish_code}')

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
