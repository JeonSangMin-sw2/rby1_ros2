#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import MultiJointCommand
from rby1_msgs.srv import StateOnOff, ControlMode
from std_msgs.msg import Int32

class JointGroupExample(Node):
    def __init__(self):
        super().__init__('joint_group_example')
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

    def send_control_mode_request(self, part_name, control_type, joint_names):
        req = ControlMode.Request()
        req.part_name = part_name
        req.control_type = control_type
        req.joint_names = joint_names
        future = self.control_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_goal(self, torso_pos, minimum_time):
        goal_msg = MultiJointCommand.Goal()
        goal_msg.torso = torso_pos
        goal_msg.minimum_time = minimum_time

        self._action_client.wait_for_server()
        self.get_logger().info('Sending Joint Group Goal for Torso subset...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = JointGroupExample()

    state = action_client.wait_for_state()
    if state not in [2, 3]:
        action_client.get_logger().info('Robot is not enabled. Sending Power and Servo ON...')
        action_client.send_power_request(True, 'all')
        action_client.send_servo_request(True, 'all')
        while action_client.wait_for_state() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(action_client, timeout_sec=0.1)

    # Controlling Torso 0, 1, 2 (1, 2, 3 in 1-indexed)
    target_part = 'torso'
    joint_names = ['torso_1', 'torso_2', 'torso_3']
    action_client.get_logger().info(f'Setting {target_part} mode to JOINT_GROUP_POSITION for: {joint_names}')
    
    action_client.send_control_mode_request(
        part_name=target_part,
        control_type=ControlMode.Request.JOINT_GROUP_POSITION,
        joint_names=joint_names
    )
    
    # Target positions for the 3 joints
    torso_pos = [0.2, -0.4, 0.2]
    min_time = 3.0
    
    future = action_client.send_goal(torso_pos, min_time)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    if not goal_handle.accepted:
        action_client.get_logger().info('Goal rejected :(')
        return

    action_client.get_logger().info('Goal accepted. Torso subset moving.')
    get_result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(action_client, get_result_future)

    result = get_result_future.result().result
    action_client.get_logger().info(f'Result: {result.success}, Code: {result.finish_code}')

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
