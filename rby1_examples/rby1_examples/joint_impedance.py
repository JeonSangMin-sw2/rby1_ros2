#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import SingleJointCommand
from rby1_msgs.srv import StateOnOff, ControlMode
from std_msgs.msg import Int32

class JointImpedanceExample(Node):
    def __init__(self):
        super().__init__('joint_impedance_example')
        self._action_client = ActionClient(self, SingleJointCommand, 'joint_states/single_position_command')
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

    def send_control_mode_request(self, part_name, control_type, stiffness=None, damping_ratio=0.0):
        req = ControlMode.Request()
        req.part_name = part_name
        req.control_type = control_type
        if stiffness:
            req.joint_stiffness = stiffness
        req.damping_ratio = damping_ratio
        future = self.control_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_goal(self, target_name, position, minimum_time):
        goal_msg = SingleJointCommand.Goal()
        goal_msg.target_name = target_name
        goal_msg.position = position
        goal_msg.minimum_time = minimum_time

        self._action_client.wait_for_server()
        self.get_logger().info(f'Sending Goal to {target_name}...')
        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = JointImpedanceExample()

    state = action_client.wait_for_state()
    if state not in [2, 3]:
        action_client.get_logger().info('Robot is not enabled. Sending Power and Servo ON...')
        action_client.send_power_request(True, 'all')
        action_client.send_servo_request(True, 'all')
        while action_client.wait_for_state() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(action_client, timeout_sec=0.1)

    target = "right_arm"
    
    # 1. Move to Ready Pose in Joint Position mode
    action_client.get_logger().info('Step 1: Moving to Ready Pose (Joint Position)...')
    action_client.send_control_mode_request(target, ControlMode.Request.JOINT_POSITION)
    
    ready_position = [0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
    future = action_client.send_goal(target, ready_position, 4.0)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()
    if goal_handle.accepted:
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(action_client, get_result_future)

    # 2. Switch to Joint Impedance mode
    action_client.get_logger().info('Step 2: Switching to Joint Impedance mode...')
    stiffness = [40.0] * 7 # Soft
    action_client.send_control_mode_request(
        part_name=target,
        control_type=ControlMode.Request.JOINT_IMPEDANCE,
        stiffness=stiffness,
        damping_ratio=0.85
    )

    # 3. Stay in Impedance mode for 10 seconds
    action_client.get_logger().info('Impedance activated. You can now push the robot gently. Waiting 10s...')
    # Send a goal to the same position to ensure the impedance controller is active
    action_client.send_goal(target, ready_position, 0.1)
    
    time.sleep(10.0)

    action_client.get_logger().info('10 seconds elapsed. Finishing example.')
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
