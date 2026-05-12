#!/usr/bin/env python3
import time
import rclpy
import numpy as np
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import SingleJointCommand, MultiJointCommand
from rby1_msgs.srv import StateOnOff, ControlMode
from rby1_msgs.msg import CartesianPose
from std_msgs.msg import Int32

class CartesianImpedanceExample(Node):
    def __init__(self):
        super().__init__('cartesian_impedance_example')
        self._single_action_client = ActionClient(self, SingleJointCommand, 'joint_states/single_position_command')
        self._multi_action_client = ActionClient(self, MultiJointCommand, 'joint_states/multi_position_command')
        
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.control_mode_client = self.create_client(ControlMode, 'set_control_mode')
        self.state_sub = self.create_subscription(Int32, 'joint_states/control_state', self.state_callback, 10)
        
        self.right_cartesian_sub = self.create_subscription(CartesianPose, 'joint_states/right_cartesian', self.right_cartesian_callback, 10)
        self.left_cartesian_sub = self.create_subscription(CartesianPose, 'joint_states/left_cartesian', self.left_cartesian_callback, 10)

        self.control_state = None
        self.current_right_matrix = None
        self.current_right_ref = None
        self.current_left_matrix = None
        self.current_left_ref = None

    def state_callback(self, msg):
        self.control_state = msg.data
        
    def right_cartesian_callback(self, msg):
        self.current_right_matrix = np.array(msg.matrix).reshape((4, 4), order='F')
        self.current_right_ref = msg.reference_link

    def left_cartesian_callback(self, msg):
        self.current_left_matrix = np.array(msg.matrix).reshape((4, 4), order='F')
        self.current_left_ref = msg.reference_link

    def wait_for_info(self, required_ref=None, check_right=True, check_left=True):
        while rclpy.ok():
            ready = True
            if self.control_state is None: ready = False
            if check_right and self.current_right_matrix is None: ready = False
            if check_left and self.current_left_matrix is None: ready = False
            
            if required_ref:
                if check_right and self.current_right_ref != required_ref: ready = False
                if check_left and self.current_left_ref != required_ref: ready = False
                
            if ready: break
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

    def send_control_mode_request(self, part_name, control_type, ref_link="", target_link="", 
                                  t_weight=None, r_weight=None):
        req = ControlMode.Request()
        req.part_name = part_name
        req.control_type = control_type
        req.ref_link = ref_link
        req.target_link = target_link
        req.translation_weight = t_weight if t_weight is not None else []
        req.rotation_weight = r_weight if r_weight is not None else []
        future = self.control_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def send_single_goal(self, target_name, matrix_4x4, minimum_time):
        goal_msg = SingleJointCommand.Goal()
        goal_msg.target_name = target_name
        goal_msg.position = matrix_4x4.flatten('F').tolist()
        goal_msg.minimum_time = minimum_time

        self._single_action_client.wait_for_server()
        self.get_logger().info(f'Sending Cartesian Goal to {target_name}...')
        return self._single_action_client.send_goal_async(goal_msg)

    def send_multi_goal(self, torso_pos, right_pos, left_pos, minimum_time):
        goal_msg = MultiJointCommand.Goal()
        goal_msg.torso = torso_pos if torso_pos is not None else []
        goal_msg.right_arm = right_pos if right_pos is not None else []
        goal_msg.left_arm = left_pos if left_pos is not None else []
        goal_msg.minimum_time = minimum_time

        self._multi_action_client.wait_for_server()
        return self._multi_action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = CartesianImpedanceExample()

    action_client.get_logger().info('Waiting for robot state...')
    state = action_client.wait_for_info()
    
    if state not in [2, 3]:
        action_client.get_logger().info('Enabling Robot...')
        action_client.send_power_request(True, 'all')
        action_client.send_servo_request(True, 'all')
        while action_client.wait_for_info() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(action_client, timeout_sec=0.1)

    # --- Step 1: Moving to Ready Pose ---
    action_client.get_logger().info('--- Step 1: Moving to Ready Pose ---')
    action_client.send_control_mode_request('torso', ControlMode.Request.JOINT_POSITION)
    action_client.send_control_mode_request('right_arm', ControlMode.Request.JOINT_POSITION)
    action_client.send_control_mode_request('left_arm', ControlMode.Request.JOINT_POSITION)

    torso_pos = [0.0, 0.1, -0.2, 0.1, 0.0, 0.0]
    right_pos = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    left_pos = [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    
    future = action_client.send_multi_goal(torso_pos, right_pos, left_pos, 3.0)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()
    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(action_client, result_future)
        result = result_future.result().result
        action_client.get_logger().info(f'Move to Ready Result: {result.success}, Code: {result.finish_code}')
        if not result.success: return
    else:
        action_client.get_logger().error('Ready Pose move rejected.')
        return

    # --- Step 2: Activate Cartesian Impedance at the SAME position ---
    target = "right_arm"
    ref_link = "link_torso_5"
    target_link = "link_right_arm_6"
    
    action_client.get_logger().info(f'--- Step 2: Activating Cartesian Impedance for {target} ---')
    
    # Switch to CARTESIAN_IMPEDANCE mode to update the reference frame
    action_client.send_control_mode_request(
        part_name=target,
        control_type=ControlMode.Request.CARTESIAN_IMPEDANCE,
        ref_link=ref_link,
        target_link=target_link,
        t_weight=[1000.0, 1000.0, 1000.0], # Translation Stiffness
        r_weight=[100.0, 100.0, 100.0]     # Rotation Stiffness
    )

    # Wait for Cartesian feedback to settle with the desired reference
    action_client.wait_for_info(required_ref=ref_link, check_left=False)
    
    # Extra safety spin to ensure data has fully settled
    start_time = action_client.get_clock().now()
    while (action_client.get_clock().now() - start_time).nanoseconds < 1e9:
        rclpy.spin_once(action_client, timeout_sec=0.1)

    # Use the current matrix as the target position
    current_T = action_client.current_right_matrix.copy()
    p = current_T[:3, 3]
    action_client.get_logger().info(f'Maintaining current pose with Impedance: x={p[0]:.4f}, y={p[1]:.4f}, z={p[2]:.4f}')

    # Send command to reach this position
    future = action_client.send_single_goal(target, current_T, 1.0)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        action_client.get_logger().info('Impedance control active. Maintaining for 10 seconds...')
        # Wait for the action to complete or just sleep to maintain state
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(action_client, get_result_future)
        
        # Maintain the state for 10 seconds as requested
        time.sleep(10.0)
        action_client.get_logger().info('Impedance session finished.')
    else:
        action_client.get_logger().error('Impedance goal rejected.')

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
