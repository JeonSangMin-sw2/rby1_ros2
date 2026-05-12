import time
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

class CartesianControl(Node):
    def __init__(self):
        super().__init__('cartesian_control')
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


    def wait_for_info(self, required_ref=None):
        while rclpy.ok():
            ready = True
            if self.control_state is None: ready = False
            if self.current_right_matrix is None: ready = False
            if self.current_left_matrix is None: ready = False
            
            if required_ref:
                if self.current_right_ref != required_ref: ready = False
                if self.current_left_ref != required_ref: ready = False
                
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
                                  linear_velocity_limit=0.5, angular_velocity_limit=0.5):
        req = ControlMode.Request()
        req.part_name = part_name
        req.control_type = control_type
        req.ref_link = ref_link
        req.target_link = target_link
        req.linear_velocity_limit = linear_velocity_limit
        req.angular_velocity_limit = angular_velocity_limit
        future = self.control_mode_client.call_async(req)
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

    def send_multi_goal(self, torso_pos, right_arm_pos, left_arm_pos, minimum_time):
        goal_msg = MultiJointCommand.Goal()
        goal_msg.torso = torso_pos if torso_pos is not None else []
        goal_msg.right_arm = right_arm_pos if right_arm_pos is not None else []
        goal_msg.left_arm = left_arm_pos if left_arm_pos is not None else []
        goal_msg.minimum_time = minimum_time

        self._multi_action_client.wait_for_server()
        self.get_logger().info('Sending Multi Goal to all parts...')
        return self._multi_action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = CartesianControl()

    action_client.get_logger().info('Waiting for robot info (DOF, state)...')
    state = action_client.wait_for_info()
    
    if state not in [2, 3]:
        action_client.get_logger().info('Enabling Robot...')
        action_client.send_power_request(True, 'all')
        action_client.send_servo_request(True, 'all')
        while action_client.wait_for_info() not in [2, 3] and rclpy.ok():
            rclpy.spin_once(action_client, timeout_sec=0.1)

    # Wait for robot info
    control_state = action_client.wait_for_info()
    action_client.get_logger().info('Robot info and Cartesian feedback received.')
    action_client.get_logger().info('--- Step 1: Moving Both Arms to Ready Pose (Multi-Command) ---')
    
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
        action_client.get_logger().info(f'Multi-move Result: {result.success}, Code: {result.finish_code}')
        if not result.success:
            return
    else:
        action_client.get_logger().error('Multi-move Goal rejected.')
        return

    # 2. Cartesian move for Both Arms (Reference: base)
    ref_link = "link_torso_5"
    action_client.get_logger().info(f'Switching reference frame to {ref_link}...')
    
    # Update control mode to CARTESIAN_POSITION with base reference first
    action_client.send_control_mode_request('torso', ControlMode.Request.CARTESIAN_POSITION, 
                                            ref_link='base', target_link='link_torso_5',
                                            linear_velocity_limit=1.5, angular_velocity_limit=4.7)
    action_client.send_control_mode_request('right_arm', ControlMode.Request.CARTESIAN_POSITION, 
                                            ref_link=ref_link, target_link='link_right_arm_6',
                                            linear_velocity_limit=1.5, angular_velocity_limit=4.7)
    action_client.send_control_mode_request('left_arm', ControlMode.Request.CARTESIAN_POSITION, 
                                            ref_link=ref_link, target_link='link_left_arm_6',
                                            linear_velocity_limit=1.5, angular_velocity_limit=4.7)

    # Wait for Cartesian feedback to update to the new reference frame (base)
    action_client.get_logger().info(f'Waiting for Cartesian feedback in {ref_link} frame...')
    action_client.wait_for_info(required_ref=ref_link)
    
    # Extra safety spin to ensure data has fully settled in the local variables
    action_client.get_logger().info('Spinning for 1 second to settle data...')
    start_time = action_client.get_clock().now()
    while (action_client.get_clock().now() - start_time).nanoseconds < 1e9:
        rclpy.spin_once(action_client, timeout_sec=0.1)

    p_right = action_client.current_right_matrix[:3, 3]
    action_client.get_logger().info(f'Current Right Pose ({ref_link} Ref): x={p_right[0]:.4f}, y={p_right[1]:.4f}, z={p_right[2]:.4f}')
    p_left = action_client.current_left_matrix[:3, 3]
    action_client.get_logger().info(f'Current Left Pose ({ref_link} Ref): x={p_left[0]:.4f}, y={p_left[1]:.4f}, z={p_left[2]:.4f}')

    # Target Absolute Coordinates (Relative to base)
    T_right = action_client.current_right_matrix.copy()
    T_right[0, 3] = 0.4
    T_right[1, 3] = -0.3
    T_right[2, 3] = 0.1
    
    T_left = action_client.current_left_matrix.copy()
    T_left[0, 3] = 0.4
    T_left[1, 3] = 0.3
    T_left[2, 3] = 0.1
    
    action_client.get_logger().info('--- Step 2: Moving Both Arms in Cartesian space (Multi-Command) ---')
    action_client.get_logger().info(f'Target Right: x={T_right[0,3]}, y={T_right[1,3]}, z={T_right[2,3]}')
    action_client.get_logger().info(f'Target Left:  x={T_left[0,3]}, y={T_left[1,3]}, z={T_left[2,3]}')

    # Flatten matrices in column-major order ('F')
    right_cartesian_pos = T_right.flatten('F').tolist()
    left_cartesian_pos = T_left.flatten('F').tolist()

    # Move both arms (torso stays at current pose)
    future = action_client.send_multi_goal(None, right_cartesian_pos, left_cartesian_pos, 3.0)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(action_client, result_future)
        result = result_future.result().result
        action_client.get_logger().info(f'Multi-move Result: {result.success}, Code: {result.finish_code}')
        
        # Settle for feedback update
        rclpy.spin_once(action_client, timeout_sec=0.2)
        pr = action_client.current_right_matrix[:3, 3]
        pl = action_client.current_left_matrix[:3, 3]
        action_client.get_logger().info(f'Final Right Pose: x={pr[0]:.4f}, y={pr[1]:.4f}, z={pr[2]:.4f}')
        action_client.get_logger().info(f'Final Left Pose:  x={pl[0]:.4f}, y={pl[1]:.4f}, z={pl[2]:.4f}')
    else:
        action_client.get_logger().error('Multi-move goal rejected.')

    action_client.get_logger().info('All steps finished.')
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
