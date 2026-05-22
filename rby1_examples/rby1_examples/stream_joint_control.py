#!/usr/bin/env python3
"""
Stream Joint Control Example
============================
Demonstrates real-time joint trajectory streaming to the RBY1 robot
using the StreamPosition action. A pre-computed JointTrajectory message
is sent as a single goal containing all waypoints; the driver then
interpolates and executes them at the requested timing.

Sequence:
  1. Ensure robot is powered on and servos are active.
  2. Move whole body to zero pose (safe starting posture).
  3. Build a JointTrajectory with 10 waypoints interpolated from
     zero to a target multi-joint configuration over 5 seconds.
  4. Send the trajectory via the StreamPosition action.
  5. Wait for the action to complete and report the result.

Run:
  ros2 run rby1_examples stream_joint_control

Actions used:
  - joint_states/stream_position_command  (StreamPosition)

Services used:
  - robot_power  (StateOnOff)
  - robot_servo  (StateOnOff)

Topics subscribed:
  - joint_states/robot_state  (RobotState)
"""
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rby1_msgs.action import StreamPosition, MultiJointCommand
from rby1_msgs.msg import RobotState
from rby1_msgs.srv import ControlMode, StateOnOff
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class StreamJointControl(Node):
    def __init__(self):
        super().__init__('stream_joint_control')
        self._stream_client = ActionClient(self, StreamPosition, '/joint_states/stream_position_command')
        self._multi_client = ActionClient(self, MultiJointCommand, '/joint_states/multi_position_command')
        self.control_mode_client = self.create_client(ControlMode, 'set_control_mode')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.state_sub = self.create_subscription(RobotState, 'joint_states/robot_state', self.state_callback, 10)
        self.control_state = None

    def state_callback(self, msg):
        self.control_state = msg.control_manager_state

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
        
        time.sleep(1.0) # Wait for power to stabilize
            
        # 3. Servo On
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False
            
        # 4. Wait for state to become 2 or 3
        if self.wait_for_state([2, 3], timeout=10.0):
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
        self.get_logger().info(f'Setting Control Mode for {part_name}...')
        self.control_mode_client.wait_for_service()
        future = self.control_mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def go_to_zero_pose(self):
        self.get_logger().info('Step 1: Moving to Zero Pose...')
        goal_msg = MultiJointCommand.Goal()
        goal_msg.torso = [0.0] * 6
        goal_msg.right_arm = [0.0] * 7
        goal_msg.left_arm = [0.0] * 7
        goal_msg.head = [0.0] * 2
        goal_msg.minimum_time = 3.0
        
        self._multi_client.wait_for_server()
        future = self._multi_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if goal_handle.accepted:
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            result = res_future.result().result
            if result.finish_code == 'kOk':
                self.get_logger().info('Zero Pose reached successfully.')
                return True
            else:
                self.get_logger().error(f'Zero Pose failed with code: {result.finish_code}')
                return False
        return True

    def send_stream_goal(self, trajectory):
        self.get_logger().info('Step 2: Starting Trajectory Streaming...')
        goal_msg = StreamPosition.Goal()
        goal_msg.trajectory = trajectory

        self._stream_client.wait_for_server()
        return self._stream_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = StreamJointControl()

    # 0. Ensure Robot is Power ON and Servo ON
    if not action_client.ensure_robot_ready():
        action_client.get_logger().error('Failed to prepare robot. Exiting.')
        return

    # 1. Set Control Mode for all parts
    parts = ["torso", "right_arm", "left_arm", "head"]
    for part in parts:
        action_client.send_control_mode_request(part, ControlMode.Request.JOINT_POSITION)

    import time
    time.sleep(0.5)

    # 2. First move Whole Body to Zero Pose
    if not action_client.go_to_zero_pose():
        action_client.get_logger().error('Failed to move to Zero Pose.')
        return

    # 3. Define target positions from multi_joint example
    target_torso = [0.0, 0.1, -0.2, 0.1, 0.0, 0.0]
    target_right = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    target_left  = [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    target_head  = [0.0, 0.0]

    # Combine into one full-body target vector
    full_target = target_torso + target_right + target_left + target_head
    full_start  = [0.0] * len(full_target)
    
    # Define joint names for the entire robot (order must match the vector)
    joint_names = [f'torso_{i}' for i in range(6)] + \
                  [f'right_arm_{i}' for i in range(7)] + \
                  [f'left_arm_{i}' for i in range(7)] + \
                  [f'head_{i}' for i in range(2)]
    
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    
    # Create trajectory from Zero to Target in 10 steps
    for i in range(1, 11):
        point = JointTrajectoryPoint()
        # Linear interpolation for all 22 joints
        point.positions = [(s + (t - s) * i / 10.0) for s, t in zip(full_start, full_target)]
        total_ms = i * 500
        point.time_from_start.sec = total_ms // 1000
        point.time_from_start.nanosec = (total_ms % 1000) * 1000000
        trajectory.points.append(point)

    action_client.get_logger().info(f'Streaming 22 joints to multi_joint pose over 5 seconds...')
    
    # Wait a bit for the previous action to fully settle in SDK
    import time
    time.sleep(2.0)

    future = action_client.send_stream_goal(trajectory)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(action_client, get_result_future)
        result = get_result_future.result().result
        action_client.get_logger().info(f'StreamPosition finished. Code: {result.finish_code}')

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
