#!/usr/bin/env python3
"""
Trajectory Joint Command Example
===============================
Demonstrates multi-point whole-body trajectory streaming via the StreamPosition
action client over a persistent command stream.

Sequence:
  1. Ensure the robot is powered and enabled.
  2. Move whole body to Zero Pose.
  3. Call the '/stream_control' service to enable a persistent command stream.
  4. Send a whole-body trajectory using the StreamPosition action client.
  5. Call the '/stream_control' service with state=False to disable stream.
  6. Return to Zero Pose.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rby1_msgs.action import StreamPosition, Rby1JointCommand
from rby1_msgs.msg import RobotState, JointCommand
from rby1_msgs.srv import StateOnOff
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class TrajectoryJointCommand(Node):
    def __init__(self):
        super().__init__('trajectory_joint_command')
        self._stream_client = ActionClient(self, StreamPosition, 'stream_position_command')
        self._zero_pose_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.stream_control_client = self.create_client(StateOnOff, 'stream_control')
        
        self.state_sub = self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
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

    def toggle_stream(self, enable: bool) -> bool:
        if not rclpy.ok():
            return False
        try:
            req = StateOnOff.Request()
            req.state = enable
            req.parameters = ""
            self.get_logger().info(f"Calling stream_control: state={enable}...")
            self.stream_control_client.wait_for_service(timeout_sec=1.0)
            future = self.stream_control_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.done():
                res = future.result()
                if res and res.success:
                    self.get_logger().info(f"Stream Control successfully {'enabled' if enable else 'disabled'}.")
                    return True
                else:
                    self.get_logger().error(f"Failed to toggle stream control: {res.message if res else 'No response'}")
            else:
                self.get_logger().error("Timeout toggling stream control.")
        except Exception as e:
            print(f"Exception during stream control toggle: {e}")
        return False

    def go_to_zero_pose(self):
        self.get_logger().info('Moving Whole Body to Zero Pose...')
        goal_msg = Rby1JointCommand.Goal()
        
        for part in ['torso', 'right_arm', 'left_arm', 'head']:
            cmd = JointCommand()
            if part == 'torso':
                cmd.position = [0.0] * 6
            elif part == 'head':
                cmd.position = [0.0] * 2
            elif part in ['right_arm', 'left_arm']:
                cmd.position = [0.0] * 7
            cmd.minimum_time = 3.0
            setattr(goal_msg, part, cmd)
        
        self._zero_pose_client.wait_for_server()
        future = self._zero_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if goal_handle.accepted:
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            result = res_future.result().result
            if result.success:
                self.get_logger().info('Zero Pose reached successfully.')
                return True
            else:
                self.get_logger().error(f'Zero Pose failed with code: {result.finish_code}')
                return False
        return True

    def send_stream_goal(self, trajectory):
        self.get_logger().info('Starting Trajectory Streaming...')
        goal_msg = StreamPosition.Goal()
        goal_msg.trajectory = trajectory

        self._stream_client.wait_for_server()
        return self._stream_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)
    action_client = TrajectoryJointCommand()

    # 0. Ensure Robot is Power ON and Servo ON
    if not action_client.ensure_robot_ready():
        action_client.get_logger().error('Failed to prepare robot. Exiting.')
        return

    # 2. Establish starting Zero Pose
    if not action_client.go_to_zero_pose():
        action_client.get_logger().error('Failed to establish zero pose.')
        return

    # 3. Enable Persistent Command Stream
    if not action_client.toggle_stream(True):
        action_client.get_logger().error('Failed to enable command stream. Exiting.')
        return

    time.sleep(1.0)

    # 4. Stream trajectory points to a target pose
    target_torso = [0.0, 0.1, -0.2, 0.1, 0.0, 0.0]
    target_right = [0.0, -0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    target_left  = [0.0, 0.5, 0.0, -1.57, 0.0, 0.0, 0.0]
    target_head  = [0.0, 0.0]

    full_target = target_torso + target_right + target_left + target_head
    full_start  = [0.0] * len(full_target)
    
    joint_names = [f'torso_{i}' for i in range(6)] + \
                  [f'right_arm_{i}' for i in range(7)] + \
                  [f'left_arm_{i}' for i in range(7)] + \
                  [f'head_{i}' for i in range(2)]
    
    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names
    
    # Create linear interpolation points
    for i in range(1, 11):
        point = JointTrajectoryPoint()
        point.positions = [(s + (t - s) * i / 10.0) for s, t in zip(full_start, full_target)]
        total_ms = i * 500
        point.time_from_start.sec = total_ms // 1000
        point.time_from_start.nanosec = (total_ms % 1000) * 1000000
        trajectory.points.append(point)

    action_client.get_logger().info('Sending whole-body trajectory via persistent stream...')
    future = action_client.send_stream_goal(trajectory)
    rclpy.spin_until_future_complete(action_client, future)
    goal_handle = future.result()

    if goal_handle.accepted:
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(action_client, get_result_future)
        result = get_result_future.result().result
        action_client.get_logger().info(f'Trajectory streaming completed: finish_code={result.finish_code}')

    time.sleep(2.0)

    # 5. Disable persistent Command Stream
    action_client.toggle_stream(False)

    time.sleep(1.0)
    action_client.get_logger().info('Returning to Zero Pose.')
    action_client.go_to_zero_pose()

    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
 
