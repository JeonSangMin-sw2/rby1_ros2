#!/usr/bin/env python3
"""
Stream Command Example
======================
Demonstrates persistent command stream longevity and responsiveness by enabling
persistent stream mode and then sending standard joint command goals (Rby1JointCommand)
sequentially with varying wait intervals (0.5s to 3s).

Because persistent stream control is enabled, the driver's command stream hold time
is kept active (up to 10 minutes), allowing regular motion commands to be received
safely even after significant delay between inputs.

Scenario:
  1. Ensure the robot is powered and enabled.
  2. Call '/stream_control' service to enable a persistent stream.
  3. Loop through varying wait intervals (0.5s, 1s, 1.5s, 2s, 2.5s, 3s):
     - Send standard JointCommand to Zero Pose.
     - Wait for completion.
     - Wait for the loop's interval.
     - Send standard JointCommand to Ready Pose.
     - Wait for completion.
     - Wait for the loop's interval.
  4. Keep the stream active after the loop completes.
  5. Upon node shutdown (e.g. Ctrl+C), safely deactivate the stream.
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rby1_msgs.action import Rby1JointCommand
from rby1_msgs.msg import JointCommand, RobotState
from rby1_msgs.srv import StateOnOff

class StreamCommand(Node):
    def __init__(self):
        super().__init__('stream_command')
        self._action_client = ActionClient(self, Rby1JointCommand, 'robot_joint')
        self.power_client = self.create_client(StateOnOff, 'robot_power')
        self.servo_client = self.create_client(StateOnOff, 'robot_servo')
        self.stream_control_client = self.create_client(StateOnOff, 'stream_control')
        
        self.state_sub = self.create_subscription(RobotState, 'robot_state', self.state_callback, 10)
        self.control_state = None

        # Build joint ready pose values
        self.zero_torso = [0.0] * 6
        self.zero_right = [0.0] * 7
        self.zero_left  = [0.0] * 7
        self.zero_head  = [0.0] * 2

        self.ready_torso = [0.0] * 6
        self.ready_right = [0.0, -0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
        self.ready_left  = [0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0]
        self.ready_head  = [0.0, 0.0]

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
        rclpy.spin_once(self, timeout_sec=0.5)
        if self.control_state in [2, 3]:
            self.get_logger().info('Robot is already enabled.')
            return True

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
        
        time.sleep(2.0)
            
        self.get_logger().info('Sending Servo ON request...')
        self.servo_client.wait_for_service()
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if not future.result().success:
            self.get_logger().error(f'Failed to servo on: {future.result().message}')
            return False
            
        if self.wait_for_state([2, 3], timeout=15.0):
            self.get_logger().info('Robot is ready.')
            time.sleep(1.0)
            return True
        else:
            self.get_logger().error(f'Timed out waiting for robot to enable.')
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

    def send_joint_goal(self, torso, right_arm, left_arm, head, minimum_time=3.0) -> bool:
        self._action_client.wait_for_server()
        
        goal_msg = Rby1JointCommand.Goal()
        
        if torso is not None:
            goal_msg.torso = JointCommand()
            goal_msg.torso.position = torso
            goal_msg.torso.minimum_time = minimum_time
            
        if right_arm is not None:
            goal_msg.right_arm = JointCommand()
            goal_msg.right_arm.position = right_arm
            goal_msg.right_arm.minimum_time = minimum_time
            
        if left_arm is not None:
            goal_msg.left_arm = JointCommand()
            goal_msg.left_arm.position = left_arm
            goal_msg.left_arm.minimum_time = minimum_time
            
        if head is not None:
            goal_msg.head = JointCommand()
            goal_msg.head.position = head
            goal_msg.head.minimum_time = minimum_time

        goal_msg.priority = 10
        
        self.get_logger().info('Sending joint command goal...')
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Joint command goal was rejected.')
            return False
            
        self.get_logger().info('Goal accepted. Waiting for execution to complete...')
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result().result
        return result.success

def main(args=None):
    rclpy.init(args=args)
    node = StreamCommand()

    # 1. Prepare Robot
    if not node.ensure_robot_ready():
        node.get_logger().error('Failed to prepare robot. Exiting.')
        return

    # 2. Enable Persistent Command Stream
    if not node.toggle_stream(True):
        node.get_logger().error('Failed to enable command stream. Exiting.')
        return

    intervals = [0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    try:
        # 3. Alternate between Zero Pose and Ready Pose using regular joint commands over persistent stream
        for idx, interval in enumerate(intervals):
            node.get_logger().info(f"\n==========================================")
            node.get_logger().info(f" Starting Loop {idx+1}/{len(intervals)} (Interval: {interval}s)")
            node.get_logger().info(f"==========================================")

            # Move to Zero Pose
            node.get_logger().info('Moving to Zero Pose...')
            if not node.send_joint_goal(node.zero_torso, node.zero_right, node.zero_left, node.zero_head, minimum_time=3.0):
                node.get_logger().error('Failed to move to Zero Pose.')
                break
            node.get_logger().info(f'Zero Pose reached. Waiting for interval {interval}s...')
            time.sleep(interval)

            # Move to Ready Pose
            node.get_logger().info('Moving to Ready Pose...')
            if not node.send_joint_goal(node.ready_torso, node.ready_right, node.ready_left, node.ready_head, minimum_time=3.0):
                node.get_logger().error('Failed to move to Ready Pose.')
                break
            node.get_logger().info(f'Ready Pose reached. Waiting for interval {interval}s...')
            time.sleep(interval)

        node.get_logger().info('\nAll motion loops completed successfully.')
        node.get_logger().info('Stream is kept ACTIVE as requested.')
        node.get_logger().info('Press Ctrl+C to shut down node and deactivate stream control safely.')
        
        # Keep spinning to hold stream active
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received. Shutting down...')
    finally:
        # 4. Safely deactivate stream on exit
        if rclpy.ok():
            node.get_logger().info('Cleaning up: Deactivating stream control...')
            node.toggle_stream(False)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
