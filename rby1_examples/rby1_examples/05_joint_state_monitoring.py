#!/usr/bin/env python3
"""
Joint State Monitoring Example
==============================
A lightweight diagnostic node that subscribes to the per-component
JointState topics published by the RBY1 driver and prints the first
few joint positions to the terminal in real time.

This is useful for verifying that the driver is publishing joint
state data and that the robot's encoders are working correctly.

Run:
  ros2 run rby1_examples joint_state_monitoring

Topics subscribed:
  - joint_states/torso      (sensor_msgs/JointState)
  - joint_states/right_arm  (sensor_msgs/JointState)
  - joint_states/left_arm   (sensor_msgs/JointState)
  - joint_states/head       (sensor_msgs/JointState)
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStateMonitoring(Node):
    def __init__(self):
        super().__init__('joint_state_monitoring')
        self.get_logger().info('Initializing Joint State Monitoring...')
        
        # Subscribe to standard JointState topics for different components
        self.torso_sub = self.create_subscription(
            JointState, 'joint_states/torso', lambda msg: self.joint_state_callback(msg, 'Torso'), 10)
        self.right_arm_sub = self.create_subscription(
            JointState, 'joint_states/right_arm', lambda msg: self.joint_state_callback(msg, 'Right Arm'), 10)
        self.left_arm_sub = self.create_subscription(
            JointState, 'joint_states/left_arm', lambda msg: self.joint_state_callback(msg, 'Left Arm'), 10)
        self.head_sub = self.create_subscription(
            JointState, 'joint_states/head', lambda msg: self.joint_state_callback(msg, 'Head'), 10)

    def joint_state_callback(self, msg, part_name):
        # We only print the first 3 joint positions to avoid terminal flooding
        if len(msg.position) >= 3:
            positions_str = ", ".join([f"{p:.3f}" for p in msg.position[:3]])
            self.get_logger().info(f'[{part_name}] First 3 joints: [{positions_str}]')
        elif len(msg.position) > 0:
            positions_str = ", ".join([f"{p:.3f}" for p in msg.position])
            self.get_logger().info(f'[{part_name}] Joints: [{positions_str}]')

def main(args=None):
    rclpy.init(args=args)
    monitor = JointStateMonitoring()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass

    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
