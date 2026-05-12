#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rby1_msgs.msg import AllMotorState
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

class StateMonitoring(Node):
    def __init__(self):
        super().__init__('state_monitoring')
        self.get_logger().info('Initializing State Monitoring...')
        
        self.active_sub = None
        self.topic_type = None
        
        # Check available topics
        self.timer = self.create_timer(1.0, self.check_topics)
        self.searching = True

    def check_topics(self):
        if not self.searching:
            return

        topic_names_and_types = self.get_topic_names_and_types()
        
        # 1. Try AllMotorState (Custom)
        all_motor_topic = '/joint_states/all_motor_state'
        if any(all_motor_topic == name for name, types in topic_names_and_types):
            self.get_logger().info(f'Found custom topic: {all_motor_topic}. Monitoring...')
            self.active_sub = self.create_subscription(
                AllMotorState, all_motor_topic, self.all_motor_callback, 10)
            self.topic_type = 'AllMotorState'
            self.searching = False
            self.timer.cancel()
            return

        # 2. Try Standard JointState (fallback)
        joint_state_topics = [
            '/joint_states/torso', 
            '/joint_states/right_arm', 
            '/joint_states/left_arm', 
            '/joint_states/head'
        ]
        
        found_any = False
        for topic in joint_state_topics:
            if any(topic == name for name, types in topic_names_and_types):
                self.get_logger().info(f'Found standard topic: {topic}. Monitoring...')
                self.create_subscription(JointState, topic, 
                                          lambda msg, t=topic: self.joint_state_callback(msg, t), 10)
                found_any = True
        
        if found_any:
            self.topic_type = 'JointState'
            self.searching = False
            self.timer.cancel()
            return

        self.get_logger().warn('Waiting for joint state topics...')

    def all_motor_callback(self, msg):
        self.get_logger().info('--- All Motor State ---')
        self.get_logger().info(f'Torso: {msg.torso[:3]}...')
        self.get_logger().info(f'Right Arm: {msg.right_arm[:3]}...')
        self.get_logger().info(f'Left Arm: {msg.left_arm[:3]}...')

    def joint_state_callback(self, msg, topic_name):
        # Only print a subset to avoid flooding
        if 'right_arm' in topic_name:
            self.get_logger().info(f'[{topic_name}] First 3 joints: {msg.position[:3]}')

def main(args=None):
    rclpy.init(args=args)
    monitor = StateMonitoring()
    
    # Run for a bit to see if we find topics
    start_time = monitor.get_clock().now()
    while monitor.searching and (monitor.get_clock().now() - start_time).nanoseconds < 10e9: # 10s timeout
        rclpy.spin_once(monitor, timeout_sec=0.1)
    
    if monitor.searching:
        monitor.get_logger().error('Error: No suitable joint state topics found within 10 seconds.')
        monitor.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass

    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
