#!/usr/bin/env python3
"""
Robot Status Monitor
====================
A comprehensive diagnostic node that subscribes to all major state
topics published by the RBY1 driver and prints a formatted summary
to the terminal at each update cycle.

Displayed information:
  - Control Manager state (IDLE / ENABLE / EXECUTING / FAULT)
  - EMO (Emergency Stop) button state
  - Center of Mass position [x, y, z]
  - Brake engagement status for every joint
  - Tool flange connection status (left / right)
  - Battery voltage, current, and percentage
  - Tool flange sensor data: FT force/torque, IMU, digital I/O

Run:
  ros2 run rby1_examples robot_status_monitor

Topics subscribed:
  - joint_states/robot_state       (RobotState)
  - joint_states/battery_state     (sensor_msgs/BatteryState)
  - joint_states/tool_flange/left  (ToolFlangeState)
  - joint_states/tool_flange/right (ToolFlangeState)
"""
import rclpy
from rclpy.node import Node
from rby1_msgs.msg import RobotState, ToolFlangeState
from sensor_msgs.msg import BatteryState

class RobotStatusMonitor(Node):
    def __init__(self):
        super().__init__('robot_status_monitor')
        
        # Subscriptions to the modernized state topics
        self.state_sub = self.create_subscription(
            RobotState, 'joint_states/robot_state', self.robot_state_callback, 10)
        self.battery_sub = self.create_subscription(
            BatteryState, 'joint_states/battery_state', self.battery_callback, 10)
        
        self.tf_left_sub = self.create_subscription(
            ToolFlangeState, 'joint_states/tool_flange/left', lambda msg: self.tf_callback(msg, 'Left'), 10)
        self.tf_right_sub = self.create_subscription(
            ToolFlangeState, 'joint_states/tool_flange/right', lambda msg: self.tf_callback(msg, 'Right'), 10)

        # Verify robot state topic is active
        if not self.verify_topic_active('joint_states/robot_state', timeout=1.5):
            self.get_logger().error(
                "Error: Topic 'joint_states/robot_state' is not active!\n"
                "Please make sure the RBY1 ROS 2 driver is running."
            )
            import sys
            sys.exit(1)

    def verify_topic_active(self, topic_name, timeout=1.5):
        import time
        start_time = time.time()
        resolved_topic = self.resolve_topic_name(topic_name)
        while rclpy.ok():
            try:
                pub_info = self.get_publishers_info_by_topic(resolved_topic)
                if len(pub_info) > 0:
                    return True
            except Exception as e:
                pass
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                break
        return False

    def robot_state_callback(self, msg):
        self.get_logger().info('=== Robot State Update ===')
        # Print Control Manager State
        state_names = {0: "NONE", 1: "IDLE", 2: "ENABLE", 3: "EXECUTING", 4: "MAJOR_FAULT", 5: "MINOR_FAULT"}
        state_str = state_names.get(msg.control_manager_state, f"UNKNOWN ({msg.control_manager_state})")
        self.get_logger().info(f'  Control Manager State: {state_str}')
        self.get_logger().info(f'  EMO Pressed: {msg.emo_state}')
        self.get_logger().info(f'  Center of Mass: {list(msg.center_of_mass)}')
        
        # Print Brake States
        self.get_logger().info('  Brakes (true = engaged, false = released):')
        self.get_logger().info(f'    Torso: {list(msg.brake_state.torso)}')
        self.get_logger().info(f'    Right Arm: {list(msg.brake_state.right_arm)}')
        self.get_logger().info(f'    Left Arm: {list(msg.brake_state.left_arm)}')
        self.get_logger().info(f'    Head: {list(msg.brake_state.head)}')
        
        # Print Tool Flange connection flags
        if len(msg.tool_flange_state) >= 2:
            self.get_logger().info(f'  Tool Flanges Connected: Left={msg.tool_flange_state[0]}, Right={msg.tool_flange_state[1]}')

    def battery_callback(self, msg):
        self.get_logger().info('=== Battery Status ===')
        self.get_logger().info(f'  Voltage: {msg.voltage:.2f} V')
        self.get_logger().info(f'  Current: {msg.current:.2f} A')
        self.get_logger().info(f'  Percentage: {msg.percentage * 100.0:.1f} %')

    def tf_callback(self, msg, side):
        self.get_logger().info(f'=== Tool Flange [{side}] ===')
        self.get_logger().info(f'  FT Force: {list(msg.ft_force)}, FT Torque: {list(msg.ft_torque)}')
        self.get_logger().info(f'  Gyro: {list(msg.gyro)}, Accel: {list(msg.acceleration)}')
        self.get_logger().info(f'  Switch A: {msg.switch_a}, Voltage: {msg.output_voltage} mV')
        self.get_logger().info(f'  Digital Inputs: A={msg.digital_input_a}, B={msg.digital_input_b}')
        self.get_logger().info(f'  Digital Outputs: A={msg.digital_output_a}, B={msg.digital_output_b}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
