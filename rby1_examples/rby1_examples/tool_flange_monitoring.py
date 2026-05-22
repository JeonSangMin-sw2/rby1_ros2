#!/usr/bin/env python3
"""
Tool Flange Monitoring Example
==============================
A continuous monitoring node that subscribes to the split left and right
tool flange topics published by the RBY1 driver and prints formatted
sensor readings to the terminal at every update.

Displayed information per flange:
  - FT sensor: force [Fx, Fy, Fz] in N and torque [Tx, Ty, Tz] in N·m
  - IMU: gyroscope [rad/s] and accelerometer [m/s²]
  - Switch A state, output voltage [mV]
  - Digital inputs (A, B) and digital outputs (A, B)

Note:
  The driver must have 'publish_tool_flange_state: true' in
  driver_parameters.yaml for these topics to be available.

Run:
  ros2 run rby1_examples tool_flange_monitoring

Topics subscribed:
  - joint_states/tool_flange/left  (ToolFlangeState)
  - joint_states/tool_flange/right (ToolFlangeState)
"""
import rclpy
from rclpy.node import Node
from rby1_msgs.msg import ToolFlangeState

class ToolFlangeMonitoring(Node):
    def __init__(self):
        super().__init__('tool_flange_monitoring')
        self.get_logger().info('Initializing Tool Flange Monitoring...')
        
        # Subscribe to separate left and right tool flange topics
        self.left_sub = self.create_subscription(
            ToolFlangeState, '/joint_states/tool_flange/left', lambda msg: self.tf_callback(msg, 'Left Flange'), 10)
        self.right_sub = self.create_subscription(
            ToolFlangeState, '/joint_states/tool_flange/right', lambda msg: self.tf_callback(msg, 'Right Flange'), 10)

    def tf_callback(self, msg, side):
        self.get_logger().info(f'=== {side} Update ===')
        # Print F/T Sensor values
        force_str = ", ".join([f"{f:.3f}" for f in msg.ft_force])
        torque_str = ", ".join([f"{t:.3f}" for t in msg.ft_torque])
        self.get_logger().info(f'  FT Force: [{force_str}], FT Torque: [{torque_str}]')
        
        # Print IMU/Gyro/Accel
        gyro_str = ", ".join([f"{g:.3f}" for g in msg.gyro])
        accel_str = ", ".join([f"{a:.3f}" for a in msg.acceleration])
        self.get_logger().info(f'  Gyro: [{gyro_str}], Accel: [{accel_str}]')
        
        # Print I/O status and voltage
        self.get_logger().info(f'  Switch A: {msg.switch_a}, Output Voltage: {msg.output_voltage} mV')
        self.get_logger().info(f'  Digital Inputs: A={msg.digital_input_a}, B={msg.digital_input_b}')
        self.get_logger().info(f'  Digital Outputs: A={msg.digital_output_a}, B={msg.digital_output_b}')

def main(args=None):
    rclpy.init(args=args)
    monitor = ToolFlangeMonitoring()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass

    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
