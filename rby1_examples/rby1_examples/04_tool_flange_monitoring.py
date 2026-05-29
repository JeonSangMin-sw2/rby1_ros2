#!/usr/bin/env python3
"""
Tool Flange Power Control & Monitoring Example
==============================================
A unified utility that demonstrates both tool flange power control and continuous 
state monitoring.

Upon startup, this example:
  1. Verifies that the tool flange topics are active.
  2. Calls the 'tool_flange_power' service to enable the 12V output rail.
  3. Continuous subscribes to the split left and right tool flange topics and prints 
     formatted sensor readings (F/T sensors, IMU, IOs, voltage) to the terminal.
  4. On node shutdown or termination, cleanly turns off the tool flange power rail.

Note:
  The driver must have 'publish_tool_flange_state: true' in
  driver_parameters.yaml for these topics to be available.

Run:
  ros2 run rby1_examples 04_tool_flange_monitoring

Services used:
  - tool_flange_power (StateOnOff)

Topics subscribed:
  - joint_states/tool_flange/left  (ToolFlangeState)
  - joint_states/tool_flange/right (ToolFlangeState)
"""
import sys
import time
import rclpy
from rclpy.node import Node
from rby1_msgs.msg import ToolFlangeState
from rby1_msgs.srv import StateOnOff

class ToolFlangeMonitoring(Node):
    def __init__(self):
        super().__init__('tool_flange_monitoring')
        self.get_logger().info('Initializing Tool Flange Power Control & Monitoring...')
        
        # Service client for power control
        self.tool_flange_client = self.create_client(StateOnOff, 'tool_flange_power')

        # Subscribe to separate left and right tool flange topics
        self.left_sub = self.create_subscription(
            ToolFlangeState, 'joint_states/tool_flange/left', lambda msg: self.tf_callback(msg, 'Left Flange'), 10)
        self.right_sub = self.create_subscription(
            ToolFlangeState, 'joint_states/tool_flange/right', lambda msg: self.tf_callback(msg, 'Right Flange'), 10)

        # Verify tool flange topics are active
        if not self.verify_topic_active('joint_states/tool_flange/left', timeout=1.5):
            self.get_logger().error(
                "Error: Topic 'joint_states/tool_flange/left' is not active!\n"
                "Please make sure the RBY1 ROS 2 driver is running and 'publish_tool_flange_state: true' is set in driver_parameters.yaml."
            )
            sys.exit(1)

        # Call service to turn on 12V tool flange power
        self.send_tool_flange_request(True, '12')

    def verify_topic_active(self, topic_name, timeout=1.5):
        start_time = time.time()
        resolved_topic = self.resolve_topic_name(topic_name)
        while rclpy.ok():
            try:
                pub_info = self.get_publishers_info_by_topic(resolved_topic)
                if len(pub_info) > 0:
                    return True
            except Exception:
                pass
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                break
        return False

    def send_tool_flange_request(self, state: bool, parameters: str) -> bool:
        if not self.tool_flange_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service 'tool_flange_power' not available after 5 seconds.")
            return False

        req = StateOnOff.Request()
        req.state = state
        req.parameters = parameters

        op = 'ON' if state else 'OFF'
        self.get_logger().info(f"Calling tool_flange_power: state={state}, params='{parameters}'...")
        future = self.tool_flange_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error("No response from tool_flange_power service.")
            return False

        if result.success:
            self.get_logger().info(f"tool_flange_power {op} OK: {result.message}")
        else:
            self.get_logger().error(f"tool_flange_power {op} FAILED: {result.message}")
        return result.success

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
        voltage_v = msg.output_voltage / 1000.0  # mV -> V
        self.get_logger().info(f'  Switch A: {msg.switch_a}, Output Voltage: {voltage_v:.2f} V')
        self.get_logger().info(f'  Digital Inputs: A={msg.digital_input_a}, B={msg.digital_input_b}')
        self.get_logger().info(f'  Digital Outputs: A={msg.digital_output_a}, B={msg.digital_output_b}')

def main(args=None):
    rclpy.init(args=args)
    monitor = ToolFlangeMonitoring()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        # Turn off power before shutdown
        try:
            monitor.get_logger().info('Shutting down: turning off tool flange power...')
            monitor.send_tool_flange_request(False, '')
        except Exception as e:
            print(f"Error during shutdown tool flange power off: {e}")
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
