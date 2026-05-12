#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rby1_msgs.msg import PowerState, ToolFlangeState, TorqueVelocityState

class RobotStatusMonitor(Node):
    def __init__(self):
        super().__init__('robot_status_monitor')
        
        # Subscriptions
        self.power_sub = self.create_subscription(PowerState, 'joint_states/power_state', self.power_callback, 10)
        self.tf_sub = self.create_subscription(ToolFlangeState, 'joint_states/tool_flange_state', self.tf_callback, 10)
        self.tv_sub = self.create_subscription(TorqueVelocityState, 'joint_states/torque_velocity_state', self.tv_callback, 10)

    def power_callback(self, msg):
        self.get_logger().info('--- Power Status ---')
        self.get_logger().info(f'Battery: {msg.battery_voltage:.2f}V, {msg.battery_current:.2f}A ({msg.battery_level_percent:.1f}%)')
        self.get_logger().info(f'EMO: {msg.emo_state}, Power: {msg.power_state} ({msg.power_voltage:.2f}V)')

    def tf_callback(self, msg):
        self.get_logger().info('--- Tool Flange Status ---')
        self.get_logger().info(f'[Right] Volt: {msg.tool_flange_right_output_voltage}V, SwitchA: {msg.tool_flange_right_switch_a}')
        self.get_logger().info(f'[Right FT] Force: {msg.ft_force_right}, Torque: {msg.ft_torque_right}')
        self.get_logger().info(f'[Left] Volt: {msg.tool_flange_left_output_voltage}V, SwitchA: {msg.tool_flange_left_switch_a}')

    def tv_callback(self, msg):
        self.get_logger().info('--- Torque & Velocity Status ---')
        if len(msg.velocity) > 0:
            self.get_logger().info(f'Velocity (sample): {msg.velocity[:3]}...')
            self.get_logger().info(f'Torque (sample): {msg.torque[:3]}...')
        self.get_logger().info(f'Center of Mass: {msg.center_of_mass}')

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
