#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class GateSubscriber(Node):
    def __init__(self):
        super().__init__('gate_subscriber')
        self.subscription = self.create_subscription(
            String,
            'gate_info',
            self.listener_callback,
            10)
        self.subscription  # prevent unused var warning

    def listener_callback(self, msg):
        # Parse the message
        data = dict(item.split(':') for item in msg.data.split(','))
        self.get_logger().info(f'Gate Info - Assigned Color: {data["assigned_color"]}')
        self.get_logger().info(f'Distance: {float(data["distance"]):.1f} cm')
        self.get_logger().info(f'Offset X: {float(data["offset_x"]):.2f}, Offset Y: {float(data["offset_y"]):.2f}')
        self.get_logger().info(f'Gate Center - X: {int(data["cx"])}, Y: {int(data["cy"])}')
        self.get_logger().info(f'Command: {data["command"]}')
        self.get_logger().info('---')

def main(args=None):
    rclpy.init(args=args)
    gate_subscriber = GateSubscriber()
    rclpy.spin(gate_subscriber)
    gate_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
