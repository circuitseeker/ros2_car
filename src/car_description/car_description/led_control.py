#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import time

class LEDControlNode(Node):
    def __init__(self):
        super().__init__('led_control_node')
        
        # Create a publisher for the LED state
        self.led_publisher = self.create_publisher(Bool, '/car/led_state', 10)
        
        # Create a timer to toggle the LED state every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        # Initialize LED state
        self.led_state = False
        
        self.get_logger().info('LED Control Node started')
    
    def timer_callback(self):
        # Toggle LED state
        self.led_state = not self.led_state
        
        # Create message
        msg = Bool()
        msg.data = self.led_state
        
        # Publish message
        self.led_publisher.publish(msg)
        
        # Log state
        self.get_logger().info(f'LED state: {"ON" if self.led_state else "OFF"}')

def main(args=None):
    rclpy.init(args=args)
    
    led_control_node = LEDControlNode()
    
    try:
        rclpy.spin(led_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        led_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 