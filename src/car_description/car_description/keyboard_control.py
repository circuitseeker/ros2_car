#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import time

# Define key mappings
key_mapping = {
    'w': [1.0, 0.0],   # Forward
    's': [-1.0, 0.0],  # Backward
    'a': [0.0, 1.0],   # Left
    'd': [0.0, -1.0],  # Right
    'q': [0.0, 0.0]    # Stop
}

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        # Create publisher for cmd_vel
        self.cmd_vel_publisher = self.create_publisher(Twist, '/car/cmd_vel', 10)
        
        # Set up parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        
        self.get_logger().info('Keyboard Control Node started')
        self.get_logger().info('Use the following keys to control the car:')
        self.get_logger().info('w: Forward')
        self.get_logger().info('s: Backward')
        self.get_logger().info('a: Left')
        self.get_logger().info('d: Right')
        self.get_logger().info('q: Stop')
        self.get_logger().info('Press Ctrl+C to exit')
    
    def get_key(self):
        # Get a single keypress from the terminal
        settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ''
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def publish_cmd_vel(self, linear, angular):
        # Create Twist message
        msg = Twist()
        msg.linear.x = linear * self.linear_speed
        msg.angular.z = angular * self.angular_speed
        
        # Publish message
        self.cmd_vel_publisher.publish(msg)
        
        # Log command
        self.get_logger().info(f'Sending command - Linear: {msg.linear.x:.2f}, Angular: {msg.angular.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    
    keyboard_control_node = KeyboardControlNode()
    
    try:
        while True:
            key = keyboard_control_node.get_key()
            
            if key == '\x03':  # Ctrl+C
                break
            
            if key in key_mapping:
                linear, angular = key_mapping[key]
                keyboard_control_node.publish_cmd_vel(linear, angular)
            
            # Small delay to prevent flooding
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the car before exiting
        keyboard_control_node.publish_cmd_vel(0.0, 0.0)
        keyboard_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 