#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import sys
import termios
import tty
import threading
import time

# Key mapping
key_mapping = {
    'w': [1.0, 0.0],    # Forward
    's': [-1.0, 0.0],   # Backward
    'a': [0.0, 1.0],    # Left
    'd': [0.0, -1.0],   # Right
    'q': [0.0, 0.0],    # Stop
}

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')
        
        # Create publishers
        self.velocity_publisher = self.create_publisher(Twist, '/car/cmd_vel', 10)
        self.led_publisher = self.create_publisher(Bool, '/car/led_state', 10)
        
        # Initialize variables
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.led_state = False
        
        # Create a timer for publishing velocity commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Keyboard Control Node started')
        self.get_logger().info('Use the following keys to control the car:')
        self.get_logger().info('  w: Forward')
        self.get_logger().info('  s: Backward')
        self.get_logger().info('  a: Left')
        self.get_logger().info('  d: Right')
        self.get_logger().info('  q: Stop')
        self.get_logger().info('  l: Toggle LED')
        self.get_logger().info('  Ctrl+C: Quit')
    
    def timer_callback(self):
        # Create and publish velocity command
        twist = Twist()
        twist.linear.x = self.linear_speed
        twist.angular.z = self.angular_speed
        
        self.velocity_publisher.publish(twist)
        
        # Log state
        self.get_logger().info(f'Linear: {self.linear_speed:.1f}, Angular: {self.angular_speed:.1f}, LED: {self.led_state}')

def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def keyboard_thread(node):
    # Save terminal settings
    settings = termios.tcgetattr(sys.stdin)
    
    try:
        while True:
            # Get key press
            key = get_key(settings)
            
            # Check for Ctrl+C
            if key == '\x03':
                break
                
            # Toggle LED with 'l' key
            if key == 'l':
                node.led_state = not node.led_state
                msg = Bool()
                msg.data = node.led_state
                node.led_publisher.publish(msg)
                node.get_logger().info(f'LED toggled to {node.led_state}')
                continue
                
            # Update velocity based on key press
            if key in key_mapping:
                node.linear_speed, node.angular_speed = key_mapping[key]
                
                # If stopping, turn off LED
                if key == 'q':
                    node.led_state = False
                    msg = Bool()
                    msg.data = False
                    node.led_publisher.publish(msg)
    except Exception as e:
        node.get_logger().error(f'Error in keyboard thread: {e}')
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

def main(args=None):
    rclpy.init(args=args)
    
    node = KeyboardControlNode()
    
    # Create and start keyboard thread
    kb_thread = threading.Thread(target=keyboard_thread, args=(node,))
    kb_thread.daemon = True
    kb_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 