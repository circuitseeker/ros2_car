#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
import time
import math
import Jetson.GPIO as GPIO

class SimpleControlNode(Node):
    def __init__(self):
        super().__init__('simple_control_node')
        
        # Create a publisher for joint states
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        
        # Subscribe to LED state
        self.led_subscriber = self.create_subscription(
            Bool,
            '/car/led_state',
            self.led_callback,
            10
        )
        
        # Create a timer to publish joint states
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Create a timer for LED control based on movement
        self.led_timer = self.create_timer(0.05, self.led_control_callback)
        
        # Initialize time
        self.start_time = time.time()
        
        # Set up GPIO for LED
        self.led_pin = 40
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.led_pin, GPIO.OUT)
        GPIO.output(self.led_pin, GPIO.LOW)
        
        # Movement tracking variables
        self.current_velocity = 0.0
        self.current_position = 0.0
        self.led_state = False
        self.is_moving = False
        
        self.get_logger().info('Simple Control Node started')
        self.get_logger().info(f'LED control initialized on GPIO pin {self.led_pin}')
    
    def led_callback(self, msg):
        # Manual override for LED state from topic
        if msg.data:
            self.led_state = True
            GPIO.output(self.led_pin, GPIO.HIGH)
            self.get_logger().info('LED state set to ON by topic')
        else:
            self.led_state = False
            GPIO.output(self.led_pin, GPIO.LOW)
            self.get_logger().info('LED state set to OFF by topic')
    
    def led_control_callback(self):
        # Control LED based on movement
        if self.is_moving:
            # Turn on LED when moving
            GPIO.output(self.led_pin, GPIO.HIGH)
            self.get_logger().info(f'LED ON (velocity: {self.current_velocity:.2f})')
        else:
            # Turn off LED when not moving
            GPIO.output(self.led_pin, GPIO.LOW)
            self.get_logger().info('LED OFF (not moving)')
    
    def timer_callback(self):
        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time
        
        # Calculate wheel positions (rotating continuously)
        wheel_position = math.fmod(elapsed_time, 2.0 * math.pi)
        
        # Calculate velocity (change in position)
        if hasattr(self, 'last_position'):
            self.current_velocity = (wheel_position - self.last_position) / 0.1  # 0.1 is the timer period
            if abs(self.current_velocity) > 5.0:  # Handle wrap-around
                if wheel_position < self.last_position:
                    self.current_velocity = (wheel_position + 2.0 * math.pi - self.last_position) / 0.1
                else:
                    self.current_velocity = (wheel_position - 2.0 * math.pi - self.last_position) / 0.1
        else:
            self.current_velocity = 1.0  # Initial velocity
            
        self.last_position = wheel_position
        self.current_position = wheel_position
        
        # Determine if the car is moving
        self.is_moving = abs(self.current_velocity) > 0.1
        
        # Create joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [wheel_position, wheel_position]
        msg.velocity = [self.current_velocity, self.current_velocity]
        msg.effort = [0.0, 0.0]
        
        # Publish message
        self.joint_state_publisher.publish(msg)
        
        # Log state
        self.get_logger().info(f'Wheel position: {wheel_position:.2f}, Velocity: {self.current_velocity:.2f}, Moving: {self.is_moving}')
    
    def destroy_node(self):
        # Clean up GPIO
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    simple_control_node = SimpleControlNode()
    
    try:
        rclpy.spin(simple_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        simple_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 