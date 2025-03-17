#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # Create subscribers for the cmd_vel topic
        self.cmd_vel_subscriber = self.create_subscription(
            Twist,
            '/car/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Create publishers for the wheel velocities
        self.left_wheel_pub = self.create_publisher(
            Float64,
            '/car/left_wheel_velocity',
            10
        )
        
        self.right_wheel_pub = self.create_publisher(
            Float64,
            '/car/right_wheel_velocity',
            10
        )
        
        self.get_logger().info('Motor Control Node started')
    
    def cmd_vel_callback(self, msg):
        # Extract linear and angular velocities
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calculate wheel velocities using differential drive kinematics
        # Assuming wheel_separation = 0.25 (from URDF)
        wheel_separation = 0.25
        
        # Calculate wheel velocities
        left_wheel_velocity = linear_x - (angular_z * wheel_separation / 2.0)
        right_wheel_velocity = linear_x + (angular_z * wheel_separation / 2.0)
        
        # Create messages
        left_msg = Float64()
        left_msg.data = left_wheel_velocity
        
        right_msg = Float64()
        right_msg.data = right_wheel_velocity
        
        # Publish messages
        self.left_wheel_pub.publish(left_msg)
        self.right_wheel_pub.publish(right_msg)
        
        # Log velocities
        self.get_logger().info(f'Left wheel velocity: {left_wheel_velocity:.2f}, Right wheel velocity: {right_wheel_velocity:.2f}')

def main(args=None):
    rclpy.init(args=args)
    
    motor_control_node = MotorControlNode()
    
    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_control_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 