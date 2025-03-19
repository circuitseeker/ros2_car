#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
import numpy as np
import time
import math
import Jetson.GPIO as GPIO

class HandDetectionControlNode(Node):
    def __init__(self):
        super().__init__('hand_detection_control_node')
        
        # Initialize camera
        self.init_camera()
        
        # Initialize MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            min_detection_confidence=0.7
        )
        
        # Initialize OpenCV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.led_state_publisher = self.create_publisher(Bool, '/car/led_state', 10)
        self.image_publisher = self.create_publisher(Image, '/camera/processed_image', 10)
        
        # Timer for camera capture and processing
        self.camera_timer = self.create_timer(0.1, self.camera_timer_callback)
        
        # Timer for publishing joint states
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Set up GPIO for LED
        self.led_pin = 40
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.led_pin, GPIO.OUT)
        GPIO.output(self.led_pin, GPIO.LOW)
        
        # Car control variables
        self.target_velocity = 5.56  # 20 km/h in m/s
        self.current_velocity = 0.0
        self.current_position = 0.0
        self.start_time = time.time()
        self.hand_detected = False
        self.stopping_distance = 2.0  # meters
        self.max_deceleration = 2.0  # m/s^2
        
        self.get_logger().info('Hand Detection Control Node started')

    def init_camera(self):
        # Try to initialize the camera
        self.cap = None
        camera_index = 1  # Specifically use index 1 for Logitech camera
        
        self.get_logger().info(f'Initializing Logitech camera on index {camera_index}')
        cap = cv2.VideoCapture(camera_index)
        
        if cap.isOpened():
            self.get_logger().info(f'Successfully opened camera {camera_index}')
            
            # Set camera properties for Logitech C270
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 30)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, 128)
            cap.set(cv2.CAP_PROP_CONTRAST, 32)
            cap.set(cv2.CAP_PROP_SATURATION, 64)
            cap.set(cv2.CAP_PROP_GAIN, 64)
            
            # Try to read a test frame
            ret, frame = cap.read()
            if ret:
                self.get_logger().info(f'Successfully read frame from camera {camera_index}. Frame shape: {frame.shape}')
                self.cap = cap
                self.camera_index = camera_index
            else:
                self.get_logger().error(f'Could not read frame from camera {camera_index}')
                cap.release()
        else:
            self.get_logger().error(f'Failed to open camera {camera_index}')
        
        if self.cap is None:
            raise RuntimeError('Failed to initialize Logitech camera')
    
    def camera_timer_callback(self):
        if not self.cap or not self.cap.isOpened():
            self.get_logger().error('Camera is not available')
            return
        
        try:
            # Read frame from camera
            ret, frame = self.cap.read()
            if not ret:
                self.get_logger().error('Failed to read frame from camera')
                return
            
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Process the image and detect hands
            results = self.hands.process(rgb_image)
            
            # Draw hand landmarks on the image
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    for landmark in hand_landmarks.landmark:
                        h, w, _ = frame.shape
                        cx, cy = int(landmark.x * w), int(landmark.y * h)
                        cv2.circle(frame, (cx, cy), 5, (0, 255, 0), -1)
            
            # Check if hands are detected
            previous_hand_state = self.hand_detected
            self.hand_detected = results.multi_hand_landmarks is not None
            
            # If hand detection state changed, log it
            if self.hand_detected != previous_hand_state:
                if self.hand_detected:
                    self.get_logger().info('Hand detected - initiating stop sequence')
                    # Set LED to ON
                    GPIO.output(self.led_pin, GPIO.HIGH)
                    led_msg = Bool()
                    led_msg.data = True
                    self.led_state_publisher.publish(led_msg)
                else:
                    self.get_logger().info('No hand detected - resuming normal operation')
                    # Set LED to OFF
                    GPIO.output(self.led_pin, GPIO.LOW)
                    led_msg = Bool()
                    led_msg.data = False
                    self.led_state_publisher.publish(led_msg)
            
            # Publish processed image
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_publisher.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'Error converting/publishing image: {str(e)}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {str(e)}')
    
    def timer_callback(self):
        # Calculate elapsed time
        elapsed_time = time.time() - self.start_time
        
        # Update velocity based on hand detection
        if self.hand_detected:
            # Gradually decrease velocity
            decel = min(self.max_deceleration, self.current_velocity / 0.1)  # Ensure smooth deceleration
            self.current_velocity = max(0.0, self.current_velocity - decel * 0.1)
        else:
            # Gradually increase velocity to target
            if self.current_velocity < self.target_velocity:
                self.current_velocity = min(
                    self.target_velocity,
                    self.current_velocity + 2.0 * 0.1  # Accelerate at 2 m/s^2
                )
        
        # Update position
        self.current_position += self.current_velocity * 0.1
        wheel_position = math.fmod(self.current_position, 2.0 * math.pi)
        
        # Create and publish joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['left_wheel_joint', 'right_wheel_joint']
        msg.position = [wheel_position, wheel_position]
        msg.velocity = [self.current_velocity, self.current_velocity]
        msg.effort = [0.0, 0.0]
        self.joint_state_publisher.publish(msg)
        
        # Log current state
        self.get_logger().info(
            f'Velocity: {self.current_velocity:.2f} m/s ({self.current_velocity * 3.6:.1f} km/h), '
            f'Hand detected: {self.hand_detected}'
        )
    
    def destroy_node(self):
        # Release camera
        if self.cap:
            self.cap.release()
        # Clean up GPIO
        GPIO.cleanup()
        # Release MediaPipe resources
        self.hands.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = HandDetectionControlNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {str(e)}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 