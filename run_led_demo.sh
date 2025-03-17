#!/bin/bash

# Change to the ROS2 car workspace
cd ~/ros2_car

# Build the project
colcon build
source install/setup.bash

# Run the car with LED demo
export DISPLAY=:0
ros2 launch car_description car_led_demo.launch.py 