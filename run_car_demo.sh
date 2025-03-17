#!/bin/bash

# Change to the ROS2 car workspace
cd ~/ros2_car

# Build the project
colcon build
source install/setup.bash

# Run the car with full display (includes simple_control.py which handles LED)
export DISPLAY=:0
ros2 launch car_description car_full_display.launch.py 