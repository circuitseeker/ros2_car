# ROS2 Car Project

This is a simple ROS2 Humble project for a car robot with a motor and LED. The project includes URDF description and MoveIt configuration.

## Project Structure

- `src/car_description`: Contains the URDF model and visualization launch files
- `src/car_moveit_config`: Contains the MoveIt configuration for the car robot

## Building the Project

```bash
# Navigate to the workspace
cd ros2_car

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

## Running the Project

### Visualize the Car in RViz

```bash
ros2 launch car_description display.launch.py
```

### Run with MoveIt

```bash
ros2 launch car_moveit_config moveit.launch.py
```

## Features

- Simple car model with differential drive
- Motor and LED components
- MoveIt integration for motion planning
- RViz visualization

## Dependencies

- ROS2 Humble
- MoveIt 2
- URDF
- Xacro
- RViz2 