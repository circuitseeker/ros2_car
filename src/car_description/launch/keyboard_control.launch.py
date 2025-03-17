import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Create keyboard control node
    keyboard_control_node = Node(
        package='car_description',
        executable='keyboard_control',
        name='keyboard_control',
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        keyboard_control_node
    ]) 