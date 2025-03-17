import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('car_description')
    
    # Set up paths
    default_model_path = os.path.join(pkg_dir, 'urdf/car.urdf.xacro')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    model = LaunchConfiguration('model', default=default_model_path)
    
    # Include the headless display launch file
    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch/display_headless.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'model': model
        }.items()
    )
    
    # Create a simple teleop node for keyboard control
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        remappings=[('/cmd_vel', '/car/cmd_vel')]
    )
    
    # Create a simple LED control node
    led_control_node = Node(
        package='car_description',
        executable='led_control',
        name='led_control',
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        display_launch,
        teleop_node,
        led_control_node
    ]) 