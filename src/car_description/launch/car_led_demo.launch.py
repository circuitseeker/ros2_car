import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package directories
    car_description_pkg = get_package_share_directory('car_description')
    
    # Set up paths
    urdf_path = os.path.join(car_description_pkg, 'urdf/car.urdf.xacro')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Create robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )
    
    # Run the simple_control.py script directly
    simple_control_script = ExecuteProcess(
        cmd=['python3', os.path.join(car_description_pkg, 'scripts/simple_control.py')],
        output='screen'
    )
    
    # Run the car_control_demo.py script for keyboard control
    car_control_demo_script = ExecuteProcess(
        cmd=['python3', os.path.join(car_description_pkg, 'scripts/car_control_demo.py')],
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        simple_control_script,
        car_control_demo_script
    ]) 