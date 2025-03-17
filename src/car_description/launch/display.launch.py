import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('car_description')
    
    # Set up paths
    default_model_path = os.path.join(pkg_dir, 'urdf/car.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_dir, 'config/car.rviz')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    model = LaunchConfiguration('model', default=default_model_path)
    rviz_config = LaunchConfiguration('rviz_config', default=default_rviz_config_path)
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_model = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Path to the robot URDF file'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Path to the RViz config file'
    )
    
    # Create robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', model])
        }]
    )
    
    # Create joint state publisher node
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )
    
    # Create joint state publisher GUI node
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # Create RViz node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    # Return launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_model,
        declare_rviz_config,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        rviz
    ]) 