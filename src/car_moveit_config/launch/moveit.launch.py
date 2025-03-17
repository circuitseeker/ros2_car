import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package directories
    car_description_pkg = get_package_share_directory('car_description')
    car_moveit_config_pkg = get_package_share_directory('car_moveit_config')
    
    # Set up paths
    urdf_path = os.path.join(car_description_pkg, 'urdf/car.urdf.xacro')
    rviz_config_path = os.path.join(car_moveit_config_pkg, 'config/moveit.rviz')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz'
    )
    
    # Include the robot state publisher launch file
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(car_description_pkg, 'launch/display.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'model': urdf_path,
            'rviz_config': rviz_config_path
        }.items()
    )
    
    # Create MoveIt move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_path]),
            'allow_trajectory_execution': True,
            'publish_robot_description_semantic': True
        }]
    )
    
    # Create RViz node with MoveIt configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path],
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
        condition=IfCondition(use_rviz)
    )
    
    # Return launch description
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        robot_state_publisher_launch,
        move_group_node,
        rviz_node
    ]) 