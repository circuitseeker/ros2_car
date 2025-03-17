import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Get the package directories
    car_description_pkg = get_package_share_directory('car_description')
    car_moveit_config_pkg = get_package_share_directory('car_moveit_config')
    
    # Set up paths
    urdf_path = os.path.join(car_description_pkg, 'urdf/car.urdf.xacro')
    srdf_path = os.path.join(car_moveit_config_pkg, 'config/car.srdf')
    rviz_config_path = os.path.join(car_moveit_config_pkg, 'config/moveit.rviz')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Load SRDF file as string
    with open(srdf_path, 'r') as f:
        srdf_content = f.read()
    
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
    
    # Create MoveIt move_group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', urdf_path]),
            'robot_description_semantic': ParameterValue(srdf_content, value_type=str),
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
        }]
    )
    
    # Return launch description
    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        joint_state_publisher,
        joint_state_publisher_gui,
        move_group_node,
        rviz_node
    ]) 