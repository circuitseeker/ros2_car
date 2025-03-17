import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Get the package directory
    car_description_pkg = get_package_share_directory('car_description')
    
    # Run the LED blink script directly
    led_blink_script = ExecuteProcess(
        cmd=['python3', os.path.join(car_description_pkg, 'scripts/led_blink.py')],
        output='screen'
    )
    
    # Return launch description
    return LaunchDescription([
        led_blink_script
    ]) 