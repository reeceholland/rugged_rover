from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Locate the config file relative to the installed package
    config_path = os.path.join(
        get_package_share_directory('sabertooth_motor_driver'),
        'config',
        'motor_driver.yml'
    )

    return LaunchDescription([
        Node(
            package='sabertooth_motor_driver',
            executable='sabertooth_driver_node',
            name='sabertooth_driver_node',
            parameters=[config_path],
            arguments=["--ros-args", "--log-level", "INFO"],
            output='screen'
        )
    ])