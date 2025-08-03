from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Construct the path to the URDF file using xacro
    xacro_path = PathJoinSubstitution([
        FindPackageShare('rugged_rover_robot_description'),
        'urdf',
        'rugged_rover.urdf.xacro'
    ])

    # Construct the path to the controller configuration file
    controller_config = PathJoinSubstitution([
        FindPackageShare('rugged_rover_robot_description'),
        'config',
        'controller_config.yaml'
    ])

    # Create the launch description
    return LaunchDescription([
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            arguments=["serial", "--dev", "/dev/ttyACM0"],
            output="screen"
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_path]),
                    value_type=str
                )
            }]
        ),
        # RViz2 for visualization
        # This node launches RViz2 with a predefined configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('rugged_rover_robot_description'),
                'rviz',
                'view.rviz'
            ])],
            output='screen'
        ),

        # ROS 2 Control Node
        # This node initializes the ROS 2 control framework with the robot description and controller configuration
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                {'robot_description': ParameterValue(
                    Command(['xacro ', xacro_path]),
                    value_type=str
                )},
                controller_config
            ],
            output='screen'
        ),

        # Timer action to spawn controllers after a delay
        # This action waits for 4 seconds before spawning the joint state broadcaster and diff drive controller
        TimerAction(
            period=1.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
                    output="screen"
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
                    output="screen"
                )
            ]
        )
    ])

