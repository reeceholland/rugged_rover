from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Find package shares
    description_pkg = FindPackageShare("rugged_rover_robot_description")
    control_pkg = FindPackageShare("rugged_rover_control")

    # Paths
    xacro_path = PathJoinSubstitution([
        description_pkg,
        "urdf",
        "rugged_rover.urdf.xacro"
    ])

    controllers_yaml = PathJoinSubstitution([
        control_pkg,
        "config",
        "controllers.yaml"
    ])


    # Launch Description
    return LaunchDescription([
        # Micro-ROS Agent
        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            arguments=["serial", "--dev", "/dev/ttyACM0"],
            output="screen"
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{'robot_description': ParameterValue(
                    Command(['xacro ', xacro_path]),
                    value_type=str
                )}],
            output="screen"
        ),

        # ros2_control Node
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            parameters=[{'robot_description': ParameterValue(
                    Command(['xacro ', xacro_path]),
                    value_type=str
                )}, controllers_yaml],
            output="screen"
        ),

        # Joint State Broadcaster
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            output="screen"
        ),

        # Differential Drive Controller
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
            output="screen"
        )
    ])
