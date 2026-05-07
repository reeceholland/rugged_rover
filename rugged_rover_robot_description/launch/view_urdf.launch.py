from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = FindPackageShare("rugged_rover_robot_description")

    xacro_path = PathJoinSubstitution([
        description_pkg,
        "urdf",
        "rugged_rover.urdf.xacro",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path]),
            value_type=str,
        )
    }

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            output="screen",
        ),

        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                PathJoinSubstitution([
                    description_pkg,
                    "rviz",
                    "view.rviz",
                ]),
            ],
            output="screen",
        ),
    ])
