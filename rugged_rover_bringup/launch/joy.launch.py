from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = FindPackageShare("rugged_rover_bringup")

    joy_config = PathJoinSubstitution([
        bringup_pkg,
        "config",
        "joy.yaml",
    ])

    joy_dev = LaunchConfiguration("joy_dev")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "joy_dev",
            default_value="0",
            description="Joystick device id passed to joy_node.",
        ),

        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/diff_drive_controller/cmd_vel",
            description="Velocity command topic for joystick teleop.",
        ),

        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[
                joy_config,
                {"device_id": joy_dev},
            ],
            output="screen",
        ),

        Node(
            package="teleop_twist_joy",
            executable="teleop_node",
            name="teleop_twist_joy_node",
            parameters=[joy_config],
            remappings=[
                ("/cmd_vel", cmd_vel_topic),
            ],
            output="screen",
        ),
    ])
