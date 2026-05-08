from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = FindPackageShare("rugged_rover_bringup")
    nav2_pkg = FindPackageShare("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    nav2_params = PathJoinSubstitution([
        bringup_pkg,
        "config",
        "nav2_params.yaml",
    ])

    navigation_launch = PathJoinSubstitution([
        nav2_pkg,
        "launch",
        "navigation_launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Unity /clock for Nav2.",
        ),

        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically activate Nav2 lifecycle nodes.",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launch),
            launch_arguments={
                "use_sim_time": use_sim_time,
                "params_file": nav2_params,
                "autostart": autostart,
            }.items(),
        ),

        Node(
            package="rugged_rover_bringup",
            executable="cmd_vel_to_stamped.py",
            name="cmd_vel_to_diff_drive",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "input_topic": "/cmd_vel",
                    "output_topic": "/diff_drive_controller/cmd_vel",
                    "frame_id": "base_link",
                }
            ],
            output="screen",
        ),
    ])
