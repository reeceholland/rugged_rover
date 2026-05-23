from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = FindPackageShare("rugged_rover_bringup")
    use_sim_time = LaunchConfiguration("use_sim_time")

    slam_params = PathJoinSubstitution([
        bringup_pkg,
        "config",
        "slam_toolbox.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description="Use Unity /clock for slam_toolbox.",
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[
                slam_params,
                {"use_sim_time": use_sim_time},
            ],
        ),

        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-lc",
                        "until ros2 service list | grep -q '^/slam_toolbox/change_state$'; "
                        "do sleep 0.2; done; "
                        "ros2 lifecycle set /slam_toolbox configure",
                    ],
                    output="screen",
                ),
            ],
        ),

        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        "bash",
                        "-lc",
                        "until ros2 lifecycle get /slam_toolbox 2>/dev/null | grep -q 'inactive'; "
                        "do sleep 0.2; done; "
                        "ros2 lifecycle set /slam_toolbox activate",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
