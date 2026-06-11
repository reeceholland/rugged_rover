from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = FindPackageShare("rugged_rover_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")

    nav2_params = PathJoinSubstitution([
        bringup_pkg,
        "config",
        "nav2_params.yaml",
    ])

    remappings = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    lifecycle_nodes = [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "velocity_smoother",
        "collision_monitor",
        "bt_navigator",
        "waypoint_follower",
    ]

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

        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Nav2 log level.",
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),

        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),

        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),

        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings + [("cmd_vel", "cmd_vel_nav")],
        ),

        Node(
            package="nav2_collision_monitor",
            executable="collision_monitor",
            name="collision_monitor",
            output="screen",
            parameters=[nav2_params, {"use_sim_time": use_sim_time}],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=remappings,
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"autostart": autostart},
                {"node_names": lifecycle_nodes},
            ],
            arguments=["--ros-args", "--log-level", log_level],
        ),

        Node(
            package="topic_tools",
            executable="transform",
            name="cmd_vel_to_diff_drive",
            arguments=[
                "/cmd_vel",
                "/diff_drive_controller/cmd_vel",
                "geometry_msgs/msg/TwistStamped",
                "geometry_msgs.msg.TwistStamped(header=std_msgs.msg.Header(frame_id='base_link'), twist=m)",
                "--import",
                "geometry_msgs",
                "std_msgs",
                "--wait-for-start",
                "--qos-reliability",
                "reliable",
            ],
            output="screen",
        ),
    ])
