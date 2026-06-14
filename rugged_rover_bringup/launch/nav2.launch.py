# Copyright 2026 Reece Holland
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
