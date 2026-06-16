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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
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
            description="Velocity command topic produced by teleop_twist_joy.",
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

        ExecuteProcess(
            cmd=[
                "ros2",
                "topic",
                "echo",
                "/joy",
            ],
            output="screen",
        ),

        ExecuteProcess(
            cmd=[
                "ros2",
                "topic",
                "echo",
                cmd_vel_topic,
            ],
            output="screen",
        ),
    ])
