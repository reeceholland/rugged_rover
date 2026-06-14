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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    serial_baudrate = LaunchConfiguration("serial_baudrate")
    frame_id = LaunchConfiguration("frame_id")
    scan_topic = LaunchConfiguration("scan_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/rplidar",
            description="Serial device for the RPLIDAR S2.",
        ),
        DeclareLaunchArgument(
            "serial_baudrate",
            default_value="1000000",
            description="Serial baudrate for the RPLIDAR S2.",
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="laser",
            description="Frame id used in LaserScan messages.",
        ),
        DeclareLaunchArgument(
            "scan_topic",
            default_value="/scan",
            description="LaserScan topic name.",
        ),

        Node(
            package="sllidar_ros2",
            executable="sllidar_node",
            name="rplidar_s2",
            output="screen",
            parameters=[{
                "channel_type": "serial",
                "serial_port": serial_port,
                "serial_baudrate": ParameterValue(serial_baudrate, value_type=int),
                "frame_id": frame_id,
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "",
            }],
            remappings=[
                ("scan", scan_topic),
            ],
        ),
    ])