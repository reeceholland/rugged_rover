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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_pkg = FindPackageShare("rugged_rover_bringup")

    ekf_params = PathJoinSubstitution([
        bringup_pkg,
        "config",
        "ekf.yaml",
    ])

    use_sim_time = LaunchConfiguration("use_sim_time")
    odom_topic = LaunchConfiguration("odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    output_odom_topic = LaunchConfiguration("output_odom_topic")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock for the EKF.",
        ),

        DeclareLaunchArgument(
            "odom_topic",
            default_value="/diff_drive_controller/odom",
            description="Wheel odometry topic from diff_drive_controller.",
        ),

        DeclareLaunchArgument(
            "imu_topic",
            default_value="/imu/data",
            description="IMU topic to fuse with wheel odometry.",
        ),

        DeclareLaunchArgument(
            "output_odom_topic",
            default_value="/odom",
            description="Filtered odometry topic for Nav2 and velocity smoothing.",
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                ekf_params,
                {
                    "use_sim_time": use_sim_time,
                    "odom0": odom_topic,
                    "imu0": imu_topic,
                },
            ],
            remappings=[
                ("odometry/filtered", output_odom_topic),
            ],
        ),
    ])
