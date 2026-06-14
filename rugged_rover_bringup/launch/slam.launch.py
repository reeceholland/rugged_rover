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
            default_value="false",
            description="Use simulated /clock for slam_toolbox.",
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
                        "ros2 service call /slam_toolbox/change_state "
                        "lifecycle_msgs/srv/ChangeState "
                        "'{transition: {id: 1}}'",
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
                        "until ros2 service call /slam_toolbox/get_state "
                        "lifecycle_msgs/srv/GetState "
                        "'{}' 2>/dev/null | grep -Eq 'id=2|id: 2'; "
                        "do sleep 0.2; done; "
                        "ros2 service call /slam_toolbox/change_state "
                        "lifecycle_msgs/srv/ChangeState "
                        "'{transition: {id: 3}}'",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
