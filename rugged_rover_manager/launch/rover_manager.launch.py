# Copyright 2026 Reece Holland
#
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    manager_config = LaunchConfiguration("manager_config")
    use_respawn = LaunchConfiguration("use_respawn")

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("rugged_rover_manager"),
            "config",
            "rover_manager.yaml",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "manager_config",
                default_value=default_config,
                description="Path to the rover manager parameter file.",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="true",
                description="Respawn the rover manager if it exits unexpectedly.",
            ),
            Node(
                package="rugged_rover_manager",
                executable="rover_manager_node",
                name="rover_manager",
                output="screen",
                parameters=[manager_config],
                respawn=use_respawn,
                respawn_delay=2.0,
            ),
        ]
    )