# Copyright 2026 Reece Holland
#
# Licensed under the Apache License, Version 2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_ekf = LaunchConfiguration("use_ekf")
    use_slam = LaunchConfiguration("use_slam")
    use_rplidar = LaunchConfiguration("use_rplidar")

    bringup_share = FindPackageShare("rugged_rover_bringup")

    bringup_launch = PathJoinSubstitution([
        bringup_share,
        "launch",
        "bringup.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_ekf",
            default_value="true",
            description="Start EKF through bringup.",
        ),
        DeclareLaunchArgument(
            "use_slam",
            default_value="true",
            description="Start SLAM through bringup.",
        ),
        DeclareLaunchArgument(
            "use_rplidar",
            default_value="true",
            description="Start RPLidar through bringup.",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch),
            launch_arguments={
                "use_ekf": use_ekf,
                "use_slam": use_slam,
                "use_rplidar": use_rplidar,
            }.items(),
        ),
    ])