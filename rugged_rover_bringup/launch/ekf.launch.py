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
