from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    micro_ros_device = LaunchConfiguration("micro_ros_device")
    use_rplidar = LaunchConfiguration("use_rplidar")
    rplidar_serial_port = LaunchConfiguration("rplidar_serial_port")
    rplidar_serial_baudrate = LaunchConfiguration("rplidar_serial_baudrate")

    xacro_path = PathJoinSubstitution([
        FindPackageShare("rugged_rover_robot_description"),
        "urdf",
        "rugged_rover.urdf.xacro",
    ])

    controller_config = PathJoinSubstitution([
        FindPackageShare("rugged_rover_control"),
        "config",
        "controllers.yaml",
    ])

    imu_launch = PathJoinSubstitution([
        FindPackageShare("razor_imu"),
        "launch",
        "razor.launch.py",
    ])

    ekf_launch = PathJoinSubstitution([
        FindPackageShare("rugged_rover_bringup"),
        "launch",
        "ekf.launch.py",
    ])

    d435_launch = PathJoinSubstitution([
        FindPackageShare("rugged_rover_bringup"),
        "launch",
        "d435.launch.py",
    ])

    rplidar_launch = PathJoinSubstitution([
        FindPackageShare("rugged_rover_bringup"),
        "launch",
        "rplidar_s2.launch.py",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path]),
            value_type=str,
        )
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "micro_ros_device",
            default_value="/dev/ttyAMA0",
            description="Serial device used by the micro-ROS Agent.",
        ),
        DeclareLaunchArgument(
            "use_rplidar",
            default_value="true",
            description="Launch the RPLIDAR S2 driver.",
        ),
        DeclareLaunchArgument(
            "rplidar_serial_port",
            default_value="/dev/rplidar",
            description="Serial device used by the RPLIDAR S2.",
        ),
        DeclareLaunchArgument(
            "rplidar_serial_baudrate",
            default_value="1000000",
            description="Serial baudrate used by the RPLIDAR S2.",
        ),

        Node(
            package="micro_ros_agent",
            executable="micro_ros_agent",
            name="micro_ros_agent",
            arguments=["serial", "--dev", micro_ros_device],
            output="screen",
        ),

        Node(
            package="rugged_rover_battery",
            executable="battery_voltage_monitor",
            name="battery_voltage_monitor",
            output="screen",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            launch_arguments={
                "use_sim_time": "false",
                "odom_topic": "/diff_drive_controller/odom",
                "imu_topic": "/imu/data",
                "output_odom_topic": "/odom",
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(d435_launch),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rplidar_launch),
            condition=IfCondition(use_rplidar),
            launch_arguments={
                "serial_port": rplidar_serial_port,
                "serial_baudrate": rplidar_serial_baudrate,
                "frame_id": "laser",
                "scan_topic": "/scan",
            }.items(),
        ),

        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                robot_description,
                controller_config,
            ],
            output="screen",
        ),

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "joint_state_broadcaster",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                    output="screen",
                ),
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "diff_drive_controller",
                        "--controller-manager",
                        "/controller_manager",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
