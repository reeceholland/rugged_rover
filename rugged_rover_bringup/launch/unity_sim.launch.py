from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = FindPackageShare("rugged_rover_robot_description")
    wheel_odom_topic = LaunchConfiguration("wheel_odom_topic")
    ekf_output_odom_topic = LaunchConfiguration("ekf_output_odom_topic")
    ros_tcp_ip = LaunchConfiguration("ros_tcp_ip")
    ros_tcp_port = LaunchConfiguration("ros_tcp_port")

    xacro_path = PathJoinSubstitution([
        description_pkg,
        "urdf",
        "rugged_rover.urdf.xacro",
    ])

    controller_params = PathJoinSubstitution([
        description_pkg,
        "config",
        "unity_controller_params.yaml",
    ])

    ekf_launch = PathJoinSubstitution([
        FindPackageShare("rugged_rover_bringup"),
        "launch",
        "ekf.launch.py",
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path, " command_qos:=reliable"]),
            value_type=str,
        )
    }

    controller_manager_params = {
        "update_rate": 100,
        "joint_state_broadcaster.type": "joint_state_broadcaster/JointStateBroadcaster",
        "joint_state_broadcaster.params_file": ParameterValue(
            controller_params,
            value_type=str,
        ),
        "diff_drive_controller.type": "diff_drive_controller/DiffDriveController",
        "diff_drive_controller.params_file": ParameterValue(
            controller_params,
            value_type=str,
        ),
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            "wheel_odom_topic",
            default_value="/odom_raw",
            description=(
                "Wheel odometry topic produced by diff_drive_controller and "
                "consumed by the EKF. Use /wheel/odom_raw when the EKF output "
                "is remapped to /odom_raw for fault injection."
            ),
        ),
        DeclareLaunchArgument(
            "ekf_output_odom_topic",
            default_value="/odom",
            description=(
                "Filtered EKF odometry topic. Set to /odom_raw when "
                "ros2_fault_injection should publish the final /odom topic."
            ),
        ),
        DeclareLaunchArgument(
            "ros_tcp_ip",
            default_value="0.0.0.0",
            description="Address for ros_tcp_endpoint to bind to. Use 0.0.0.0 for remote Unity clients.",
        ),
        DeclareLaunchArgument(
            "ros_tcp_port",
            default_value="10000",
            description="Port for ros_tcp_endpoint.",
        ),

        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            name="ros_tcp_endpoint",
            parameters=[{
                "ROS_IP": ros_tcp_ip,
                "ROS_TCP_PORT": ParameterValue(ros_tcp_port, value_type=int),
            }],
            remappings=[
                # Keep Unity's pose estimate available for comparison, but do not let
                # it own the navigation odom topic or odom -> base_link transform.
                ("/odom", "/ground_truth/odom"),
                ("/tf", "/unity/tf"),
                ("/tf_static", "/unity/tf_static"),

                # Unity is the upstream motor feedback producer. Route it through
                # fault injection before ros2_control consumes it.
                # ("/platform/motors/feedback", "/platform/motors/feedback_raw"),
            ],
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
            name="robot_state_publisher",
            parameters=[robot_description, {"use_sim_time": True}],
            output="screen",
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            name="controller_manager",
            parameters=[
                robot_description,
                controller_manager_params,
                {"use_sim_time": True},
            ],
            remappings=[
                # Feed wheel-derived odometry into EKF. EKF republishes the
                # fused odometry on /odom and owns odom -> base_link.
                ("/diff_drive_controller/odom", wheel_odom_topic),
            ],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            launch_arguments={
                "use_sim_time": "true",
                "odom_topic": wheel_odom_topic,
                "imu_topic": "/imu/data",
                "output_odom_topic": ekf_output_odom_topic,
            }.items(),
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
                        "--controller-manager-timeout",
                        "20",
                        "--switch-timeout",
                        "20",
                    ],
                    output="screen",
                ),
            ],
        ),

        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package="controller_manager",
                    executable="spawner",
                    arguments=[
                        "diff_drive_controller",
                        "--controller-manager",
                        "/controller_manager",
                        "--controller-manager-timeout",
                        "20",
                        "--switch-timeout",
                        "20",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
