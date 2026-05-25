from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = FindPackageShare("rugged_rover_robot_description")

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
            Command(["xacro ", xacro_path]),
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
        Node(
            package="ros_tcp_endpoint",
            executable="default_server_endpoint",
            name="ros_tcp_endpoint",
            parameters=[{
                "ROS_IP": "127.0.0.1",
                "ROS_TCP_PORT": 10000,
            }],
            remappings=[
                # Keep Unity's pose estimate available for comparison, but do not let
                # it own the navigation odom topic or odom -> base_link transform.
                ("/odom", "/ground_truth/odom"),
                ("/tf", "/unity/tf"),
                ("/tf_static", "/unity/tf_static"),

                # Unity is the upstream motor feedback producer. Route it through
                # fault injection before ros2_control consumes it.
                ("/platform/motors/feedback", "/platform/motors/feedback_raw"),
            ],
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
                # diff_drive_controller publishes wheel-derived odometry under the
                # controller namespace. Fault injection consumes /odom_raw and
                # republishes the possibly faulted stream on /odom for Nav2/SLAM.
                ("/diff_drive_controller/odom", "/odom_raw"),
            ],
            output="screen",
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            launch_arguments={
                "use_sim_time": "true",
                "odom_topic": "/odom_raw",
                "imu_topic": "/imu/data",
                "output_odom_topic": "/odom",
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
                        "--activate",
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
                        "--activate",
                    ],
                    output="screen",
                ),
            ],
        ),
    ])
