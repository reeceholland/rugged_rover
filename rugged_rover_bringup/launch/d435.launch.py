from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    realsense_launch = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py'
    ])

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'camera_name': 'camera',
                'camera_namespace': 'camera',
                'device_type': 'd435',
                'enable_color': 'true',
                'enable_depth': 'true',
                'enable_sync': 'true',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'false',
                'publish_tf': 'true',
                'tf_publish_rate': '0.0',
            }.items(),
        ),
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            parameters=[{
                'output_frame': 'camera_depth_frame',
                'range_min': 0.15,
                'range_max': 2.0,
                'scan_height': 4,
            }],
            remappings=[
                ('depth', '/camera/camera/depth/image_rect_raw'),
                ('depth_camera_info', '/camera/camera/depth/camera_info'),
                ('scan', '/depth_scan'),
            ],
        ),
    ])
