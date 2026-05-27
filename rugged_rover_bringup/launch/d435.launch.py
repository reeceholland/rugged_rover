from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
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
                'pointcloud.enable': 'true',
                'publish_tf': 'true',
                'tf_publish_rate': '0.0',
            }.items(),
        ),
    ])
