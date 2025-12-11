from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # Path to Interbotix xsarm_control launch file
    xsarm_launch = os.path.join(
        get_package_share_directory('interbotix_xsarm_control'),
        'launch',
        'xsarm_control.launch.py'
    )

    interbotix = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(xsarm_launch),
        launch_arguments={'robot_model': 'rx200'}.items()
    )

    realsense = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='D415',
        namespace='robot',
        parameters=[
            {'enable_color': True},
            {'enable_depth': False},
            {'enable_infra1': False},
            {'enable_infra2': False},
            {'enable_pointcloud': False},
            {'enable_sync': False},
            {'align_depth.enable': False},

            {'rgb_camera.profile': '640x480x6'},
            {'color0.topic_publisher.queue_size': 2},
        ]
    )

    return LaunchDescription([interbotix, realsense])
