from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_drive_interface',
            executable='drive_node',
            name='drive_node'
        ),
        # Node(
        #     package='robot_drive_interface',
        #     executable='video_publisher',
        #     name='video_publisher'
        # ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='D415',
            namespace='robot',
            parameters=[
                {'color_profile': '640x480x15'},
                {'depth_profile': '640x480x15'},

                {'enable_infra1': False},
                {'enable_infra2': False},

                {'enable_color': True},
                {'enable_depth': True},

                {'enable_pointcloud': False},
                {'enable_sync': False},
                {'align_depth.enable': False},

                {'color0.topic_publisher.queue_size': 2},
                {'depth0.topic_publisher.queue_size': 2},
            ]
        )
    ])
