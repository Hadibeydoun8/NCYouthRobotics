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
                {'enable_color': True},
                {'enable_depth': False},
                {'enable_infra1': False},
                {'enable_infra2': False},
                {'enable_pointcloud': False},
                {'enable_sync': False},
                {'align_depth.enable': False},

                # Correct RealSense ROS2 parameter names
                {'rgb_camera.profile': '640x480x6'},

                # Optional queue tuning
                {'color0.topic_publisher.queue_size': 2},
            ]
        )
    ])
