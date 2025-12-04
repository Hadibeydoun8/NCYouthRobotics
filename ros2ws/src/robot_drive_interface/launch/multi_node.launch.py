from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_drive_interface',
            executable='drive_node',
            name='drive_node'
        ),
        Node(
            package='robot_drive_interface',
            executable='video_publisher',
            name='video_publisher'
        )
    ])
