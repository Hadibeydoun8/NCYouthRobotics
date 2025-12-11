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
            namespace='robot1',
            parameters=[
                # Lower RGB resolution + FPS for Pi bandwidth limits
                {'rgb_camera.profile': '640x480x15'},
                {'enable_color': True},

                # Lower depth workload
                {'depth_module.profile': '640x480x15'},
                {'enable_depth': True},

                # Disable heavy filters
                {'enable_pointcloud': False},
                {'enable_sync': False},
                {'enable_accel': False},
                {'enable_gyro': False},

                # Reduce CPU load
                {'align_depth.enable': False},

                # Force image compression for network transport
                {'enable_infra1': False},
                {'enable_infra2': False},
            ],
            remappings=[
                ('/color/image_raw', '/robot1/color/image_raw/compressed'),
                ('/depth/image_rect_raw', '/robot1/depth/image_rect_raw/compressed'),
            ]
        )
    ])
