from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depth_to_scan',
            remappings=[
                ('image', '/kinect/depth/image_raw'),
                ('camera_info', '/kinect/depth/camera_info'),
                ('scan', '/scan'),
            ],
            parameters=[{
                'range_min': 0.3,
                'range_max': 3.5,
                'scan_time': 0.033,
                'output_frame_id': 'camera_link'
            }]
        )
    ])
