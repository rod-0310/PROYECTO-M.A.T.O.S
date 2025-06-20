import os

import launch_ros
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package="kinect_ros2").find("kinect_ros2")
    default_rviz_config_path = os.path.join(pkg_share, "rviz/pointcloud.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            name="rvizconfig",
            default_value=default_rviz_config_path,
            description="Absolute path to rviz config file",
        ),
        Node(
            package="kinect_ros2",
            executable="kinect_ros2_node",
            name="kinect_ros2",
            namespace="kinect",
        ),
        Node(
            package="depthimage_to_laserscan",
            executable="depthimage_to_laserscan_node",
            name="depthimage_to_laserscan",
            output="screen",
            parameters=[{
                "output_frame_id": "kinect_depth_optical_frame",
                "range_min": 0.3,
                "range_max": 5.0,
                "scan_time": 0.033,
                "scan_height": 20
            }],
            remappings=[
                ("/depth", "/kinect/depth/image_raw"),
                ("/depth_camera_info", "/kinect/depth/camera_info"),
                ("/scan", "/scan")
            ]
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rvizconfig")],
        ),
    ])
