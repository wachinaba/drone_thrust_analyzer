#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """USBカメラノードを起動し、camera_info_urlでYAMLファイルを読み込むlaunchファイル"""
    
    # Launch引数の定義
    camera_info_file_arg = DeclareLaunchArgument(
        'camera_info_file',
        default_value='',
        description='camera_infoのYAMLファイルへのパス（絶対パスで指定）'
    )
    
    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='カメラデバイスID'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='カメラフレームID'
    )
    
    # USBカメラノード
    usb_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_camera',
        output='screen',
        parameters=[{
            'video_device': ['/dev/video', LaunchConfiguration('camera_id')],
            'image_width': 1024,
            'image_height': 576,
            'pixel_format': 'mjpeg2rgb',
            'camera_frame_id': LaunchConfiguration('frame_id'),
            'camera_info_url': ['file://', LaunchConfiguration('camera_info_file')],
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ]
    )
    
    return LaunchDescription([
        camera_info_file_arg,
        camera_id_arg,
        frame_id_arg,
        usb_camera_node,
    ]) 