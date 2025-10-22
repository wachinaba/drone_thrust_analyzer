#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # パッケージのパスを取得
    pkg_share = get_package_share_directory('seven_segment_reader')
    
    # 設定ファイルのパス
    config_file = os.path.join(pkg_share, 'config', 'detector_params_server.yaml')
    
    # パラメータファイルの存在確認
    if not os.path.exists(config_file):
        raise FileNotFoundError(f"設定ファイルが見つかりません: {config_file}")
    
    return LaunchDescription([
        # 7セグメントディスプレイ読み取りノード（HTTP Client版）
        Node(
            package='seven_segment_reader',
            executable='seven_segment_reader_http_client',
            name='seven_segment_reader_http_client',
            output='screen',
            parameters=[config_file],
            remappings=[
                ('seven_segment_detection', '/seven_segment/detection'),
                ('seven_segment_values', '/seven_segment/values'),
                ('detection_timestamp', '/seven_segment/timestamp')
            ]
        )
    ])
