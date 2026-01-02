#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # 設定ファイルのパス
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('seven_segment_reader'),
            'config',
            'detector_params.yaml'
        ]),
        description='設定ファイルのパス'
    )
    
    # secrets設定ファイルのパス
    secrets_file_arg = DeclareLaunchArgument(
        'secrets_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('seven_segment_reader'),
            'config',
            'secrets.yaml'
        ]),
        description='secrets設定ファイルのパス'
    )
    
    # 7セグメント読み取りノード
    seven_segment_reader_node = Node(
        package='seven_segment_reader',
        executable='seven_segment_reader_node',
        name='seven_segment_reader_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            LaunchConfiguration('secrets_file')
        ],
        remappings=[
            ('seven_segment_detection', '/seven_segment/detection'),
            ('seven_segment_values', '/seven_segment/values'),
            ('detection_timestamp', '/seven_segment/timestamp'),
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        secrets_file_arg,
        seven_segment_reader_node,
    ])
