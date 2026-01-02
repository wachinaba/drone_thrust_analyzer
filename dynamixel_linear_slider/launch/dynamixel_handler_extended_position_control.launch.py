#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch引数の宣言
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # DynamixelHandler_ros2ノード
    dynamixel_handler_node = Node(
        package='dynamixel_handler',
        executable='dynamixel_handler',
        name='dynamixel_handler',
        output='screen',
        parameters=['../DynamixelHandler-ros2/dynamixel_handler/config/config_dynamixel_handler.yaml'],
    )
    
    # DynamixelHandler位置制御ノード
    dynamixel_handler_position_controller_node = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_handler_position_controller_node',
        name='dynamixel_handler_position_controller_node',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'control_frequency': 100.0,
            'max_position_deg': 180.0,
            'min_position_deg': -180.0,
            'profile_velocity_deg_s': 100.0,
            'profile_acceleration_deg_ss': 500.0,
            'position_tolerance_deg': 1.0,
            'timeout_seconds': 10.0,
            # 位置変換パラメータ
            'rack_pitch': 0.106214,  # m/rev
            'gear_ratio': 1.0,
            'max_position_m': 2.0,  # 最大位置制限 (m)
            'min_position_m': 0.0,  # 最小位置制限 (m)
        }]
    )
    
    # テスト用位置指令パブリッシャー（オプション）
    position_test_publisher_node = Node(
        package='dynamixel_linear_slider',
        executable='position_test_publisher_node',
        name='position_test_publisher_node',
        output='screen',
        parameters=[{
            'publish_frequency': 0.1,  # 10秒ごとに新しい位置を設定
            'position_amplitude': 0.5,  # ±0.5mの範囲でテスト
            'position_offset': 1.0,     # 中心位置（1.0m）
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        dynamixel_handler_node,
        dynamixel_handler_position_controller_node,
        position_test_publisher_node,
    ]) 