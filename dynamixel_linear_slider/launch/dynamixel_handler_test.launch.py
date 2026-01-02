#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # dynamixel_handlerノード
        Node(
            package='dynamixel_handler',
            executable='dynamixel_handler',
            name='dynamixel_handler',
            output='screen',
            parameters=['../DynamixelHandler-ros2/dynamixel_handler/config/config_dynamixel_handler.yaml'],
        ),
        # テスト用の制御ノード（速度制御）
        Node(
            package='dynamixel_linear_slider',
            executable='dynamixel_handler_controller',
            name='dynamixel_handler_controller',
            output='screen',
            parameters=[{
                'id_list': [1],
                'amplitude': 50.0,  # deg/s（小さめの値でテスト）
                'frequency': 0.05,  # Hz（ゆっくりとした動作）
                'publish_rate': 20.0, # Hz
                'control_mode': 'velocity',  # 速度制御
            }],
        ),
        # オドメトリノード（状態監視用）
        Node(
            package='dynamixel_linear_slider',
            executable='dynamixel_handler_odometry_node',
            name='dynamixel_handler_odometry_node',
            output='screen',
            parameters=[{
                'motor_id': 1,
                'rack_pitch': 0.005,  # m/rev (5mm/rev)
                'gear_ratio': 100.0,
                'control_frequency': 10.0,  # 低い周波数でテスト
            }]
        ),
    ]) 