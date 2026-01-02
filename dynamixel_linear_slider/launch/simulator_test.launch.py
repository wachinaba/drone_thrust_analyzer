#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Dynamixelシミュレーションノードのテスト用launchファイル"""
    
    # Launch引数の定義
    test_pattern_arg = DeclareLaunchArgument(
        'test_pattern',
        default_value='sine',
        description='テストパターン (sine, step, ramp)'
    )
    
    amplitude_arg = DeclareLaunchArgument(
        'amplitude',
        default_value='180.0',
        description='速度振幅 (deg/s) - XC330-T288-T仕様に基づく安全値'
    )
    
    frequency_arg = DeclareLaunchArgument(
        'frequency',
        default_value='0.5',
        description='テストパターンの周波数 (Hz)'
    )
    
    # Dynamixelシミュレーションノード
    dynamixel_simulator_node = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_simulator_node',
        name='dynamixel_simulator_node',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'control_frequency': 100.0,
            'max_velocity': 65.0,  # XC330-T288-T仕様
            'max_acceleration': 130.0,  # XC330-T288-T仕様
            'encoder_resolution': 4096,
            'gear_ratio': 1.0,
            'stall_torque': 0.92,  # XC330-T288-T仕様
            'stall_current': 0.8,   # XC330-T288-T仕様
            'no_load_speed': 65.0,  # XC330-T288-T仕様
            'no_load_current': 0.05, # XC330-T288-T仕様
            'resistance_type': 'time_varying',  # テスト用に時間変化抵抗を設定
            'viscous_coefficient': 0.1,
            'friction_coefficient': 0.05,
            'time_varying_amplitude': 0.2,
            'time_varying_frequency': 1.0,
            'random_resistance_std': 0.1,
            'slider_total_length': 100.0,  # テスト用スライダ全長
            'slider_initial_position': 50.0,  # テスト用初期位置
            'encoder_to_slider_ratio': 0.01,  # テスト用変換比
            'slider_position_topic': 'slider_position',
        }],
        remappings=[
            ('velocity_command', 'velocity_command'),
            ('present_position', 'present_position'),
            ('present_velocity', 'present_velocity'),
            ('joint_states', 'joint_states'),
            ('motor_status', 'motor_status'),
        ]
    )
    
    # テストパブリッシャー
    simulator_test_publisher = Node(
        package='dynamixel_linear_slider',
        executable='simulator_test_publisher',
        name='simulator_test_publisher',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'test_pattern': LaunchConfiguration('test_pattern'),
            'amplitude': LaunchConfiguration('amplitude'),
            'frequency': LaunchConfiguration('frequency'),
            'publish_frequency': 10.0,
        }],
        remappings=[
            ('/dynamixel/commands/x', '/dynamixel/commands/x'),
            ('/dynamixel/states', '/dynamixel/states'),
        ]
    )
    
    # スライダ位置パブリッシャー（テスト用）
    slider_position_publisher = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_simulator_node',
        name='slider_position_publisher',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'control_frequency': 100.0,
            'max_velocity': 65.0,
            'max_acceleration': 130.0,
            'encoder_resolution': 4096,
            'gear_ratio': 1.0,
            'slider_total_length': 100.0,
            'slider_initial_position': 50.0,
            'encoder_to_slider_ratio': 0.01,
            'slider_position_topic': 'slider_position',
        }],
        remappings=[
            ('slider_position', 'slider_position'),
        ]
    )
    
    return LaunchDescription([
        test_pattern_arg,
        amplitude_arg,
        frequency_arg,
        dynamixel_simulator_node,
        simulator_test_publisher,
    ]) 