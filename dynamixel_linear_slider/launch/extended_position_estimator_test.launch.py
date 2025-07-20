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
    
    # Dynamixelシミュレーションノード（テスト用）
    dynamixel_simulator_node = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_simulator_node',
        name='dynamixel_simulator_node',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'control_frequency': 100.0,
            'max_velocity': 65.0,
            'max_acceleration': 130.0,
            'encoder_resolution': 4096,
            'gear_ratio': 1.0,
            'stall_torque': 0.92,
            'stall_current': 0.8,
            'no_load_speed': 65.0,
            'no_load_current': 0.05,
            'resistance_type': 'viscous',
            'viscous_coefficient': 0.1,
            'friction_coefficient': 0.05,
            'time_varying_amplitude': 0.2,
            'time_varying_frequency': 1.0,
            'slider_total_length': 100.0,
            'slider_initial_position': 50.0,
            'encoder_to_slider_ratio': 0.01,
            'slider_position_topic': 'slider_position',
        }]
    )
    
    # テスト用速度指令パブリッシャー
    velocity_sine_publisher_node = Node(
        package='dynamixel_linear_slider',
        executable='velocity_sine_publisher',
        name='velocity_sine_publisher',
        output='screen',
        parameters=[{
            'id_list': [1],
            'amplitude': 160.0,  # deg/s
            'frequency': 0.08,   # Hz
            'publish_rate': 20.0, # Hz
        }]
    )
    
    # ARマーカー推定ノード群（テスト用）
    estimator_node = Node(
        package='aruco_slider_estimator',
        executable='estimator_node',
        name='estimator_node',
        output='screen',
        parameters=[{
            'marker_dictionary': 'DICT_4X4_100',
            'base_board_config': PathJoinSubstitution([
                FindPackageShare('aruco_slider_estimator'),
                'config',
                'base_board_config.json'
            ]),
            'slider_board_config': PathJoinSubstitution([
                FindPackageShare('aruco_slider_estimator'),
                'config',
                'slider_board_config.json'
            ]),
            'detector_params_file': PathJoinSubstitution([
                FindPackageShare('aruco_slider_estimator'),
                'config',
                'detector_params.yaml'
            ]),
            'world_frame_id': 'world',
            'slider_frame_id': 'slider_base',
            'publish_tf': True,
            'show_debug_image': True,
            'camera_id': 0,
        }],
        remappings=[
            ('~/input/image_raw', '/camera/image_raw'),
            ('~/input/camera_info', '/camera/camera_info'),
        ]
    )
    
    # 位置変換ノード
    pose_to_position_converter_node = Node(
        package='dynamixel_linear_slider',
        executable='pose_to_position_converter_node',
        name='pose_to_position_converter_node',
        output='screen',
        parameters=[{
            'position_axis': 'x',  # スライダ軸方向
            'scale_factor': 1.0,   # スケール係数
        }]
    )
    
    # 新しいExtendedPositionEstimatorNode
    extended_position_estimator_node = Node(
        package='dynamixel_linear_slider',
        executable='extended_position_estimator_node',
        name='extended_position_estimator_node',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'rack_pitch': 0.106214,  # m/rev
            'gear_ratio': 1.0,
            'control_frequency': 100.0,
            'velocity_zero_threshold': 0.01,  # deg/s
            'ar_marker_average_duration': 1.0,  # 秒
            'ar_marker_std_threshold': 0.01,  # m
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        dynamixel_handler_node,
        dynamixel_simulator_node,
        velocity_sine_publisher_node,
        estimator_node,
        pose_to_position_converter_node,
        extended_position_estimator_node,
    ]) 