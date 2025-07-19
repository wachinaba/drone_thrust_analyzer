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
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # 位置コマンドパラメータ
    target_position_arg = DeclareLaunchArgument(
        'target_position',
        default_value='0.05',
        description='Target position to move to (m)'
    )
    
    wait_timeout_arg = DeclareLaunchArgument(
        'wait_timeout',
        default_value='30.0',
        description='Timeout for waiting movement completion (s)'
    )
    
    # DynamixelHandler_ros2ノード
    dynamixel_handler_node = Node(
        package='dynamixel_handler',
        executable='dynamixel_handler',
        name='dynamixel_handler',
        output='screen',
        parameters=['../DynamixelHandler-ros2/dynamixel_handler/config/config_dynamixel_handler.yaml'],
    )
    
    # DynamixelHandlerオドメトリノード
    dynamixel_handler_odometry_node = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_handler_odometry_node',
        name='dynamixel_handler_odometry_node',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'rack_pitch': 0.005,  # m/rev (5mm/rev)
            'gear_ratio': 100.0,
            'control_frequency': 100.0,
        }]
    )
    
    # Dynamixelシミュレーターノード
    dynamixel_simulator_node = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_simulator_node',
        name='dynamixel_simulator_node',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'max_velocity': 100.0,  # rpm
            'control_frequency': 100.0,
            'resistance_type': 'combined',
            'friction_coefficient': 0.1,
            'viscous_coefficient': 0.01,
            'time_varying_amplitude': 0.05,
            'time_varying_frequency': 0.5,
            'slider_total_length': 100.0,  # mm
            'slider_initial_position': 50.0,  # mm
            'encoder_to_slider_ratio': 0.01,  # mm/encoder_unit
            'slider_position_topic': 'slider_position',
        }]
    )
    
    # センサーフュージョンノード（シミュレーター用）
    sensor_fusion_node = Node(
        package='dynamixel_linear_slider',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
        output='screen',
        parameters=[{
            'alpha': 0.95,
            'drift_bias': 0.001,  # m/s
            'control_frequency': 100.0,
        }],
        remappings=[
            ('ar_marker_position', 'slider_position'),  # シミュレーターの位置を使用
        ]
    )
    
    # 軌道生成ノード
    trajectory_generator_node = Node(
        package='dynamixel_linear_slider',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen',
        parameters=[{
            'max_velocity': 0.05,  # m/s（低速でテスト）
            'acceleration': 0.02,  # m/s²
            'control_frequency': 100.0,
        }]
    )
    
    # 位置制御ノード（PID制御機能統合済み）
    position_controller_node = Node(
        package='dynamixel_linear_slider',
        executable='position_controller_node',
        name='position_controller_node',
        output='screen',
        parameters=[{
            'control_frequency': 100.0,
            'max_velocity': 0.1,  # m/s（低速でテスト）
            'min_velocity': -0.1,  # m/s
            # PID制御パラメータ
            'Kp': 2.0,
            'Ki': 0.1,
            'Kd': 0.05,
            'integral_limit': 5.0,
            'max_pid_output': 0.1,  # m/s
            'min_pid_output': -0.1,  # m/s
        }]
    )
    
    # 速度指令変換ノード
    velocity_command_converter_node = Node(
        package='dynamixel_linear_slider',
        executable='velocity_command_converter_node',
        name='velocity_command_converter_node',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'rack_pitch': 0.005,  # m/rev
            'gear_ratio': 100.0,
            'max_velocity_deg_s': 50.0,  # deg/s（低速でテスト）
            'min_velocity_deg_s': -50.0,  # deg/s
        }]
    )
    
    # DynamixelHandler位置制御ノード（収束ブレーキ機能付き）
    dynamixel_handler_position_controller_node = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_handler_position_controller_node',
        name='dynamixel_handler_position_controller_node',
        output='screen',
        parameters=[{
            'control_frequency': 100.0,
            'rack_pitch': 0.005,  # m/rev
            'gear_ratio': 100.0,
            # 収束ブレーキパラメータ
            'convergence_threshold': 0.002,
            'convergence_duration': 1.0,
            'velocity_zero_threshold': 0.01,
            'enable_convergence_brake': True,
            # 移動状態判定パラメータ
            'movement_velocity_threshold': 0.005,
            'movement_position_threshold': 0.001,
        }]
    )
    
    # 位置コマンド送信と待機ノード
    position_command_and_wait_node = Node(
        package='dynamixel_linear_slider',
        executable='position_command_and_wait_node',
        name='position_command_and_wait_node',
        output='screen',
        parameters=[{
            'target_position': LaunchConfiguration('target_position'),
            'wait_timeout': LaunchConfiguration('wait_timeout'),
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        target_position_arg,
        wait_timeout_arg,
        dynamixel_handler_node,
        dynamixel_handler_odometry_node,
        dynamixel_simulator_node,
        sensor_fusion_node,
        trajectory_generator_node,
        position_controller_node,
        velocity_command_converter_node,
        dynamixel_handler_position_controller_node,
        position_command_and_wait_node,
    ]) 