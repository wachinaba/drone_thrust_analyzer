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
    
    # 収束ブレーキパラメータ
    convergence_threshold_arg = DeclareLaunchArgument(
        'convergence_threshold',
        default_value='0.002',
        description='Position error threshold for convergence (m)'
    )
    
    convergence_duration_arg = DeclareLaunchArgument(
        'convergence_duration',
        default_value='1.0',
        description='Duration required for convergence (s)'
    )
    
    velocity_zero_threshold_arg = DeclareLaunchArgument(
        'velocity_zero_threshold',
        default_value='0.01',
        description='Velocity command threshold for zero detection (m/s)'
    )
    
    enable_convergence_brake_arg = DeclareLaunchArgument(
        'enable_convergence_brake',
        default_value='true',
        description='Enable convergence brake functionality'
    )
    
    # 移動状態判定パラメータ
    movement_velocity_threshold_arg = DeclareLaunchArgument(
        'movement_velocity_threshold',
        default_value='0.005',
        description='Velocity threshold for movement detection (m/s)'
    )
    
    movement_position_threshold_arg = DeclareLaunchArgument(
        'movement_position_threshold',
        default_value='0.001',
        description='Position error threshold for movement detection (m)'
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
    
    # 位置テストパブリッシャーノード
    position_test_publisher_node = Node(
        package='dynamixel_linear_slider',
        executable='position_test_publisher_node',
        name='position_test_publisher_node',
        output='screen',
        parameters=[{
            'publish_frequency': 0.1,  # Hz（10秒ごと）
            'position_amplitude': 0.02,  # 位置振幅 (m)
            'position_offset': 0.05,  # 位置オフセット (m)
            'test_pattern': 'step',  # step, sine, random, increment
            'enable_test': True,  # テストの有効/無効
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
            'convergence_threshold': LaunchConfiguration('convergence_threshold'),
            'convergence_duration': LaunchConfiguration('convergence_duration'),
            'velocity_zero_threshold': LaunchConfiguration('velocity_zero_threshold'),
            'enable_convergence_brake': LaunchConfiguration('enable_convergence_brake'),
            # 移動状態判定パラメータ
            'movement_velocity_threshold': LaunchConfiguration('movement_velocity_threshold'),
            'movement_position_threshold': LaunchConfiguration('movement_position_threshold'),
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        convergence_threshold_arg,
        convergence_duration_arg,
        velocity_zero_threshold_arg,
        enable_convergence_brake_arg,
        movement_velocity_threshold_arg,
        movement_position_threshold_arg,
        dynamixel_handler_node,
        dynamixel_handler_odometry_node,
        dynamixel_simulator_node,
        position_test_publisher_node,
        sensor_fusion_node,
        trajectory_generator_node,
        position_controller_node,
        velocity_command_converter_node,
        dynamixel_handler_position_controller_node,
    ]) 