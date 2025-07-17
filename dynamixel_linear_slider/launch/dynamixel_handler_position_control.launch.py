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
    
    # ARマーカー推定ノード群
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
    
    # センサーフュージョンノード
    sensor_fusion_node = Node(
        package='dynamixel_linear_slider',
        executable='sensor_fusion_node',
        name='sensor_fusion_node',
        output='screen',
        parameters=[{
            'alpha': 0.9,
            'drift_bias': 0.001,  # m/s
            'control_frequency': 100.0,
        }]
    )
    
    # 軌道生成ノード
    trajectory_generator_node = Node(
        package='dynamixel_linear_slider',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen',
        parameters=[{
            'target_position': 0.1,  # m（初期値）
            'max_velocity': 0.1,  # m/s
            'acceleration': 0.05,  # m/s²
            'control_frequency': 100.0,
            'trajectory_type': 'linear',
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
            'max_velocity': 100.0,
            'min_velocity': -100.0,
            # PID制御パラメータ
            'Kp': 1.0,
            'Ki': 0.1,
            'Kd': 0.05,
            'integral_limit': 10.0,
            'max_pid_output': 100.0,
            'min_pid_output': -100.0,
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
            'max_velocity_deg_s': 100.0,
            'min_velocity_deg_s': -100.0,
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        dynamixel_handler_node,
        dynamixel_handler_odometry_node,
        estimator_node,
        pose_to_position_converter_node,
        sensor_fusion_node,
        trajectory_generator_node,
        position_controller_node,
        velocity_command_converter_node,
    ]) 