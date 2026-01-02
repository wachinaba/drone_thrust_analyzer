#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パッケージのパスを取得
    pkg_share = FindPackageShare('dynamixel_linear_slider')
    
    # Launch引数の宣言
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # ハードウェア制御ノード群
    dynamixel_driver_node = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_driver_node',
        name='dynamixel_driver_node',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'baud_rate': 57600,
            'port_name': '/dev/ttyUSB0',
            'control_frequency': 100.0,
        }]
    )
    
    dynamixel_odometry_node = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_odometry_node',
        name='dynamixel_odometry_node',
        output='screen',
        parameters=[{
            'motor_id': 1,
            'baud_rate': 57600,
            'port_name': '/dev/ttyUSB0',
            'rack_pitch': 0.005,  # m/rev (5mm/rev)
            'gear_ratio': 100.0,
            'encoder_resolution': 4096,
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
    
    # 制御ノード群
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
    
    pid_controller_node = Node(
        package='dynamixel_linear_slider',
        executable='pid_controller_node',
        name='pid_controller_node',
        output='screen',
        parameters=[{
            'Kp': 1.0,
            'Ki': 0.1,
            'Kd': 0.05,
            'control_frequency': 100.0,
            'max_output': 100.0,
            'min_output': -100.0,
            'integral_limit': 10.0,
        }]
    )
    
    trajectory_generator_node = Node(
        package='dynamixel_linear_slider',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen',
        parameters=[{
            'target_position': 0.1,  # m
            'max_velocity': 0.1,  # m/s
            'acceleration': 0.05,  # m/s²
            'control_frequency': 100.0,
            'trajectory_type': 'linear',  # linear, sine, step
        }]
    )
    
    position_controller_node = Node(
        package='dynamixel_linear_slider',
        executable='position_controller_node',
        name='position_controller_node',
        output='screen',
        parameters=[{
            'control_frequency': 100.0,
            'max_velocity': 100.0,
            'min_velocity': -100.0,
        }]
    )
    
    # 可視化ノード（オプション）
    rqt_plot_node = Node(
        package='rqt_plot',
        executable='rqt_plot',
        name='rqt_plot',
        arguments=['/estimated_position/data', '/target_position/data', '/velocity_command/data'],
        output='screen'
    )
    
    # Launch Descriptionの作成
    return LaunchDescription([
        use_sim_time_arg,
        
        # ハードウェア制御ノード群
        dynamixel_driver_node,
        dynamixel_odometry_node,
        
        # ARマーカー推定ノード群
        estimator_node,
        pose_to_position_converter_node,
        
        # 制御ノード群
        sensor_fusion_node,
        pid_controller_node,
        trajectory_generator_node,
        position_controller_node,
        
        # 可視化ノード
        rqt_plot_node,
    ]) 