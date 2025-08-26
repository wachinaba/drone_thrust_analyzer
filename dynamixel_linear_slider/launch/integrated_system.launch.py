#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """統合システムのlaunchファイル"""
    
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
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'dynamixel_handler:=WARN'],
        parameters=[{
            'device_name': '/dev/ttyACM0',
            'baudrate': 57600,
            'latency_timer': 16,
            'init/dummy_servo_list': [-1],
            'init/baudrate_auto_set': False,
            'init/uesd_servo_series': {'X': True, 'P': False, 'Pro': False},
            'init/expected_servo_num': 0,
            'init/servo_auto_search': {'min_id': 0, 'max_id': 10, 'retry_times': 0},
            'init/hardware_error_auto_clean': True,
            'init/torque_auto_enable': True,
            'term/torque_auto_disable': True,
            'term/servo_auto_stop': True,
            'default/profile_acc': 600.0,
            'default/profile_vel': 100.0,
            'default/return_delay_time': 0.0,
            'loop_rate': 100,
            'verbose_ratio': 300,
            'pub_outdated_present_value': True,
            'pub_ratio/present': {
                'pwm': 2,
                'current': 2,
                'velocity': 2,
                'position': 2,
                'velocity_trajectory': 0,
                'position_trajectory': 0,
                'input_voltage': 29,
                'temperature': 29
            },
            'pub_ratio/status': 47,
            'pub_ratio/goal': 11,
            'pub_ratio/gain': 101,
            'pub_ratio/limit': 307,
            'pub_ratio/error': 53,
            'method/fast_read': False,
            'method/split_read': False,
            'method/split_write': True,
            'option/external_port': {
                'use': False,
                'pub_ratio/data': 10,
                'pub_ratio/mode': 100,
                'verbose/callback': False,
                'verbose/write': False,
                'verbose/read': {'raw': False, 'err': False}
            },
            'max_log_width': 6,
            'verbose/callback': True,
            'verbose/write_status': False,
            'verbose/write_goal': False,
            'verbose/write_gain': False,
            'verbose/write_limit': False,
            'verbose/read_status': {'raw': False, 'err': False},
            'verbose/read_present': {'raw': False, 'err': False},
            'verbose/read_goal': {'raw': False, 'err': False},
            'verbose/read_gain': {'raw': False, 'err': False},
            'verbose/read_limit': {'raw': False, 'err': False},
            'verbose/read_hardware_error': True,
            'dyn_comm/retry_num': 5,
            'dyn_comm/inerval_msec': 4,
            'dyn_comm/verbose': False,
            'debug': False,
            'no_use_command_line': False
        }]
    )

    # 力センサノード
    force_sensor_node = Node(
        package='force_sensor_pkg',
        executable='zef6a_sensor_driver_continuous',
        name='force_sensor_node',
        output='screen',
    )
    
    # ARマーカー推定ノード
    estimator_node = Node(
        package='aruco_slider_estimator',
        executable='estimator_node',
        name='estimator_node',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'estimator_node:=WARN'],
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
            'camera_info_file': PathJoinSubstitution([
                FindPackageShare('aruco_slider_estimator'),
                'config',
                'emeet_c960.yaml'
            ]),
            'world_frame_id': 'world',
            'slider_frame_id': 'slider_base',
            'publish_tf': True,
            'show_debug_image': True,
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
        emulate_tty=True
    )
    
    # 位置推定ノード
    extended_position_estimator_node = Node(
        package='dynamixel_linear_slider',
        executable='extended_position_estimator_node',
        name='extended_position_estimator_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'extended_position_estimator_node:=INFO']
    )
    
    # 軌道生成ノード
    trajectory_generator_node = Node(
        package='dynamixel_linear_slider',
        executable='trajectory_generator_node',
        name='trajectory_generator_node',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'trajectory_generator_node:=WARN']
    )
    
    # DynamixelHandler位置制御ノード
    dynamixel_handler_position_controller_node = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_handler_position_controller_node',
        name='dynamixel_handler_position_controller_node',
        output='screen',
        emulate_tty=True
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        dynamixel_handler_node,
        estimator_node,
        extended_position_estimator_node,
        pose_to_position_converter_node,
        trajectory_generator_node,
        dynamixel_handler_position_controller_node,
        force_sensor_node,
    ]) 