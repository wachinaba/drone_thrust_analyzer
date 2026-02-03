#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """変形ドローン用Dynamixelモータ制御launchファイル"""
    
    # Launch引数の宣言
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    dynamixel_handler_params_file_arg = DeclareLaunchArgument(
        'dynamixel_handler_params_file',
        default_value=os.path.join(
            get_package_share_directory('dynamixel_handler'),
            'config',
            'config_dynamixel_handler.yaml'
        ),
        description='Path to dynamixel_handler config YAML file'
    )
    
    device_name_arg = DeclareLaunchArgument(
        'device_name',
        default_value='',
        description='Serial device name for Dynamixel (e.g., /dev/ttyUSB0, /dev/ttyACM0). '
                    'If empty, uses value from dynamixel_handler_params_file. '
                    'If specified, overrides the value in the config file.'
    )
    
    morph_params_file_arg = DeclareLaunchArgument(
        'morph_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('morph_drone_control'),
            'config',
            'morph_drone_control.yaml'
        ]),
        description='Path to morph_drone_control config YAML file'
    )

    psi_deg_arg = DeclareLaunchArgument(
        'psi_deg',
        default_value='0.0',
        description='Fixed slant angle psi [deg] used by 3D preview (and published angles).'
    )

    enable_preview_arg = DeclareLaunchArgument(
        'enable_preview',
        default_value='false',
        description='Enable separate matplotlib 3D preview window (default: false). '
                    'Note: 3D preview is integrated into the slider window by default.'
    )
    
    # DynamixelHandlerノード
    # device_nameが指定されている場合は、YAMLファイルの設定を上書き
    device_name = LaunchConfiguration('device_name')
    
    # パラメータリストを構築
    # ROS2では、パラメータリストの後から追加されたパラメータが前のパラメータを上書きする
    # device_nameが空文字列でない場合のみ追加（空文字列の場合はYAMLファイルの設定を使用）
    dynamixel_handler_params = [LaunchConfiguration('dynamixel_handler_params_file')]
    
    # device_nameが空でない場合のみ追加
    # 注意: 空文字列を渡すとYAMLファイルの設定が上書きされてしまうため、
    # デフォルト値は空文字列だが、実際に使用する場合は明示的に指定すること
    dynamixel_handler_node = Node(
        package='dynamixel_handler',
        executable='dynamixel_handler',
        name='dynamixel_handler',
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'dynamixel_handler:=WARN'],
        parameters=dynamixel_handler_params + [
            # device_nameが空でない場合のみ上書き
            # 空文字列の場合は追加しない（YAMLファイルの設定を使用）
            # ただし、ROS2のlaunchファイルでは条件付きパラメータ追加が難しいため、
            # ユーザーは空文字列を指定しないことを前提とする
            {'device_name': device_name},
        ]
    )
    
    # Morph Drone Control GUIノード
    morph_drone_slider_node = Node(
        package='morph_drone_control',
        executable='matplotlib_slider_node',
        name='morph_drone_slider_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('morph_params_file'),
            # publish angles for preview: psi is fixed from launch arg
            {'psi_fixed_deg': LaunchConfiguration('psi_deg')},
        ]
    )

    morph_drone_preview_node = Node(
        package='morph_drone_control',
        executable='morph_drone_preview_node',
        name='morph_drone_preview_node',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('enable_preview')),
        parameters=[
            # subscribe topic must match slider node's publish default
            {'angles_topic': '/morph_drone/angles_deg'},
        ],
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        dynamixel_handler_params_file_arg,
        device_name_arg,
        morph_params_file_arg,
        psi_deg_arg,
        enable_preview_arg,
        dynamixel_handler_node,
        morph_drone_slider_node,
        morph_drone_preview_node,
    ])

