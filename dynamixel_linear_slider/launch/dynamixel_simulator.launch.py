#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Dynamixelシミュレーションノードのlaunchファイル"""
    
    # Launch引数の定義
    motor_id_arg = DeclareLaunchArgument(
        'motor_id',
        default_value='1',
        description='DynamixelモーターのID'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='100.0',
        description='制御周波数 (Hz)'
    )
    
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='65.0',
        description='最大速度 (rpm) - XC330-T288-T仕様'
    )
    
    max_acceleration_arg = DeclareLaunchArgument(
        'max_acceleration',
        default_value='130.0',
        description='最大加速度 (rpm/s) - XC330-T288-T仕様'
    )
    
    encoder_resolution_arg = DeclareLaunchArgument(
        'encoder_resolution',
        default_value='4096',
        description='エンコーダー分解能'
    )
    
    gear_ratio_arg = DeclareLaunchArgument(
        'gear_ratio',
        default_value='1.0',
        description='ギア比'
    )
    
    # 抵抗力パラメータ
    resistance_type_arg = DeclareLaunchArgument(
        'resistance_type',
        default_value='viscous',
        description='抵抗力タイプ (none, viscous, friction, time_varying, random, combined)'
    )
    
    viscous_coefficient_arg = DeclareLaunchArgument(
        'viscous_coefficient',
        default_value='0.1',
        description='粘性抵抗係数 (N·m·s/rad)'
    )
    
    friction_coefficient_arg = DeclareLaunchArgument(
        'friction_coefficient',
        default_value='0.05',
        description='摩擦係数 (N·m)'
    )
    
    time_varying_amplitude_arg = DeclareLaunchArgument(
        'time_varying_amplitude',
        default_value='0.2',
        description='時間変化抵抗の振幅 (N·m)'
    )
    
    time_varying_frequency_arg = DeclareLaunchArgument(
        'time_varying_frequency',
        default_value='1.0',
        description='時間変化抵抗の周波数 (Hz)'
    )
    
    # スライダ位置パラメータ
    slider_total_length_arg = DeclareLaunchArgument(
        'slider_total_length',
        default_value='100.0',
        description='スライダ全長 (mm)'
    )
    
    slider_initial_position_arg = DeclareLaunchArgument(
        'slider_initial_position',
        default_value='50.0',
        description='スライダ初期位置 (mm)'
    )
    
    encoder_to_slider_ratio_arg = DeclareLaunchArgument(
        'encoder_to_slider_ratio',
        default_value='0.01',
        description='エンコーダー値とスライダ位置の変換比 (mm/encoder_unit)'
    )
    
    slider_position_topic_arg = DeclareLaunchArgument(
        'slider_position_topic',
        default_value='slider_position',
        description='スライダ位置トピック名'
    )
    
    # Dynamixelシミュレーションノード
    dynamixel_simulator_node = Node(
        package='dynamixel_linear_slider',
        executable='dynamixel_simulator_node',
        name='dynamixel_simulator_node',
        output='screen',
        parameters=[{
            'motor_id': LaunchConfiguration('motor_id'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'max_velocity': LaunchConfiguration('max_velocity'),
            'max_acceleration': LaunchConfiguration('max_acceleration'),
            'encoder_resolution': LaunchConfiguration('encoder_resolution'),
            'gear_ratio': LaunchConfiguration('gear_ratio'),
            'stall_torque': 0.92,  # XC330-T288-T仕様
            'stall_current': 0.8,   # XC330-T288-T仕様
            'no_load_speed': 65.0,  # XC330-T288-T仕様
            'no_load_current': 0.05, # XC330-T288-T仕様
            'resistance_type': LaunchConfiguration('resistance_type'),
            'viscous_coefficient': LaunchConfiguration('viscous_coefficient'),
            'friction_coefficient': LaunchConfiguration('friction_coefficient'),
            'time_varying_amplitude': LaunchConfiguration('time_varying_amplitude'),
            'time_varying_frequency': LaunchConfiguration('time_varying_frequency'),
            'random_resistance_std': 0.1,
            'slider_total_length': LaunchConfiguration('slider_total_length'),
            'slider_initial_position': LaunchConfiguration('slider_initial_position'),
            'encoder_to_slider_ratio': LaunchConfiguration('encoder_to_slider_ratio'),
            'slider_position_topic': LaunchConfiguration('slider_position_topic'),
        }],
        remappings=[
            ('/dynamixel/commands/x', '/dynamixel/commands/x'),
            ('/dynamixel/states', '/dynamixel/states'),
            ('joint_states', 'joint_states'),
            ('motor_status', 'motor_status'),
            ('slider_position', 'slider_position'),
        ]
    )
    
    return LaunchDescription([
        motor_id_arg,
        control_frequency_arg,
        max_velocity_arg,
        max_acceleration_arg,
        encoder_resolution_arg,
        gear_ratio_arg,
        resistance_type_arg,
        viscous_coefficient_arg,
        friction_coefficient_arg,
        time_varying_amplitude_arg,
        time_varying_frequency_arg,
        slider_total_length_arg,
        slider_initial_position_arg,
        encoder_to_slider_ratio_arg,
        slider_position_topic_arg,
        dynamixel_simulator_node,
    ]) 