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
        # dynamixel_handler_controllerノード
        Node(
            package='dynamixel_linear_slider',
            executable='dynamixel_handler_controller',
            name='dynamixel_handler_controller',
            output='screen',
            parameters=[{
                'id_list': [1],
                'amplitude': 100.0,  # deg/s
                'frequency': 0.1,    # Hz
                'publish_rate': 20.0, # Hz
                'control_mode': 'velocity',  # velocity, position, current_base_position
            }],
        ),
    ]) 