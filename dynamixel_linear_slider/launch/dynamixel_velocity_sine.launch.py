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
            parameters=['../DynamixelHandler-ros2/config/config_dynamixel_handler.yaml'],
        ),
        # velocity_sine_publisherノード
        Node(
            package='dynamixel_linear_slider',
            executable='velocity_sine_publisher',
            name='velocity_sine_publisher',
            output='screen',
            parameters=[{
                'id_list': [1],
                'amplitude': 100.0,
                'frequency': 0.1,
                'publish_rate': 20.0,
            }],
        ),
    ]) 