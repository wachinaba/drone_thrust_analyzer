#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'target_mm',
            description='Target position in mm'
        ),
        DeclareLaunchArgument(
            'tolerance',
            default_value='0.5',
            description='Position tolerance in mm'
        ),
        DeclareLaunchArgument(
            'timeout',
            default_value='30.0',
            description='Timeout in seconds'
        ),
        
        # Log launch parameters
        LogInfo(
            msg=['Launching move_to_position with: target_mm=', LaunchConfiguration('target_mm'),
                 ', tolerance=', LaunchConfiguration('tolerance'),
                 ', timeout=', LaunchConfiguration('timeout')]
        ),
        
        # Move to position node
        Node(
            package='dynamixel_leadscrew_slider',
            executable='move_to_position',
            name='move_to_position_node',
            arguments=[
                LaunchConfiguration('target_mm'),
                '--tolerance', LaunchConfiguration('tolerance'),
                '--timeout', LaunchConfiguration('timeout')
            ],
            output='screen'
        ),
    ])
