#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch引数の宣言
        DeclareLaunchArgument(
            'target_velocity',
            default_value='0.1',
            description='Target velocity in m/s'
        ),
        DeclareLaunchArgument(
            'publish_rate',
            default_value='10.0',
            description='Publish rate in Hz'
        ),
        DeclareLaunchArgument(
            'velocity_topic',
            default_value='/dynamixel/velocity_command',
            description='Velocity command topic name'
        ),
        DeclareLaunchArgument(
            'joint_state_topic',
            default_value='/joint_states',
            description='Joint state topic name'
        ),
        DeclareLaunchArgument(
            'max_velocity',
            default_value='0.5',
            description='Maximum velocity in m/s'
        ),
        DeclareLaunchArgument(
            'min_velocity',
            default_value='-0.5',
            description='Minimum velocity in m/s'
        ),
        
        # velocity_publisherノード
        Node(
            package='dynamixel_linear_slider',
            executable='velocity_publisher',
            name='dynamixel_velocity_publisher',
            output='screen',
            parameters=[{
                'target_velocity': LaunchConfiguration('target_velocity'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'velocity_topic': LaunchConfiguration('velocity_topic'),
                'joint_state_topic': LaunchConfiguration('joint_state_topic'),
                'max_velocity': LaunchConfiguration('max_velocity'),
                'min_velocity': LaunchConfiguration('min_velocity'),
            }],
            remappings=[
                ('/dynamixel/velocity_command', LaunchConfiguration('velocity_topic')),
                ('/joint_states', LaunchConfiguration('joint_state_topic')),
            ]
        )
    ]) 