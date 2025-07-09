import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Launch Argumentsの宣言
    max_thrust_percent_arg = DeclareLaunchArgument(
        'max_thrust_percent',
        default_value='40.0',
        description='Maximum thrust percentage (e.g., 40.0)'
    )

    thrust_step_arg = DeclareLaunchArgument(
        'thrust_step',
        default_value='0.02',
        description='Thrust increment step (e.g., 0.02)'
    )

    # 必要に応じて他のパラメータも追加
    command_publish_rate_arg = DeclareLaunchArgument(
        'command_publish_rate',
        default_value='100.0',
        description='Rate at which commands are published (Hz)'
    )

    log_filename_prefix_arg = DeclareLaunchArgument(
        'log_filename_prefix',
        default_value='thrust',
        description='Prefix for the log filenames'
    )

    pause_duration_arg = DeclareLaunchArgument(
        'pause_duration',
        default_value='0.2',
        description='Duration to pause logging between thrust steps (seconds)'
    )

    recording_time_increment_arg = DeclareLaunchArgument(
        'recording_time_increment',
        default_value='1.0',
        description='Time increment for recording data (seconds)'
    )

    control_smoothing_factor_arg = DeclareLaunchArgument(
        'control_smoothing_factor',
        default_value='0.1',
        description='Factor for control smoothing'
    )

    # Launch Configurationの取得
    max_thrust_percent = LaunchConfiguration('max_thrust_percent')
    thrust_step = LaunchConfiguration('thrust_step')
    command_publish_rate = LaunchConfiguration('command_publish_rate')
    log_filename_prefix = LaunchConfiguration('log_filename_prefix')
    pause_duration = LaunchConfiguration('pause_duration')
    recording_time_increment = LaunchConfiguration('recording_time_increment')
    control_smoothing_factor = LaunchConfiguration('control_smoothing_factor')

    # ノードの定義
    prop_coef_finder_node = Node(
        package='auto_thrust_recorder',
        executable='prop_coef_finder',
        name='prop_coef_finder',
        namespace='',
        output='screen',
        parameters=[
            {
                "max_thrust_percent": max_thrust_percent,
                "thrust_step": thrust_step,
                "command_publish_rate": command_publish_rate,
                "log_filename_prefix": log_filename_prefix,
                "pause_duration": pause_duration,
                "recording_time_increment": recording_time_increment,
                "control_smoothing_factor": control_smoothing_factor
            }
        ]
    )

    force_sensor_node = Node(
        package='force_sensor_pkg',
        executable='force_sensor_node',
        name='force_sensor_node',
        namespace='',
        output='screen'
        # 必要に応じてforce_sensor_nodeのパラメータもここに追加
    )

    # Launch Descriptionの作成
    ld = LaunchDescription()

    # 宣言したLaunch ArgumentsをLaunch Descriptionに追加
    ld.add_action(max_thrust_percent_arg)
    ld.add_action(thrust_step_arg)
    ld.add_action(command_publish_rate_arg)
    ld.add_action(log_filename_prefix_arg)
    ld.add_action(pause_duration_arg)
    ld.add_action(recording_time_increment_arg)
    ld.add_action(control_smoothing_factor_arg)

    # ノードを追加
    ld.add_action(prop_coef_finder_node)
    ld.add_action(force_sensor_node)

    return ld
