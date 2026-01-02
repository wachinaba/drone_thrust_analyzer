from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    device = LaunchConfiguration('device')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            os.path.dirname(__file__), '..', 'config', 'slider_params.yaml'
        ),
        description='Path to slider params YAML'
    )

    declare_device = DeclareLaunchArgument(
        'device', default_value='/dev/ttyUSB0', description='Serial device name for Dynamixel'
    )

    handler_node = Node(
        package='dynamixel_handler',
        executable='dynamixel_handler',
        name='dynamixel_handler',
        output='screen',
        parameters=[
            # ユーザ環境の config を使うなら差し替え可
            # ここでは最低限 device をパラメータとして渡す
            {'device_name': device}
        ]
    )

    # slider_bottom (ID=2)
    controller_bottom_node = Node(
        package='dynamixel_leadscrew_slider',
        executable='leadscrew_slider_controller',
        name='leadscrew_slider_controller',
        namespace='slider_bottom',
        output='screen',
        parameters=[
            params_file,
            {'motor.id': 2},
        ],
        remappings=[
            ('/move_mm', '/slider_bottom/move_mm'),
            ('/current_position', '/slider_bottom/current_position'),
        ]
    )

    # slider_top (ID=3)
    controller_top_node = Node(
        package='dynamixel_leadscrew_slider',
        executable='leadscrew_slider_controller',
        name='leadscrew_slider_controller',
        namespace='slider_top',
        output='screen',
        parameters=[
            params_file,
            {
                'motor.id': 3,
                'homing.seek_current_ma': 800,
                'homing.seek_current_ma_max': 800,
                'homing.backoff_current_ma': 800,
            },
        ],
        remappings=[
            ('/move_mm', '/slider_top/move_mm'),
            ('/current_position', '/slider_top/current_position'),
        ]
    )

    return LaunchDescription([
        declare_params_file,
        declare_device,
        handler_node,
        controller_bottom_node,
        controller_top_node,
    ])


