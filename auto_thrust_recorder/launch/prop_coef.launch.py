from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(Node(
        package='auto_thrust_recorder',
        executable='prop_coef_finder',
        name='prop_coef_finder',
        namespace='',
        parameters=[
            {"max_thrust_percent": 40.0, "thrust_step": 0.02},
        ]
    ))
    ld.add_action(Node(
        package='force_sensor_pkg',
        executable='force_sensor_node',
        name='force_sensor_node',
        namespace=''
    ))
    
    return ld